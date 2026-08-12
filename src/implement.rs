//! ISOBUS / ISO 11783-7 Implement Controller
//!
//! Covers: Rear & Front PTO, 3-Point Hitch (position / draft / mixed / float),
//! four Auxiliary Hydraulic valve banks, loader hydraulics, and Task Controller
//! interface.  Emits J1939 PGN 65093 (PTO) and PGN 65091 (Hitch) frames.

use crate::j1939::addr;
use crate::j1939::pgn;
use crate::j1939::{Builder, J1939Frame};
use std::fmt;

// ── Module-level constants ────────────────────────────────────────────────────

/// Maximum hitch movement rate [%/s]
const HITCH_MAX_RATE: f64 = 20.0;
/// PTO slip at which overload protection engages [%]
const PTO_OVERLOAD_SLIP: f64 = 15.0;
/// PTO slip at which overload protection clears [%]
const PTO_RECOVER_SLIP: f64 = 5.0;
/// Maximum flow per auxiliary bank [L/min]
const AUX_MAX_FLOW: f64 = 60.0;
/// Auxiliary circuit relief pressure [bar]
const AUX_RELIEF_BAR: f64 = 200.0;
/// Hitch draft sensor bandwidth approximation [rad/s]
const HITCH_SENSOR_BW: f64 = 2.0 * std::f64::consts::PI * 5.0;

// ── PtoMode ───────────────────────────────────────────────────────────────────

/// PTO engagement / speed mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PtoMode {
    Off,
    Std540,      // 540 RPM at rated engine speed
    Std1000,     // 1000 RPM at rated engine speed
    Economy540,  // 540 RPM at reduced engine speed (fuel economy)
    Economy1000, // 1000 RPM at reduced engine speed
}

impl PtoMode {
    /// Target PTO shaft output speed [RPM]
    pub fn target_rpm(self) -> f64 {
        match self {
            PtoMode::Off => 0.0,
            PtoMode::Std540 | PtoMode::Economy540 => 540.0,
            PtoMode::Std1000 | PtoMode::Economy1000 => 1000.0,
        }
    }

    /// Nominal engine RPM required to achieve the target PTO speed
    pub fn required_engine_rpm(self) -> f64 {
        match self {
            PtoMode::Off => 800.0,
            PtoMode::Std540 => 2100.0,
            PtoMode::Economy540 => 1450.0,
            PtoMode::Std1000 => 2100.0,
            PtoMode::Economy1000 => 1700.0,
        }
    }
}

impl fmt::Display for PtoMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PtoMode::Off => write!(f, "OFF      "),
            PtoMode::Std540 => write!(f, "540 STD  "),
            PtoMode::Economy540 => write!(f, "540 ECO  "),
            PtoMode::Std1000 => write!(f, "1000 STD "),
            PtoMode::Economy1000 => write!(f, "1000 ECO "),
        }
    }
}

// ── HitchMode ─────────────────────────────────────────────────────────────────

/// 3-point hitch control strategy
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HitchMode {
    /// Pure position setpoint tracking
    Position,
    /// Regulate draft (pull) force; adjusts depth automatically
    Draft,
    /// 50/50 blend of position and draft control
    Mixed,
    /// Free-float — implement follows ground contour, no active control
    Float,
}

impl fmt::Display for HitchMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HitchMode::Position => write!(f, "POSITION"),
            HitchMode::Draft => write!(f, "DRAFT   "),
            HitchMode::Mixed => write!(f, "MIXED   "),
            HitchMode::Float => write!(f, "FLOAT   "),
        }
    }
}

// ── ImplementType ─────────────────────────────────────────────────────────────

/// Category of the implement currently attached to the 3-point hitch or loader
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImplementType {
    Plow,
    Disc,
    Cultivator,
    Planter,
    Sprayer,
    Harvester,
    Loader,
    None,
}

impl fmt::Display for ImplementType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImplementType::Plow => write!(f, "Plow      "),
            ImplementType::Disc => write!(f, "Disc      "),
            ImplementType::Cultivator => write!(f, "Cultivator"),
            ImplementType::Planter => write!(f, "Planter   "),
            ImplementType::Sprayer => write!(f, "Sprayer   "),
            ImplementType::Harvester => write!(f, "Harvester "),
            ImplementType::Loader => write!(f, "Loader    "),
            ImplementType::None => write!(f, "None      "),
        }
    }
}

// ── ValveDirection ────────────────────────────────────────────────────────────

/// Hydraulic valve commanded flow direction
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValveDirection {
    Neutral,
    Extend,
    Retract,
}

impl fmt::Display for ValveDirection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValveDirection::Neutral => write!(f, "NEUT"),
            ValveDirection::Extend => write!(f, "EXT "),
            ValveDirection::Retract => write!(f, "RET "),
        }
    }
}

// ── HydraulicBank ─────────────────────────────────────────────────────────────

/// One auxiliary hydraulic valve bank (remote hydraulics)
#[derive(Debug, Clone)]
pub struct HydraulicBank {
    pub id: u8,
    pub direction: ValveDirection,
    pub flow_lpm: f64,
    pub pressure_bar: f64,
    pub max_flow_lpm: f64,
    pub max_pressure_bar: f64,
    pub engaged: bool,
}

impl HydraulicBank {
    pub fn new(id: u8) -> Self {
        HydraulicBank {
            id,
            direction: ValveDirection::Neutral,
            flow_lpm: 0.0,
            pressure_bar: 0.0,
            max_flow_lpm: AUX_MAX_FLOW,
            max_pressure_bar: AUX_RELIEF_BAR,
            engaged: false,
        }
    }

    pub fn update(&mut self, supply_pressure_bar: f64, dt: f64) {
        if !self.engaged || self.direction == ValveDirection::Neutral {
            // Bleed back to tank when valve is neutral or disengaged
            self.flow_lpm = (self.flow_lpm - 15.0 * dt).max(0.0);
            self.pressure_bar = (self.pressure_bar - 25.0 * dt).max(0.0);
        } else {
            // Q ∝ √ΔP (simplified orifice equation near operating point)
            self.pressure_bar = (supply_pressure_bar * 0.95).min(self.max_pressure_bar);
            self.flow_lpm = self.max_flow_lpm * (self.pressure_bar / self.max_pressure_bar).sqrt();
        }
    }
}

// ── ImplementControl ──────────────────────────────────────────────────────────

/// ISOBUS implement controller — manages PTO, 3-pt hitch, and aux hydraulics
#[derive(Debug, Clone)]
pub struct ImplementControl {
    // ─ PTO System ────────────────────────────────────────────
    pub pto_rear_enabled: bool,
    pub pto_front_enabled: bool,
    pub pto_rear_rpm: f64, // actual output shaft speed [RPM]
    pub pto_front_rpm: f64,
    pub pto_target_rpm: f64, // 540 or 1000 per mode
    pub pto_mode: PtoMode,
    pub pto_shaft_torque_nm: f64, // torque transmitted to implement [Nm]
    pub pto_slip_pct: f64,        // shaft slip relative to target [%]

    // ─ 3-Point Hitch (rear) ───────────────────────────────────
    pub hitch_position_pct: f64, // 0 = fully lowered, 100 = fully raised
    pub hitch_target_pct: f64,
    pub hitch_draft_force_kn: f64,      // measured draft force [kN]
    pub hitch_ground_pressure_kpa: f64, // soil contact pressure [kPa]
    pub hitch_control_mode: HitchMode,
    pub hitch_moving: bool,
    pub hitch_speed_pct_per_s: f64, // current hitch movement rate [%/s]

    // ─ Auxiliary Hydraulic Valves (4 banks) ──────────────────
    pub aux_banks: [HydraulicBank; 4],

    // ─ Remote Hydraulic (loader / front attachment) ───────────
    pub loader_lift_position_pct: f64,
    pub loader_tilt_position_pct: f64,
    pub loader_pressure_bar: f64,

    // ─ Attached implement ─────────────────────────────────────
    pub implement_attached: Option<ImplementType>,
    pub implement_width_m: f64,
    pub implement_working_depth_cm: f64,
    pub isobus_connected: bool,
    pub task_controller_active: bool,

    // ─ Internal state (pub per crate convention) ──────────────
    pub overload_protection_active: bool,
    pub hitch_draft_target_kn: f64,
    pub pto_spin_up_timer: f64,
}

impl ImplementControl {
    pub fn new() -> Self {
        ImplementControl {
            pto_rear_enabled: false,
            pto_front_enabled: false,
            pto_rear_rpm: 0.0,
            pto_front_rpm: 0.0,
            pto_target_rpm: 540.0,
            pto_mode: PtoMode::Off,
            pto_shaft_torque_nm: 0.0,
            pto_slip_pct: 0.0,

            hitch_position_pct: 100.0,
            hitch_target_pct: 100.0,
            hitch_draft_force_kn: 0.0,
            hitch_ground_pressure_kpa: 0.0,
            hitch_control_mode: HitchMode::Position,
            hitch_moving: false,
            hitch_speed_pct_per_s: 0.0,

            aux_banks: [
                HydraulicBank::new(0),
                HydraulicBank::new(1),
                HydraulicBank::new(2),
                HydraulicBank::new(3),
            ],

            loader_lift_position_pct: 0.0,
            loader_tilt_position_pct: 50.0,
            loader_pressure_bar: 0.0,

            implement_attached: None,
            implement_width_m: 3.0,
            implement_working_depth_cm: 0.0,
            isobus_connected: false,
            task_controller_active: false,

            overload_protection_active: false,
            hitch_draft_target_kn: 8.0,
            pto_spin_up_timer: 0.0,
        }
    }

    /// Advance simulation by `dt` seconds.
    pub fn update(&mut self, throttle: f64, ground_speed_kmh: f64, dt: f64) {
        self.update_pto(throttle, dt);
        self.update_hitch(ground_speed_kmh, dt);
        self.update_aux_hydraulics(dt);
    }

    // ── PTO dynamics ─────────────────────────────────────────────────────────

    fn update_pto(&mut self, throttle: f64, dt: f64) {
        // Coast-down when disabled
        if self.pto_mode == PtoMode::Off || !self.pto_rear_enabled {
            let drag = 8.0 + self.pto_rear_rpm * 0.012;
            self.pto_rear_rpm = (self.pto_rear_rpm - drag * dt).max(0.0);
            self.pto_front_rpm = (self.pto_front_rpm - 12.0 * dt).max(0.0);
            self.pto_shaft_torque_nm = (self.pto_shaft_torque_nm - 50.0 * dt).max(0.0);
            self.pto_slip_pct = 0.0;
            if self.pto_rear_rpm < 1.0 {
                self.overload_protection_active = false;
            }
            return;
        }

        let target = self.pto_mode.target_rpm();
        self.pto_target_rpm = target;

        // Implement load factor — heavier implements demand more torque
        let load_k = match self.implement_attached {
            Some(ImplementType::Harvester) => 0.80,
            Some(ImplementType::Plow) => 0.75,
            Some(ImplementType::Disc) => 0.60,
            Some(ImplementType::Cultivator) => 0.45,
            Some(ImplementType::Planter) => 0.30,
            Some(ImplementType::Sprayer) => 0.12,
            _ => 0.08,
        };
        let desired_torque = 150.0 + load_k * 850.0 * throttle;

        // Overload protection: hysteresis on slip threshold
        if self.pto_slip_pct > PTO_OVERLOAD_SLIP {
            self.overload_protection_active = true;
        } else if self.pto_slip_pct < PTO_RECOVER_SLIP {
            self.overload_protection_active = false;
        }
        let effective_torque = if self.overload_protection_active {
            desired_torque * 0.25 // severe de-rate to allow slip recovery
        } else {
            desired_torque
        };

        // First-order speed dynamics: acceleration from speed error minus torque drag
        let speed_error = target - self.pto_rear_rpm;
        let accel = (speed_error * 4.0 - effective_torque * 0.035).clamp(-350.0, 700.0);
        self.pto_rear_rpm = (self.pto_rear_rpm + accel * dt).clamp(0.0, target * 1.15);

        // Slip is the normalised speed deficit relative to no-load target
        self.pto_slip_pct = if target > 1.0 {
            ((target - self.pto_rear_rpm) / target * 100.0).clamp(0.0, 100.0)
        } else {
            0.0
        };
        // Shaft torque is slightly reduced under slip (clutch modulation)
        self.pto_shaft_torque_nm = effective_torque * (1.0 - self.pto_slip_pct * 0.005);

        // Front PTO: driven from same shaft at a fixed reduction ratio
        self.pto_front_rpm = if self.pto_front_enabled {
            self.pto_rear_rpm * 0.72
        } else {
            (self.pto_front_rpm - 12.0 * dt).max(0.0)
        };

        self.pto_spin_up_timer += dt;
    }

    // ── 3-Point Hitch ────────────────────────────────────────────────────────

    fn update_hitch(&mut self, ground_speed_kmh: f64, dt: f64) {
        // Float mode: no servo action, let implement ride on ground
        if self.hitch_control_mode == HitchMode::Float {
            self.hitch_moving = false;
            self.hitch_speed_pct_per_s = 0.0;
            return;
        }

        let pos_error = self.hitch_target_pct - self.hitch_position_pct;
        let draft_error = self.hitch_draft_target_kn - self.hitch_draft_force_kn;

        // Proportional control output for each mode
        let speed = match self.hitch_control_mode {
            HitchMode::Position => pos_error * 6.0,
            // Raise hitch when draft exceeds target (negative draft error → lower)
            HitchMode::Draft => -draft_error * 3.0,
            HitchMode::Mixed => pos_error * 3.0 - draft_error * 1.5,
            HitchMode::Float => 0.0,
        };
        self.hitch_speed_pct_per_s = speed.clamp(-HITCH_MAX_RATE, HITCH_MAX_RATE);
        self.hitch_moving = self.hitch_speed_pct_per_s.abs() > 0.5;
        self.hitch_position_pct =
            (self.hitch_position_pct + self.hitch_speed_pct_per_s * dt).clamp(0.0, 100.0);

        // Draft force model: depends on working depth, width, implement, and speed
        let depth_ratio = (1.0 - self.hitch_position_pct / 100.0).max(0.0);
        let speed_factor = (ground_speed_kmh / 8.0).clamp(0.0, 2.0);
        let implement_k = match self.implement_attached {
            Some(ImplementType::Plow) => 14.0,
            Some(ImplementType::Disc) => 9.0,
            Some(ImplementType::Cultivator) => 5.5,
            Some(ImplementType::Planter) => 3.0,
            _ => 1.0,
        };
        let width_factor = (self.implement_width_m / 3.0).max(0.1);
        let target_draft = depth_ratio * speed_factor * implement_k * width_factor;
        // Low-pass filter models the draft sensor (≈5 Hz bandwidth)
        let tau = HITCH_SENSOR_BW * dt;
        self.hitch_draft_force_kn += (target_draft - self.hitch_draft_force_kn) * tau.min(1.0);

        // Ground pressure: weight bearing on soil when implement is fully lowered
        let mass_kg = self.implement_width_m * 180.0; // ~180 kg per metre of working width
        self.hitch_ground_pressure_kpa = if self.hitch_position_pct < 5.0 {
            let contact_area_m2 = self.implement_width_m * 0.25 + 0.01;
            (mass_kg * 9.81 / 1000.0) / contact_area_m2 // kPa
        } else {
            0.0
        };
    }

    // ── Auxiliary Hydraulics ─────────────────────────────────────────────────

    fn update_aux_hydraulics(&mut self, dt: f64) {
        // Main circuit assumed to supply 175 bar when engine is running
        let supply_bar = 175.0;
        for bank in &mut self.aux_banks {
            bank.update(supply_bar, dt);
        }

        // Bank 0 → loader lift cylinders
        if self.aux_banks[0].engaged {
            let delta = match self.aux_banks[0].direction {
                ValveDirection::Extend => 18.0 * dt,
                ValveDirection::Retract => -18.0 * dt,
                ValveDirection::Neutral => 0.0,
            };
            self.loader_lift_position_pct =
                (self.loader_lift_position_pct + delta).clamp(0.0, 100.0);
        }

        // Bank 1 → loader/bucket tilt cylinder
        if self.aux_banks[1].engaged {
            let delta = match self.aux_banks[1].direction {
                ValveDirection::Extend => 25.0 * dt,
                ValveDirection::Retract => -25.0 * dt,
                ValveDirection::Neutral => 0.0,
            };
            self.loader_tilt_position_pct =
                (self.loader_tilt_position_pct + delta).clamp(0.0, 100.0);
        }

        let active_p = if self.aux_banks[0].engaged || self.aux_banks[1].engaged {
            self.aux_banks[0]
                .pressure_bar
                .max(self.aux_banks[1].pressure_bar)
        } else {
            0.0
        };
        // Bleed loader circuit when nothing is active
        self.loader_pressure_bar += (active_p - self.loader_pressure_bar) * (5.0 * dt).min(1.0);
    }

    // ── Queries ───────────────────────────────────────────────────────────────

    /// Torque demand placed on the engine by all active PTO shafts [Nm]
    pub fn pto_torque_demand(&self) -> f64 {
        if !self.pto_rear_enabled {
            return 0.0;
        }
        let front_add = if self.pto_front_enabled {
            self.pto_shaft_torque_nm * 0.30
        } else {
            0.0
        };
        self.pto_shaft_torque_nm + front_add
    }

    // ── J1939 output ─────────────────────────────────────────────────────────

    /// Build J1939 frames for this controller (typical period 100 ms)
    pub fn generate_j1939_frames(&self, ts: f64) -> Vec<J1939Frame> {
        let mut frames = Vec::new();

        // PGN 65093 — PTO Information
        frames.push(Builder::pto(
            ts,
            self.pto_rear_rpm,
            self.pto_rear_enabled,
            addr::IMPLEMENT,
        ));

        // PGN 65091 — Hitch & PTO Commands
        // Bytes: [0] actual position raw, [1] target position raw,
        //        [2] control mode, [3] draft force raw, [4–7] reserved
        {
            let pos_raw = (self.hitch_position_pct / 0.4).clamp(0.0, 250.0) as u8;
            let target_raw = (self.hitch_target_pct / 0.4).clamp(0.0, 250.0) as u8;
            let mode_byte = match self.hitch_control_mode {
                HitchMode::Position => 0x00,
                HitchMode::Draft => 0x01,
                HitchMode::Mixed => 0x02,
                HitchMode::Float => 0x03,
            };
            let draft_raw = (self.hitch_draft_force_kn * 10.0).clamp(0.0, 250.0) as u8;
            let data = [
                pos_raw, target_raw, mode_byte, draft_raw, 0xFF, 0xFF, 0xFF, 0xFF,
            ];
            frames.push(J1939Frame::from_raw(
                ts,
                J1939Frame::build_id(6, pgn::HITCH, addr::HITCH, 0xFF),
                &data,
            ));
        }

        frames
    }
}

impl Default for ImplementControl {
    fn default() -> Self {
        ImplementControl::new()
    }
}
