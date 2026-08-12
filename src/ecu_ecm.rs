//! Engine Control Module (ECM) — Tier 4 Final / Stage V diesel engine.
//!
//! Models a 6-cylinder, ~220 kW / 1050 Nm diesel with:
//!   • Common Rail Direct Injection (CRDI)
//!   • Variable Geometry Turbocharger (VGT)
//!   • Exhaust Gas Recirculation (EGR)
//!   • Diesel Particulate Filter (DPF) + SCR (DEF/AdBlue)
//!   • Electronic governor (all-speed, droop or low-idle/high-idle)
//!
//! J1939 source address: 0x00 (Engine #1)
//! Transmits: EEC1(10ms), EEC2(50ms), ET1(1000ms), EFL/P1(500ms),
//!            IC1(500ms), LFE(100ms), HOURS(1000ms), DM1(1000ms)

use crate::j1939::{addr, Builder, Dtc, DtcSeverity, J1939Frame};

// ── Engine constants ─────────────────────────────────────────────────────────
pub const ECM_SA: u8 = addr::ECM_1; // 0x00
const CYLINDERS: u8 = 6;
const IDLE_RPM: f64 = 800.0;
const LO_IDLE_RPM: f64 = 700.0;
const HI_IDLE_RPM: f64 = 2300.0;
const RATED_RPM: f64 = 2200.0;
const MAX_RPM: f64 = 2600.0;
const PEAK_TORQUE_NM: f64 = 1050.0;
const PEAK_TORQUE_RPM: f64 = 1400.0;
#[allow(dead_code)]
const RATED_POWER_KW: f64 = 220.0;
const ENGINE_INERTIA_KGM2: f64 = 3.2;
const FUEL_TANK_L: f64 = 200.0;
const DEF_TANK_L: f64 = 25.0;
const DPF_REGEN_SOOT: f64 = 75.0; // % soot load triggers regen

// ── Governor Mode ────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GovernorMode {
    /// Low-idle / high-idle switch — no intermediate speeds
    LowHighIdle,
    /// All-speed (isochronous): maintains set RPM under all loads
    AllSpeed,
    /// Droop: RPM sags ~100 rpm under full load (good for PTO stability)
    Droop,
}

impl std::fmt::Display for GovernorMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GovernorMode::LowHighIdle => write!(f, "L/H IDLE"),
            GovernorMode::AllSpeed => write!(f, "ALL-SPD "),
            GovernorMode::Droop => write!(f, "DROOP   "),
        }
    }
}

// ── Fuel Map Modes ───────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FuelMap {
    Standard,
    Economy,
    Power,
}

impl std::fmt::Display for FuelMap {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FuelMap::Standard => write!(f, "STANDARD"),
            FuelMap::Economy => write!(f, "ECONOMY "),
            FuelMap::Power => write!(f, "POWER   "),
        }
    }
}

// ── Aftertreatment state ──────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AftertreatmentState {
    Normal,
    /// Active DPF regeneration (≥600°C DPF outlet)
    DpfRegen,
    /// SCR efficiency degraded (low DEF, wrong concentration)
    ScrDegraded,
    /// Severe derating — >80% soot, <2% DEF
    Derating,
    /// System fault
    Fault,
}

impl std::fmt::Display for AftertreatmentState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AftertreatmentState::Normal => write!(f, "NORMAL      "),
            AftertreatmentState::DpfRegen => write!(f, "DPF REGEN   "),
            AftertreatmentState::ScrDegraded => write!(f, "SCR DEGRADE "),
            AftertreatmentState::Derating => write!(f, "DERATING!   "),
            AftertreatmentState::Fault => write!(f, "AT FAULT    "),
        }
    }
}

// ── ECM ──────────────────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct EcuEcm {
    // ─ Identity ─────────────────────────────────────────────────────────────
    pub sa: u8,
    pub cylinders: u8,
    pub fuel_map: FuelMap,
    pub governor_mode: GovernorMode,
    pub engine_hours: f64,
    pub service_interval_h: f64,
    pub service_hours_left: f64,

    // ─ Engine Speed ──────────────────────────────────────────────────────────
    pub rpm: f64,
    pub rpm_target: f64,
    pub rpm_demand: f64, // what the operator demands
    pub rpm_limit: f64,  // current limit (derating may reduce)

    // ─ Torque ────────────────────────────────────────────────────────────────
    pub actual_torque_nm: f64,
    pub percent_load: f64,      // 0–100%
    pub actual_torque_pct: f64, // relative to peak, −125..+125
    pub demand_torque_pct: f64,
    pub driveline_torque_nm: f64, // torque demanded by drivetrain load

    // ─ Throttle ──────────────────────────────────────────────────────────────
    pub throttle_pct: f64,        // accelerator pedal position 0–100%
    pub remote_throttle_pct: f64, // from TCM/armrest (PTO speed control)
    pub active_throttle: f64,     // whichever is highest

    // ─ Fuel ──────────────────────────────────────────────────────────────────
    pub fuel_level_pct: f64,
    pub fuel_pressure_kpa: f64,
    pub fuel_rail_pressure_mpa: f64, // CRDI rail pressure (MPa)
    pub fuel_rate_lph: f64,
    pub total_fuel_l: f64,
    pub trip_fuel_l: f64,

    // ─ Temperatures ──────────────────────────────────────────────────────────
    pub coolant_temp_c: f64,
    pub oil_temp_c: f64,
    pub fuel_temp_c: f64,
    pub intake_temp_c: f64,
    pub exhaust_temp_c: f64,
    pub turbo_oil_temp_c: f64,
    pub ambient_temp_c: f64,

    // ─ Pressures / Flow ──────────────────────────────────────────────────────
    pub oil_pressure_kpa: f64,
    pub oil_level_pct: f64,
    pub coolant_pres_kpa: f64,
    pub boost_pressure_kpa: f64, // after intercooler
    pub vgt_position_pct: f64,   // 0=open, 100=closed
    pub egr_valve_pct: f64,      // 0=closed, 100=open
    pub air_filter_dp_kpa: f64,
    pub intake_manifold_kpa: f64,

    // ─ Electrical ────────────────────────────────────────────────────────────
    pub battery_v: f64,
    pub alternator_v: f64,
    pub starter_cranking: bool,

    // ─ Aftertreatment ────────────────────────────────────────────────────────
    pub def_level_pct: f64,
    pub def_quality_pct: f64, // 32.5% urea is nominal
    pub dpf_soot_pct: f64,
    pub dpf_temp_c: f64,
    pub scr_inlet_temp_c: f64,
    pub scr_efficiency_pct: f64,
    pub nox_raw_ppm: f64,
    pub nox_tailpipe_ppm: f64,
    pub aftertreatment: AftertreatmentState,
    pub regen_requested: bool,
    pub regen_inhibited: bool,

    // ─ Diagnostics ───────────────────────────────────────────────────────────
    pub active_dtcs: Vec<Dtc>,
    pub stored_dtcs: Vec<Dtc>,
    pub mil_active: bool,
    pub amber_lamp: bool,
    pub red_lamp: bool,
    pub protect_lamp: bool,

    // ─ Periodic TX timers ────────────────────────────────────────────────────
    t_eec1: f64,
    t_eec2: f64,
    t_et1: f64,
    t_efl: f64,
    t_ic1: f64,
    t_lfe: f64,
    t_hrs: f64,
    t_dm1: f64,

    // ─ Internal dynamics ─────────────────────────────────────────────────────
    rpm_error_integral: f64,
    warmup_done: bool,
}

impl Default for EcuEcm {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuEcm {
    pub fn new() -> Self {
        EcuEcm {
            sa: ECM_SA,
            cylinders: CYLINDERS,
            fuel_map: FuelMap::Standard,
            governor_mode: GovernorMode::AllSpeed,
            engine_hours: 2347.5,
            service_interval_h: 500.0,
            service_hours_left: 152.5,

            rpm: 0.0,
            rpm_target: 0.0,
            rpm_demand: 0.0,
            rpm_limit: MAX_RPM,

            actual_torque_nm: 0.0,
            percent_load: 0.0,
            actual_torque_pct: 0.0,
            demand_torque_pct: 0.0,
            driveline_torque_nm: 0.0,

            throttle_pct: 0.0,
            remote_throttle_pct: 0.0,
            active_throttle: 0.0,

            fuel_level_pct: 85.0,
            fuel_pressure_kpa: 450.0,
            fuel_rail_pressure_mpa: 180.0,
            fuel_rate_lph: 0.0,
            total_fuel_l: 0.0,
            trip_fuel_l: 0.0,

            coolant_temp_c: 20.0,
            oil_temp_c: 20.0,
            fuel_temp_c: 22.0,
            intake_temp_c: 22.0,
            exhaust_temp_c: 150.0,
            turbo_oil_temp_c: 20.0,
            ambient_temp_c: 22.0,

            oil_pressure_kpa: 0.0,
            oil_level_pct: 95.0,
            coolant_pres_kpa: 0.0,
            boost_pressure_kpa: 100.0,
            vgt_position_pct: 50.0,
            egr_valve_pct: 0.0,
            air_filter_dp_kpa: 1.2,
            intake_manifold_kpa: 100.0,

            battery_v: 12.8,
            alternator_v: 0.0,
            starter_cranking: false,

            def_level_pct: 72.0,
            def_quality_pct: 99.0,
            dpf_soot_pct: 18.0,
            dpf_temp_c: 200.0,
            scr_inlet_temp_c: 200.0,
            scr_efficiency_pct: 95.0,
            nox_raw_ppm: 0.0,
            nox_tailpipe_ppm: 0.0,
            aftertreatment: AftertreatmentState::Normal,
            regen_requested: false,
            regen_inhibited: false,

            active_dtcs: Vec::new(),
            stored_dtcs: Vec::new(),
            mil_active: false,
            amber_lamp: false,
            red_lamp: false,
            protect_lamp: false,

            t_eec1: 0.0,
            t_eec2: 0.0,
            t_et1: 0.0,
            t_efl: 0.0,
            t_ic1: 0.0,
            t_lfe: 0.0,
            t_hrs: 0.0,
            t_dm1: 0.0,

            rpm_error_integral: 0.0,
            warmup_done: false,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Diesel torque curve: flat torque plateau 1200–1600 rpm (turbocharged)
    // Returns Nm for given RPM and throttle 0-100%
    pub fn torque_at_rpm(&self, rpm: f64, throttle: f64) -> f64 {
        if rpm < 500.0 {
            return 0.0;
        }
        let t = throttle / 100.0;

        // Fuel map multiplier
        let fuel_mult = match self.fuel_map {
            FuelMap::Economy => 0.88,
            FuelMap::Standard => 1.00,
            FuelMap::Power => 1.08,
        };

        // Torque curve shape (normalised)
        let shape = if rpm < 800.0 {
            0.0
        } else if rpm < PEAK_TORQUE_RPM {
            0.65 + 0.35 * (rpm - 800.0) / (PEAK_TORQUE_RPM - 800.0)
        } else if rpm <= (RATED_RPM - 100.0) {
            // Flat plateau ± slight rise
            1.0 - 0.05 * (rpm - PEAK_TORQUE_RPM) / (RATED_RPM - PEAK_TORQUE_RPM)
        } else if rpm <= RATED_RPM {
            0.95
        } else if rpm <= MAX_RPM {
            0.95 - 0.95 * (rpm - RATED_RPM) / (MAX_RPM - RATED_RPM)
        } else {
            0.0
        };

        PEAK_TORQUE_NM * shape * t * fuel_mult
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Main ECM tick — call every simulation step.
    /// `throttle_pct` 0-100, `load_nm` is torque demanded by drivetrain.
    pub fn tick(&mut self, throttle_pct: f64, load_nm: f64, dt: f64) -> Vec<J1939Frame> {
        self.throttle_pct = throttle_pct.clamp(0.0, 100.0);
        self.active_throttle = self.throttle_pct.max(self.remote_throttle_pct);
        self.driveline_torque_nm = load_nm;

        // ─ Governor: compute target RPM ──────────────────────────────────────
        let throttle_demand_rpm =
            IDLE_RPM + (self.active_throttle / 100.0) * (self.rpm_limit - IDLE_RPM);
        let target_rpm = match self.governor_mode {
            GovernorMode::AllSpeed => throttle_demand_rpm,
            GovernorMode::Droop => throttle_demand_rpm - (load_nm / PEAK_TORQUE_NM) * 100.0,
            GovernorMode::LowHighIdle => {
                if self.active_throttle < 5.0 {
                    LO_IDLE_RPM
                } else {
                    HI_IDLE_RPM
                }
            }
        };
        self.rpm_target = target_rpm.clamp(LO_IDLE_RPM, self.rpm_limit);

        // ─ Engine dynamics (governor PI loop) ───────────────────────────────
        let available_torque = self.torque_at_rpm(self.rpm, self.active_throttle);
        let net_torque = available_torque - load_nm;
        // Angular acceleration = net_torque / J_engine
        let rpm_accel = (net_torque / ENGINE_INERTIA_KGM2) * (60.0 / (2.0 * std::f64::consts::PI));
        let rpm_err = self.rpm_target - self.rpm;
        self.rpm_error_integral = (self.rpm_error_integral + rpm_err * dt).clamp(-500.0, 500.0);
        // Governor correction (PI)
        let governor_correction = rpm_err * 0.8 + self.rpm_error_integral * 0.3;
        self.rpm += (rpm_accel * 0.05 + governor_correction) * dt;
        self.rpm = self.rpm.clamp(0.0, MAX_RPM);

        // Engine stalls if RPM drops below threshold with no throttle
        if self.rpm < 400.0 && self.active_throttle < 2.0 {
            self.rpm = 0.0;
        }

        // ─ Torque and load computation ────────────────────────────────────────
        self.actual_torque_nm = available_torque.max(0.0);
        self.percent_load = (load_nm / PEAK_TORQUE_NM * 100.0).clamp(0.0, 150.0);
        self.actual_torque_pct =
            (self.actual_torque_nm / PEAK_TORQUE_NM * 100.0 - 125.0).clamp(-125.0, 125.0);
        self.demand_torque_pct = (self.active_throttle - 125.0).clamp(-125.0, 125.0);

        // ─ Fuel rail pressure (CRDI) ─────────────────────────────────────────
        // Rail pressure: 300-2000 bar depending on RPM and load
        let rpm_norm = (self.rpm / RATED_RPM).clamp(0.2, 1.2);
        self.fuel_rail_pressure_mpa = 30.0 + rpm_norm * (self.active_throttle / 100.0) * 170.0;
        self.fuel_pressure_kpa = 400.0 + self.active_throttle * 2.0;

        // ─ Fuel consumption ──────────────────────────────────────────────────
        let power_kw = self.actual_torque_nm * self.rpm * std::f64::consts::PI / 30000.0;
        // BSFC ~210 g/kWh; diesel density 840 g/L → L/h = kW * g/kWh / (g/L)
        let bsfc = 210.0 + (1.0 - self.active_throttle / 100.0) * 90.0;
        self.fuel_rate_lph = if self.rpm > 400.0 {
            (power_kw * bsfc / 840.0).max(1.5)
        } else {
            0.0
        };
        let consumed = self.fuel_rate_lph / 3600.0 * dt;
        self.total_fuel_l += consumed;
        self.trip_fuel_l += consumed;
        self.fuel_level_pct = (self.fuel_level_pct - consumed / FUEL_TANK_L * 100.0).max(0.0);

        // ─ Thermal model ─────────────────────────────────────────────────────
        self.warmup_done = self.warmup_done || self.coolant_temp_c > 82.0;
        let load_heat = self.percent_load / 100.0;
        let tgt_coolant = 88.0 + load_heat * 6.0;
        let warm_rate = if self.rpm > 500.0 { 0.015 } else { -0.008 };
        self.coolant_temp_c += (tgt_coolant - self.coolant_temp_c) * warm_rate * dt;
        self.oil_temp_c = self.coolant_temp_c + 12.0 + load_heat * 5.0;
        self.exhaust_temp_c = 120.0 + load_heat * 520.0 * (self.rpm / RATED_RPM).min(1.0);
        self.turbo_oil_temp_c = self.exhaust_temp_c * 0.6;
        self.intake_temp_c = self.ambient_temp_c + self.boost_pressure_kpa / 100.0 * 18.0;

        // ─ Pressure model ────────────────────────────────────────────────────
        self.oil_pressure_kpa = if self.rpm > 400.0 {
            150.0 + rpm_norm * 280.0
        } else {
            0.0
        };
        self.boost_pressure_kpa = 100.0 + (self.active_throttle / 100.0) * rpm_norm * 180.0;
        self.intake_manifold_kpa = self.boost_pressure_kpa * 0.95;

        // VGT control: close vanes at low RPM/high load to build boost faster
        self.vgt_position_pct = if self.rpm < PEAK_TORQUE_RPM {
            (60.0 + load_heat * 30.0).min(95.0)
        } else {
            (30.0 + (1.0 - load_heat) * 40.0).max(10.0)
        };

        // EGR: open at part load, close at high load
        self.egr_valve_pct = if self.active_throttle < 40.0 && self.coolant_temp_c > 60.0 {
            30.0 - self.active_throttle * 0.5
        } else {
            0.0
        }
        .max(0.0);

        // ─ Electrical ────────────────────────────────────────────────────────
        self.alternator_v = if self.rpm > 900.0 {
            14.2
        } else if self.rpm > 400.0 {
            13.0
        } else {
            0.0
        };
        self.battery_v = if self.alternator_v > 13.0 {
            13.8
        } else {
            (self.battery_v - 0.001 * dt).max(11.5)
        };

        // ─ Aftertreatment ────────────────────────────────────────────────────
        self.update_aftertreatment(dt);

        // ─ Service hours ─────────────────────────────────────────────────────
        if self.rpm > 400.0 {
            self.engine_hours += dt / 3600.0;
            self.service_hours_left -= dt / 3600.0;
        }

        // ─ Fault detection → DTC generation ─────────────────────────────────
        self.run_diagnostics();

        // ─ Periodic J1939 transmission ───────────────────────────────────────
        self.t_eec1 += dt;
        self.t_eec2 += dt;
        self.t_et1 += dt;
        self.t_efl += dt;
        self.t_ic1 += dt;
        self.t_lfe += dt;
        self.t_hrs += dt;
        self.t_dm1 += dt;

        let ts = self.engine_hours; // use hours as timestamp (always increasing)
        let mut frames: Vec<J1939Frame> = Vec::new();

        if self.t_eec1 >= 0.010 {
            self.t_eec1 = 0.0;
            frames.push(Builder::eec1(
                ts,
                self.rpm,
                self.actual_torque_pct,
                self.demand_torque_pct,
                self.sa,
            ));
        }
        if self.t_eec2 >= 0.050 {
            self.t_eec2 = 0.0;
            frames.push(Builder::eec2(
                ts,
                self.active_throttle,
                self.percent_load,
                self.sa,
            ));
        }
        if self.t_et1 >= 1.000 {
            self.t_et1 = 0.0;
            frames.push(Builder::et1(
                ts,
                self.coolant_temp_c,
                self.fuel_temp_c,
                self.oil_temp_c,
                self.sa,
            ));
        }
        if self.t_efl >= 0.500 {
            self.t_efl = 0.0;
            frames.push(Builder::efl_p1(
                ts,
                self.fuel_pressure_kpa,
                self.oil_level_pct,
                self.oil_pressure_kpa,
                self.sa,
            ));
        }
        if self.t_ic1 >= 0.500 {
            self.t_ic1 = 0.0;
            frames.push(Builder::ic1(
                ts,
                self.boost_pressure_kpa,
                self.intake_temp_c,
                self.exhaust_temp_c,
                self.sa,
            ));
        }
        if self.t_lfe >= 0.100 {
            self.t_lfe = 0.0;
            frames.push(Builder::lfe(
                ts,
                self.fuel_rate_lph,
                self.active_throttle,
                self.sa,
            ));
        }
        if self.t_hrs >= 1.000 {
            self.t_hrs = 0.0;
            frames.push(Builder::hours(ts, self.engine_hours, self.sa));
        }
        if self.t_dm1 >= 1.000 {
            self.t_dm1 = 0.0;
            let (spn, fmi) = self.first_active_dtc_spn_fmi();
            frames.push(Builder::dm1(
                ts,
                self.amber_lamp,
                self.red_lamp,
                self.mil_active,
                spn,
                fmi,
                self.sa,
            ));
        }

        frames
    }

    fn update_aftertreatment(&mut self, dt: f64) {
        let load = self.percent_load / 100.0;
        let rpm_running = self.rpm > 400.0;

        // DPF soot accumulates with load, decreases during regen
        if rpm_running {
            let soot_rate = match self.aftertreatment {
                AftertreatmentState::DpfRegen => -3.0 * dt, // burn soot at 3%/s
                _ => 0.0008 * load * dt,
            };
            self.dpf_soot_pct = (self.dpf_soot_pct + soot_rate).clamp(0.0, 100.0);
        }

        // DPF temperature
        let dpf_target = if self.aftertreatment == AftertreatmentState::DpfRegen {
            620.0
        } else {
            200.0 + self.exhaust_temp_c * 0.5
        };
        self.dpf_temp_c += (dpf_target - self.dpf_temp_c) * dt * 0.05;

        // SCR inlet temperature
        self.scr_inlet_temp_c = self.dpf_temp_c * 0.85;

        // SCR efficiency depends on temperature and DEF availability
        let scr_temp_ok = self.scr_inlet_temp_c > 200.0;
        self.scr_efficiency_pct = if self.def_level_pct > 5.0 && scr_temp_ok {
            93.0 + load * 4.0
        } else if self.def_level_pct > 2.0 {
            40.0
        } else {
            5.0 // near zero DEF
        };

        // DEF consumption: ~3-5% of fuel consumed
        let def_consumed = self.fuel_rate_lph * 0.04 / 3600.0 * dt;
        self.def_level_pct = (self.def_level_pct - def_consumed / DEF_TANK_L * 100.0).max(0.0);

        // NOx
        self.nox_raw_ppm = if rpm_running {
            800.0 + load * 1200.0 * (1.0 - self.egr_valve_pct / 80.0)
        } else {
            0.0
        };
        self.nox_tailpipe_ppm = self.nox_raw_ppm * (1.0 - self.scr_efficiency_pct / 100.0);

        // Aftertreatment state transitions
        self.aftertreatment = if self.dpf_soot_pct > DPF_REGEN_SOOT && !self.regen_inhibited {
            self.regen_requested = true;
            AftertreatmentState::DpfRegen
        } else if self.dpf_soot_pct < 5.0 {
            self.regen_requested = false;
            AftertreatmentState::Normal
        } else if self.def_level_pct < 2.0 {
            AftertreatmentState::Derating
        } else if self.scr_efficiency_pct < 50.0 {
            AftertreatmentState::ScrDegraded
        } else {
            self.aftertreatment
        };
    }

    fn run_diagnostics(&mut self) {
        self.active_dtcs.retain(|d| d.active);
        self.mil_active = false;
        self.amber_lamp = false;
        self.red_lamp = false;
        self.protect_lamp = false;

        // SPN 110: Engine Coolant Temperature
        if self.coolant_temp_c > 107.0 {
            self.add_dtc(
                110,
                0,
                DtcSeverity::Red,
                "Coolant temp above normal — Red Stop",
                true,
            );
            self.red_lamp = true;
        } else if self.coolant_temp_c > 102.0 {
            self.add_dtc(
                110,
                15,
                DtcSeverity::Amber,
                "Coolant temp high — Amber Warning",
                true,
            );
            self.amber_lamp = true;
        }

        // SPN 100: Engine Oil Pressure
        if self.rpm > 600.0 && self.oil_pressure_kpa < 80.0 {
            self.add_dtc(
                100,
                1,
                DtcSeverity::Red,
                "Engine oil pressure critically low — Red Stop",
                true,
            );
            self.red_lamp = true;
        }

        // SPN 183: Fuel rate (SPN 94 = fuel delivery pressure)
        if self.fuel_pressure_kpa < 150.0 && self.rpm > 600.0 {
            self.add_dtc(
                94,
                1,
                DtcSeverity::Amber,
                "Fuel delivery pressure low — check fuel filter",
                true,
            );
            self.amber_lamp = true;
        }

        // SPN 3361: DEF Level
        if self.def_level_pct < 10.0 {
            self.add_dtc(
                3361,
                17,
                DtcSeverity::Amber,
                "DEF level low — refill before derating",
                true,
            );
            self.amber_lamp = true;
        }
        if self.def_level_pct < 2.0 {
            self.add_dtc(
                3361,
                1,
                DtcSeverity::Red,
                "DEF level critically low — power derate active",
                true,
            );
            self.red_lamp = true;
            self.rpm_limit = RATED_RPM * 0.60; // 60% derating
        } else {
            self.rpm_limit = MAX_RPM;
        }

        // SPN 3251: DPF Soot Load
        if self.dpf_soot_pct > 80.0 {
            self.add_dtc(
                3251,
                16,
                DtcSeverity::Amber,
                "DPF soot load high — manual regen required",
                true,
            );
            self.amber_lamp = true;
        }

        // SPN 247: Service interval
        if self.service_hours_left < 0.0 {
            self.add_dtc(247, 31, DtcSeverity::Mil, "Service interval overdue", true);
            self.mil_active = true;
        }

        // Propagate lamp states from DTC list
        for dtc in &self.active_dtcs {
            match dtc.severity {
                DtcSeverity::Red => self.red_lamp = true,
                DtcSeverity::Amber => self.amber_lamp = true,
                DtcSeverity::Mil => self.mil_active = true,
                DtcSeverity::Protect => self.protect_lamp = true,
            }
        }
    }

    fn add_dtc(&mut self, spn: u32, fmi: u8, sev: DtcSeverity, desc: &'static str, active: bool) {
        if !self
            .active_dtcs
            .iter()
            .any(|d| d.spn == spn && d.fmi == fmi)
        {
            // Move to stored if was previously active
            self.stored_dtcs.push(Dtc {
                spn,
                fmi,
                count: 1,
                active: false,
                desc,
                severity: sev,
            });
            if active {
                self.active_dtcs.push(Dtc {
                    spn,
                    fmi,
                    count: 1,
                    active: true,
                    desc,
                    severity: sev,
                });
            }
        }
    }

    fn first_active_dtc_spn_fmi(&self) -> (u32, u8) {
        self.active_dtcs
            .first()
            .map(|d| (d.spn, d.fmi))
            .unwrap_or((0, 0))
    }

    pub fn clear_dtcs(&mut self) {
        self.active_dtcs.clear();
        self.mil_active = false;
        self.amber_lamp = false;
        self.red_lamp = false;
        self.protect_lamp = false;
    }

    // ─ Status helpers ─────────────────────────────────────────────────────────
    pub fn power_kw(&self) -> f64 {
        self.actual_torque_nm * self.rpm * std::f64::consts::PI / 30000.0
    }
    pub fn acceleration_ms2(&self) -> f64 {
        // Simplified: torque at wheel / vehicle mass estimate
        self.actual_torque_nm * 0.003
    }
    pub fn is_running(&self) -> bool {
        self.rpm > 400.0
    }
}
