//! Hydraulic Control Module (HCM) — SA 0x1E.
//!
//! Controls the entire hydraulic system on a heavy machine:
//!   • Variable-displacement axial-piston pump (load-sensing)
//!   • Main system pressure regulation via PRV
//!   • 3-point hitch electro-hydraulic cylinder control
//!   • PTO wet clutch hydraulic engagement
//!   • 4× remote auxiliary spool valve banks
//!   • Brake charge accumulator
//!   • Hydraulic-to-oil cooling circuit
//!
//! J1939 SA: 0x1E. Transmits proprietary status heartbeat (100 ms).

use crate::j1939::{addr, pgn, J1939Frame};

pub const HCM_SA: u8 = addr::HITCH; // 0x1E

// ── Pump Mode ────────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PumpMode {
    /// Standby: zero displacement, only replenish charge pressure
    Standby,
    /// Load-sensing: displacement set to match load demand + margin (~ΔP 25 bar)
    LoadSensing,
    /// Pressure-limiting: displacement reduced to hold system PRV
    PressureLimit,
}

impl std::fmt::Display for PumpMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PumpMode::Standby => write!(f, "STANDBY  "),
            PumpMode::LoadSensing => write!(f, "LOAD-SNS "),
            PumpMode::PressureLimit => write!(f, "PRV LIM  "),
        }
    }
}

// ── Hydraulic Actuator (cylinder) ─────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct HydActuator {
    pub name: &'static str,
    pub bore_mm: f64,       // cylinder bore
    pub rod_mm: f64,        // rod diameter
    pub stroke_mm: f64,     // full stroke
    pub position_mm: f64,   // current extension (0=fully retracted)
    pub velocity_mm_s: f64, // positive = extending
    pub pres_a_bar: f64,    // cap-end pressure
    pub pres_b_bar: f64,    // rod-end pressure
    pub force_kn: f64,      // net push force (positive = extending)
    pub valve_cmd: f64,     // −1 retract, 0 hold, +1 extend
    pub leakage_cc_m: f64,  // internal leakage cm³/min
}

impl HydActuator {
    pub fn new(name: &'static str, bore_mm: f64, rod_mm: f64, stroke_mm: f64) -> Self {
        HydActuator {
            name,
            bore_mm,
            rod_mm,
            stroke_mm,
            position_mm: 0.0,
            velocity_mm_s: 0.0,
            pres_a_bar: 0.0,
            pres_b_bar: 0.0,
            force_kn: 0.0,
            valve_cmd: 0.0,
            leakage_cc_m: 0.5,
        }
    }

    /// Area of each side in cm²
    pub fn area_cap_cm2(&self) -> f64 {
        std::f64::consts::PI * (self.bore_mm / 20.0).powi(2)
    }
    pub fn area_rod_cm2(&self) -> f64 {
        self.area_cap_cm2() - std::f64::consts::PI * (self.rod_mm / 20.0).powi(2)
    }

    /// Update given supply pressure and flow from the pump. Returns flow consumed (L/min).
    pub fn update(&mut self, supply_bar: f64, dt: f64) -> f64 {
        if self.valve_cmd.abs() < 0.01 {
            // Valve closed: hold position, pressures bleed through leakage
            self.pres_a_bar = (self.pres_a_bar - 2.0 * dt).max(0.0);
            self.pres_b_bar = (self.pres_b_bar - 2.0 * dt).max(0.0);
            self.velocity_mm_s = 0.0;
            return self.leakage_cc_m / 1000.0;
        }

        let cmd = self.valve_cmd.clamp(-1.0, 1.0);
        let extending = cmd > 0.0;

        // Pressure at active side ≈ supply × valve-opening factor
        if extending {
            self.pres_a_bar = supply_bar * cmd.abs() * 0.92;
            self.pres_b_bar =
                (self.pres_a_bar * self.area_cap_cm2() / self.area_rod_cm2().max(0.01)) * 0.15;
        } else {
            self.pres_b_bar = supply_bar * cmd.abs() * 0.92;
            self.pres_a_bar = self.pres_b_bar * 0.1;
        }

        // Net force
        let f_cap = self.pres_a_bar * self.area_cap_cm2() * 10.0; // N
        let f_rod = self.pres_b_bar * self.area_rod_cm2() * 10.0;
        self.force_kn = (f_cap - f_rod) / 1000.0;

        // Velocity from flow-through valve (orifice model Q = Cv × cmd × √ΔP)
        let dp = (supply_bar
            - if extending {
                self.pres_b_bar
            } else {
                self.pres_a_bar
            })
        .max(0.0);
        let flow_lpm = 40.0 * cmd.abs() * dp.sqrt() / 10.0; // simplified Cv
        let area_cm2 = if extending {
            self.area_cap_cm2()
        } else {
            self.area_rod_cm2()
        };
        self.velocity_mm_s = flow_lpm * 1000.0 / 60.0 / area_cm2 * cmd.signum();

        // Integrate position — hard stops at 0 and stroke
        self.position_mm = (self.position_mm + self.velocity_mm_s * dt).clamp(0.0, self.stroke_mm);

        // Stop at end-of-stroke
        if (self.position_mm <= 0.0 && self.velocity_mm_s < 0.0)
            || (self.position_mm >= self.stroke_mm && self.velocity_mm_s > 0.0)
        {
            self.velocity_mm_s = 0.0;
        }

        flow_lpm.abs()
    }

    pub fn position_pct(&self) -> f64 {
        self.position_mm / self.stroke_mm.max(1.0) * 100.0
    }
}

// ── HCM ──────────────────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct EcuHcm {
    pub sa: u8,

    // ─ Pump ─────────────────────────────────────────────────────────────────
    pub pump_displacement_cc: f64, // cm³/rev (max for this pump)
    pub pump_actual_cc: f64,       // current displacement
    pub pump_mode: PumpMode,
    pub pump_rpm: f64,
    pub pump_flow_lpm: f64,   // actual output flow
    pub pump_efficiency: f64, // volumetric efficiency

    // ─ System pressures ──────────────────────────────────────────────────────
    pub system_pressure_bar: f64,
    pub relief_pressure_bar: f64, // main PRV setting
    pub ls_signal_bar: f64,       // load-sensing signal from highest load
    pub charge_pres_bar: f64,     // charge circuit (brake / control pilot)
    pub return_pres_bar: f64,     // return line / back-pressure
    pub pilot_pres_bar: f64,      // control pilot supply

    // ─ Fluid condition ───────────────────────────────────────────────────────
    pub fluid_temp_c: f64,
    pub fluid_level_pct: f64,
    pub filter_dp_bar: f64, // filter differential pressure
    pub filter_bypass_open: bool,

    // ─ Actuators ─────────────────────────────────────────────────────────────
    pub hitch_cylinder: HydActuator, // 3-point rear hitch main
    pub loader_lift: HydActuator,    // front loader lift
    pub loader_tilt: HydActuator,    // bucket tilt

    // ─ 4× Aux remote valves (spool position −1..1) ───────────────────────────
    pub aux_valve: [f64; 4], // valve command −1=retract, +1=extend
    pub aux_flow_lpm: [f64; 4],
    pub aux_pres_bar: [f64; 4],

    // ─ Brake accumulator ─────────────────────────────────────────────────────
    pub brake_acc_pres_bar: f64,
    pub brake_acc_charged: bool,

    // ─ Alarms ────────────────────────────────────────────────────────────────
    pub alarm_high_temp: bool,
    pub alarm_low_level: bool,
    pub alarm_filter: bool,
    pub alarm_low_pressure: bool,

    // ─ Power ─────────────────────────────────────────────────────────────────
    pub hydraulic_power_kw: f64,
    pub total_flow_demand_lpm: f64,

    // ─ TX timer ──────────────────────────────────────────────────────────────
    t_hb: f64,
    sim_time_s: f64,
}

impl EcuHcm {
    pub fn new() -> Self {
        EcuHcm {
            sa: HCM_SA,
            pump_displacement_cc: 100.0, // 100 cc/rev (large agricultural pump)
            pump_actual_cc: 0.0,
            pump_mode: PumpMode::Standby,
            pump_rpm: 0.0,
            pump_flow_lpm: 0.0,
            pump_efficiency: 0.92,
            system_pressure_bar: 0.0,
            relief_pressure_bar: 230.0, // 230 bar PRV
            ls_signal_bar: 0.0,
            charge_pres_bar: 0.0,
            return_pres_bar: 5.0,
            pilot_pres_bar: 0.0,
            fluid_temp_c: 25.0,
            fluid_level_pct: 92.0,
            filter_dp_bar: 0.8,
            filter_bypass_open: false,
            hitch_cylinder: HydActuator::new("HITCH", 100.0, 50.0, 400.0),
            loader_lift: HydActuator::new("LOADER-LIFT", 80.0, 40.0, 800.0),
            loader_tilt: HydActuator::new("LOADER-TILT", 70.0, 35.0, 300.0),
            aux_valve: [0.0; 4],
            aux_flow_lpm: [0.0; 4],
            aux_pres_bar: [0.0; 4],
            brake_acc_pres_bar: 0.0,
            brake_acc_charged: false,
            alarm_high_temp: false,
            alarm_low_level: false,
            alarm_filter: false,
            alarm_low_pressure: false,
            hydraulic_power_kw: 0.0,
            total_flow_demand_lpm: 0.0,
            t_hb: 0.0,
            sim_time_s: 0.0,
        }
    }

    /// Main HCM tick. `engine_rpm` from ECM, `hitch_cmd` from implement controller (−1..+1).
    pub fn tick(
        &mut self,
        engine_rpm: f64,
        hitch_cmd: f64,
        loader_lift_cmd: f64,
        loader_tilt_cmd: f64,
        dt: f64,
    ) -> Vec<J1939Frame> {
        self.sim_time_s += dt;
        self.pump_rpm = engine_rpm * 1.12; // pump driven at 1.12× engine (PTO shaft)

        // ─ Determine load-sensing demand ─────────────────────────────────────
        self.hitch_cylinder.valve_cmd = hitch_cmd.clamp(-1.0, 1.0);
        self.loader_lift.valve_cmd = loader_lift_cmd.clamp(-1.0, 1.0);
        self.loader_tilt.valve_cmd = loader_tilt_cmd.clamp(-1.0, 1.0);

        // Peak load pressure among active actuators is the LS signal
        let actuator_pressures = [
            self.hitch_cylinder
                .pres_a_bar
                .max(self.hitch_cylinder.pres_b_bar),
            self.loader_lift.pres_a_bar.max(self.loader_lift.pres_b_bar),
            self.loader_tilt.pres_a_bar.max(self.loader_tilt.pres_b_bar),
        ];
        self.ls_signal_bar = actuator_pressures.iter().cloned().fold(0.0_f64, f64::max);
        // Add aux valves to LS calculation
        for &cmd in self.aux_valve.iter() {
            if cmd.abs() > 0.01 {
                self.ls_signal_bar = self.ls_signal_bar.max(100.0 + cmd.abs() * 100.0);
            }
        }

        // ─ Load-sensing pump control ──────────────────────────────────────────
        // Target: system = LS + 25 bar margin (Δp)
        let target_sys_pres = (self.ls_signal_bar + 25.0).min(self.relief_pressure_bar);

        // Pump displacement tracks demand
        if self.pump_rpm > 400.0 {
            let _max_flow = self.pump_rpm * self.pump_displacement_cc / 1000.0; // L/min
            let demand_ratio = (target_sys_pres / self.relief_pressure_bar).clamp(0.0, 1.0);
            let target_disp = self.pump_displacement_cc * demand_ratio;
            self.pump_actual_cc += (target_disp - self.pump_actual_cc) * dt * 8.0;
            self.pump_actual_cc = self.pump_actual_cc.clamp(0.0, self.pump_displacement_cc);
            self.pump_flow_lpm =
                self.pump_rpm * self.pump_actual_cc / 1000.0 * self.pump_efficiency;
            self.pump_mode = if demand_ratio < 0.05 {
                PumpMode::Standby
            } else if self.system_pressure_bar >= self.relief_pressure_bar * 0.98 {
                PumpMode::PressureLimit
            } else {
                PumpMode::LoadSensing
            };
        } else {
            self.pump_flow_lpm = 0.0;
            self.pump_actual_cc = 0.0;
            self.pump_mode = PumpMode::Standby;
        }

        // ─ System pressure: flow → pressure through restrictor + PRV ─────────
        let flow_excess = self.pump_flow_lpm - self.total_flow_demand_lpm;
        self.system_pressure_bar += flow_excess * 0.5 * dt;
        self.system_pressure_bar = self
            .system_pressure_bar
            .clamp(0.0, self.relief_pressure_bar);
        if self.system_pressure_bar >= self.relief_pressure_bar {
            self.system_pressure_bar = self.relief_pressure_bar;
        }

        // ─ Update actuators ──────────────────────────────────────────────────
        let sp = self.system_pressure_bar;
        let q_hitch = self.hitch_cylinder.update(sp, dt);
        let q_loader = self.loader_lift.update(sp, dt);
        let q_tilt = self.loader_tilt.update(sp, dt);

        // ─ Auxiliary valves ──────────────────────────────────────────────────
        let mut q_aux = 0.0;
        for i in 0..4 {
            let cmd = self.aux_valve[i];
            if cmd.abs() > 0.01 {
                self.aux_pres_bar[i] = sp * cmd.abs() * 0.90;
                self.aux_flow_lpm[i] = 40.0 * cmd.abs() * (sp / self.relief_pressure_bar).sqrt();
                q_aux += self.aux_flow_lpm[i];
            } else {
                self.aux_pres_bar[i] = (self.aux_pres_bar[i] - 10.0 * dt).max(0.0);
                self.aux_flow_lpm[i] = 0.0;
            }
        }

        self.total_flow_demand_lpm = q_hitch + q_loader + q_tilt + q_aux;

        // ─ Charge circuit (brake accumulator) ────────────────────────────────
        if self.pump_flow_lpm > 5.0 {
            self.brake_acc_pres_bar = (self.brake_acc_pres_bar + 20.0 * dt).min(180.0);
        }
        self.brake_acc_charged = self.brake_acc_pres_bar > 120.0;
        self.charge_pres_bar = self.brake_acc_pres_bar * 0.5;
        self.pilot_pres_bar = (self.system_pressure_bar * 0.1).min(40.0);

        // ─ Thermal model ─────────────────────────────────────────────────────
        // Heat = (pressure × flow) × (1 − efficiency) + heat from PRV bypass
        let kw_in = self.system_pressure_bar * self.pump_flow_lpm / 600.0;
        let kw_useful = self.total_flow_demand_lpm * sp / 600.0;
        let heat_kw = (kw_in - kw_useful).max(0.0);
        self.hydraulic_power_kw = kw_useful;
        let tgt_temp = 45.0 + heat_kw * 1.5;
        self.fluid_temp_c += (tgt_temp - self.fluid_temp_c) * dt * 0.008;

        // ─ Filter ────────────────────────────────────────────────────────────
        // ΔP rises with flow and viscosity (higher at cold start)
        let viscosity_factor = (60.0 / self.fluid_temp_c.max(10.0)).sqrt();
        self.filter_dp_bar = 0.5 + self.pump_flow_lpm * 0.015 * viscosity_factor;
        self.filter_bypass_open = self.filter_dp_bar > 6.0;

        // ─ Alarms ────────────────────────────────────────────────────────────
        self.alarm_high_temp = self.fluid_temp_c > 100.0;
        self.alarm_low_level = self.fluid_level_pct < 20.0;
        self.alarm_filter = self.filter_bypass_open;
        self.alarm_low_pressure =
            self.system_pressure_bar < 50.0 && self.total_flow_demand_lpm > 5.0;

        // ─ J1939 heartbeat (100 ms, Proprietary B) ────────────────────────────
        self.t_hb += dt;
        let mut frames: Vec<J1939Frame> = Vec::new();
        if self.t_hb >= 0.100 {
            self.t_hb = 0.0;
            let mut data = [0u8; 8];
            data[0] = (self.system_pressure_bar / 2.0).clamp(0.0, 255.0) as u8;
            data[1] = (self.fluid_temp_c + 40.0).clamp(0.0, 255.0) as u8;
            data[2] = (self.pump_flow_lpm / 2.0).clamp(0.0, 255.0) as u8;
            data[3] = (if self.alarm_high_temp { 0x01 } else { 0 })
                | (if self.alarm_low_level { 0x02 } else { 0 })
                | (if self.alarm_filter { 0x04 } else { 0 })
                | (if self.brake_acc_charged { 0x08 } else { 0 });
            data[4] = (self.hitch_cylinder.position_pct() * 2.55) as u8;
            let raw_id = J1939Frame::build_id(6, pgn::HITCH, self.sa, 0xFF);
            frames.push(J1939Frame::from_raw(self.sim_time_s, raw_id, &data));
        }
        frames
    }

    // ─ Convenience setters ────────────────────────────────────────────────────
    pub fn set_hitch_cmd(&mut self, cmd: f64) {
        self.hitch_cylinder.valve_cmd = cmd.clamp(-1.0, 1.0);
    }
    pub fn set_loader_lift_cmd(&mut self, cmd: f64) {
        self.loader_lift.valve_cmd = cmd.clamp(-1.0, 1.0);
    }
    pub fn set_loader_tilt_cmd(&mut self, cmd: f64) {
        self.loader_tilt.valve_cmd = cmd.clamp(-1.0, 1.0);
    }
    pub fn set_aux_cmd(&mut self, bank: usize, cmd: f64) {
        if bank < 4 {
            self.aux_valve[bank] = cmd.clamp(-1.0, 1.0);
        }
    }
}
