//! Heavy Machinery Engine Control Module (ECM)
//! Models a Tier 4 / Stage V diesel engine (6-cylinder, ~200 kW)

use crate::j1939::{Dtc, DtcSeverity};

// Engine configuration constants
const IDLE_RPM: f64 = 800.0;
const RATED_RPM: f64 = 2200.0;
const MAX_RPM: f64 = 2600.0;
#[allow(dead_code)]
const REDLINE_RPM: f64 = 2400.0;
const PEAK_TORQUE_NM: f64 = 1000.0; // ~900 Nm at 1400 RPM
const FUEL_TANK_L: f64 = 200.0;
const NUM_CYLINDERS: u8 = 6;

/// Aftertreatment / DEF / DPF state
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AftertreatmentState {
    Normal,
    Regenerating,
    HighSoot,
    Fault,
}

impl std::fmt::Display for AftertreatmentState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AftertreatmentState::Normal => write!(f, "NORMAL    "),
            AftertreatmentState::Regenerating => write!(f, "REGEN     "),
            AftertreatmentState::HighSoot => write!(f, "HIGH SOOT "),
            AftertreatmentState::Fault => write!(f, "FAULT     "),
        }
    }
}

/// Governor mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GovernorMode {
    Low,
    High,
    Droop,
    All,
}

impl std::fmt::Display for GovernorMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GovernorMode::Low => write!(f, "LOW IDLE"),
            GovernorMode::High => write!(f, "HIGH    "),
            GovernorMode::Droop => write!(f, "DROOP   "),
            GovernorMode::All => write!(f, "ALL SPD "),
        }
    }
}

/// Comprehensive Heavy Machinery ECM
#[derive(Clone)]
pub struct HeavyEcm {
    // ─ Engine Speed ─────────────────────────────────────────
    pub rpm: f64,
    pub rated_rpm: f64,
    pub idle_rpm: f64,
    pub governor_mode: GovernorMode,

    // ─ Torque & Load ────────────────────────────────────────
    pub torque_nm: f64,
    pub percent_load: f64, // 0-100%
    pub demand_torque_pct: f64,
    pub actual_torque_pct: f64,
    pub max_torque_nm: f64,

    // ─ Throttle ─────────────────────────────────────────────
    pub throttle_pct: f64, // 0-100%
    pub remote_throttle: bool,
    pub engine_brake: bool,

    // ─ Fuel System ──────────────────────────────────────────
    pub fuel_level_pct: f64,
    pub fuel_pressure_kpa: f64,
    pub fuel_rate_lph: f64,
    pub total_fuel_l: f64,
    pub trip_fuel_l: f64,

    // ─ Temperatures ─────────────────────────────────────────
    pub coolant_temp_c: f64,
    pub oil_temp_c: f64,
    pub fuel_temp_c: f64,
    pub intake_temp_c: f64,
    pub exhaust_temp_c: f64,
    pub turbo_oil_temp_c: f64,

    // ─ Pressures ────────────────────────────────────────────
    pub oil_pressure_kpa: f64,
    pub oil_level_pct: f64,
    pub coolant_pres_kpa: f64,
    pub boost_pressure_kpa: f64,
    pub intake_manifold_kpa: f64,
    pub air_filter_dp_kpa: f64,

    // ─ Electrical ───────────────────────────────────────────
    pub battery_v: f64,
    pub alternator_v: f64,
    pub starter_active: bool,

    // ─ Aftertreatment (Tier 4 / Stage V) ────────────────────
    pub def_level_pct: f64, // Diesel Exhaust Fluid
    pub def_quality_pct: f64,
    pub dpf_soot_pct: f64, // DPF soot load
    pub dpf_temp_c: f64,
    pub scr_efficiency_pct: f64, // SCR NOx reduction
    pub nox_ppm: f64,
    pub aftertreatment: AftertreatmentState,

    // ─ Service & Hours ──────────────────────────────────────
    pub engine_hours: f64,
    pub service_hours_left: f64,
    pub num_cylinders: u8,

    // ─ Diagnostics ──────────────────────────────────────────
    pub active_dtcs: Vec<Dtc>,
    pub stored_dtcs: Vec<Dtc>,
    pub mil_active: bool, // Malfunction Indicator Lamp
    pub amber_lamp: bool, // Amber Warning
    pub red_lamp: bool,   // Red Stop

    // Internal state
    rpm_integral: f64,
    warmup_complete: bool,
}

impl Default for HeavyEcm {
    fn default() -> Self {
        Self::new()
    }
}

impl HeavyEcm {
    pub fn new() -> Self {
        Self {
            rpm: IDLE_RPM,
            rated_rpm: RATED_RPM,
            idle_rpm: IDLE_RPM,
            governor_mode: GovernorMode::All,
            torque_nm: 0.0,
            percent_load: 0.0,
            demand_torque_pct: 0.0,
            actual_torque_pct: 0.0,
            max_torque_nm: PEAK_TORQUE_NM,
            throttle_pct: 0.0,
            remote_throttle: false,
            engine_brake: false,
            fuel_level_pct: 85.0,
            fuel_pressure_kpa: 400.0,
            fuel_rate_lph: 2.0,
            total_fuel_l: 0.0,
            trip_fuel_l: 0.0,
            coolant_temp_c: 20.0,
            oil_temp_c: 20.0,
            fuel_temp_c: 25.0,
            intake_temp_c: 25.0,
            exhaust_temp_c: 200.0,
            turbo_oil_temp_c: 20.0,
            oil_pressure_kpa: 350.0,
            oil_level_pct: 95.0,
            coolant_pres_kpa: 50.0,
            boost_pressure_kpa: 100.0,
            intake_manifold_kpa: 100.0,
            air_filter_dp_kpa: 1.5,
            battery_v: 12.8,
            alternator_v: 14.2,
            starter_active: false,
            def_level_pct: 72.0,
            def_quality_pct: 98.5,
            dpf_soot_pct: 15.0,
            dpf_temp_c: 250.0,
            scr_efficiency_pct: 96.0,
            nox_ppm: 45.0,
            aftertreatment: AftertreatmentState::Normal,
            engine_hours: 2347.5,
            service_hours_left: 152.5,
            num_cylinders: NUM_CYLINDERS,
            active_dtcs: Vec::new(),
            stored_dtcs: Vec::new(),
            mil_active: false,
            amber_lamp: false,
            red_lamp: false,
            rpm_integral: 0.0,
            warmup_complete: false,
        }
    }

    /// Diesel torque curve: peak ~1000 Nm at 1200-1600 RPM
    pub fn torque_curve(&self, rpm: f64, throttle_pct: f64) -> f64 {
        let t = throttle_pct / 100.0;
        let peak_rpm = 1400.0;
        let _norm = (rpm - peak_rpm) / 800.0;
        let curve = if rpm < 800.0 {
            0.0
        } else if rpm < peak_rpm {
            0.7 + 0.3 * (rpm - 800.0) / (peak_rpm - 800.0)
        } else if rpm < RATED_RPM {
            1.0 - 0.25 * (rpm - peak_rpm) / (RATED_RPM - peak_rpm)
        } else {
            (0.75 - (rpm - RATED_RPM) / (MAX_RPM - RATED_RPM) * 0.75).max(0.0)
        };
        PEAK_TORQUE_NM * curve * t
    }

    pub fn update(&mut self, throttle_pct: f64, load_nm: f64, dt: f64) {
        self.throttle_pct = throttle_pct.clamp(0.0, 100.0);
        let t = self.throttle_pct / 100.0;

        // RPM dynamics: governor tries to maintain rated RPM under load
        let target_rpm = self.idle_rpm + t * (RATED_RPM - self.idle_rpm);
        let available_torque = self.torque_curve(self.rpm, self.throttle_pct);
        let net_torque = available_torque - load_nm;
        // d(rpm)/dt based on engine inertia
        let rpm_accel = net_torque * 30.0 / (std::f64::consts::PI * 2.5); // J=2.5 kg.m²
        self.rpm_integral += rpm_accel * dt;
        self.rpm += (target_rpm - self.rpm) * dt * 1.5 + rpm_accel * dt * 0.1;
        self.rpm = self.rpm.clamp(self.idle_rpm * 0.8, MAX_RPM);

        // Torque & load
        self.torque_nm = available_torque.min(load_nm + 50.0);
        self.actual_torque_pct = (self.torque_nm / PEAK_TORQUE_NM * 100.0).clamp(-100.0, 100.0);
        self.demand_torque_pct = (t * 100.0 - 125.0 + 125.0).clamp(-100.0, 100.0);
        self.percent_load = (load_nm / PEAK_TORQUE_NM * 100.0).clamp(0.0, 150.0);

        // Fuel consumption (diesel: ~0.22 kg/kWh specific consumption)
        let power_kw = self.torque_nm * self.rpm * std::f64::consts::PI / 30000.0;
        self.fuel_rate_lph = (power_kw * 0.22 / 0.85 + 1.5).max(1.5); // 0.85 kg/L diesel
        let consumed = self.fuel_rate_lph * dt / 3600.0;
        self.total_fuel_l += consumed;
        self.trip_fuel_l += consumed;
        self.fuel_level_pct = (self.fuel_level_pct - consumed / FUEL_TANK_L * 100.0).max(0.0);

        // Thermal model
        if !self.warmup_complete {
            self.warmup_complete = self.coolant_temp_c > 80.0;
        }
        let tgt_coolant = if self.warmup_complete {
            90.0 + t * 5.0
        } else {
            self.coolant_temp_c + 3.0 * dt
        };
        self.coolant_temp_c += (tgt_coolant - self.coolant_temp_c) * dt * 0.02;
        self.oil_temp_c = self.coolant_temp_c + 12.0;
        self.exhaust_temp_c = 200.0 + t * 450.0 * (self.rpm / RATED_RPM).min(1.0);
        self.intake_temp_c = 25.0 + self.boost_pressure_kpa / 100.0 * 15.0;

        // Pressures
        let rpm_factor = (self.rpm / RATED_RPM).clamp(0.2, 1.2);
        self.oil_pressure_kpa = 250.0 + rpm_factor * 200.0;
        self.boost_pressure_kpa = 100.0 + t * rpm_factor * 200.0;
        self.intake_manifold_kpa = 100.0 + self.boost_pressure_kpa * 0.8;
        self.fuel_pressure_kpa = 350.0 + t * 100.0;

        // Aftertreatment
        self.dpf_soot_pct = (self.dpf_soot_pct + t * 0.0005 * dt - 0.0001 * dt).clamp(0.0, 100.0);
        self.dpf_temp_c = 250.0 + t * 300.0;
        self.def_level_pct =
            (self.def_level_pct - self.fuel_rate_lph * 0.05 * dt / 3600.0 * 100.0).max(0.0);
        self.scr_efficiency_pct = if self.def_level_pct > 5.0 {
            94.0 + t * 3.0
        } else {
            20.0
        };
        self.nox_ppm = (1500.0 * t / (self.scr_efficiency_pct / 100.0 + 0.01)).min(2000.0);

        if self.dpf_soot_pct > 60.0 && self.aftertreatment == AftertreatmentState::Normal {
            self.aftertreatment = AftertreatmentState::HighSoot;
        }
        if self.dpf_soot_pct > 80.0 {
            self.aftertreatment = AftertreatmentState::Regenerating;
        }
        if self.dpf_soot_pct < 10.0 && self.aftertreatment == AftertreatmentState::Regenerating {
            self.aftertreatment = AftertreatmentState::Normal;
        }

        // Electrical
        self.alternator_v = if self.rpm > 1000.0 { 14.2 } else { 12.4 };
        self.battery_v = if self.rpm > 1000.0 { 13.8 } else { 12.6 };

        // Service hours
        self.engine_hours += dt / 3600.0;
        self.service_hours_left -= dt / 3600.0;

        // Fault detection
        self.check_faults();
    }

    fn check_faults(&mut self) {
        self.active_dtcs.retain(|d| d.active);
        self.mil_active = false;
        self.amber_lamp = false;
        self.red_lamp = false;

        // High coolant temp → Red Stop
        if self.coolant_temp_c > 108.0 {
            self.inject_dtc(
                110,
                0,
                DtcSeverity::Red,
                "Engine Coolant Temp High — Red Stop",
            );
            self.red_lamp = true;
        }
        // Low oil pressure → Red Stop
        if self.oil_pressure_kpa < 80.0 {
            self.inject_dtc(
                100,
                1,
                DtcSeverity::Red,
                "Engine Oil Pressure Low — Red Stop",
            );
            self.red_lamp = true;
        }
        // Low DEF → Amber
        if self.def_level_pct < 10.0 {
            self.inject_dtc(
                3361,
                17,
                DtcSeverity::Amber,
                "DEF Level Low — SCR Derate Imminent",
            );
            self.amber_lamp = true;
        }
        // High soot → Amber
        if self.dpf_soot_pct > 70.0 {
            self.inject_dtc(
                3251,
                15,
                DtcSeverity::Amber,
                "DPF Soot Load High — Regen Required",
            );
            self.amber_lamp = true;
        }
        // Service overdue
        if self.service_hours_left < 0.0 {
            self.inject_dtc(247, 31, DtcSeverity::Mil, "Service Interval Overdue");
            self.mil_active = true;
        }
        for dtc in &self.active_dtcs {
            match dtc.severity {
                DtcSeverity::Red => self.red_lamp = true,
                DtcSeverity::Amber => self.amber_lamp = true,
                DtcSeverity::Mil => self.mil_active = true,
                _ => {}
            }
        }
    }

    fn inject_dtc(&mut self, spn: u32, fmi: u8, sev: DtcSeverity, desc: &'static str) {
        if !self
            .active_dtcs
            .iter()
            .any(|d| d.spn == spn && d.fmi == fmi)
        {
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

    pub fn clear_dtcs(&mut self) {
        for dtc in &mut self.active_dtcs {
            dtc.active = false;
        }
        self.active_dtcs.retain(|d| d.active);
    }
}
