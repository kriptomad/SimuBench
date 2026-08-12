//! Instrument Cluster Module (ICM/DASH) — SA 0x1C.
//! Receives J1939 data from all ECUs, presents live gauge readings.
//! Tracks odometer, trip computer, warning lamp states.
//! J1939: listens only (display module); sends heartbeat on Prop-B (100ms).

use crate::j1939::{self, addr, J1939Frame};

pub const ICM_SA: u8 = addr::INSTRUMENT; // 0x1C

// ── Gauge ─────────────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct Gauge {
    pub name: &'static str,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub unit: &'static str,
    pub warn_lo: Option<f64>,
    pub warn_hi: Option<f64>,
    pub crit_hi: Option<f64>,
    pub in_warning: bool,
    pub in_critical: bool,
}

impl Gauge {
    pub fn new(name: &'static str, min: f64, max: f64, unit: &'static str) -> Self {
        Gauge {
            name,
            value: 0.0,
            min,
            max,
            unit,
            warn_lo: None,
            warn_hi: None,
            crit_hi: None,
            in_warning: false,
            in_critical: false,
        }
    }
    pub fn with_warn_hi(mut self, w: f64, c: f64) -> Self {
        self.warn_hi = Some(w);
        self.crit_hi = Some(c);
        self
    }
    pub fn with_warn_lo(mut self, w: f64) -> Self {
        self.warn_lo = Some(w);
        self
    }

    pub fn update(&mut self, val: f64) {
        self.value = val.clamp(self.min, self.max);
        self.in_warning = self.warn_hi.is_some_and(|w| self.value > w)
            || self.warn_lo.is_some_and(|l| self.value < l);
        self.in_critical = self.crit_hi.is_some_and(|c| self.value > c);
    }

    /// Fraction 0-1 of gauge arc
    pub fn fraction(&self) -> f64 {
        (self.value - self.min) / (self.max - self.min)
    }
}

// ── Warning Lamp ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LampColor {
    Off,
    Green,
    Yellow,
    Red,
    Flashing,
}

#[derive(Debug, Clone)]
pub struct WarningLamp {
    pub id: &'static str,
    pub label: &'static str,
    pub color: LampColor,
}

impl WarningLamp {
    pub fn new(id: &'static str, label: &'static str) -> Self {
        WarningLamp {
            id,
            label,
            color: LampColor::Off,
        }
    }
    pub fn set(&mut self, c: LampColor) {
        self.color = c;
    }
}

// ── Trip Computer ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct TripComputer {
    pub trip_distance_km: f64,
    pub trip_fuel_l: f64,
    pub trip_hours: f64,
    pub avg_fuel_lph: f64,
    pub avg_speed_kmh: f64,
    samples: u64,
    speed_sum: f64,
}

impl Default for TripComputer {
    fn default() -> Self {
        Self::new()
    }
}

impl TripComputer {
    pub fn new() -> Self {
        TripComputer {
            trip_distance_km: 0.0,
            trip_fuel_l: 0.0,
            trip_hours: 0.0,
            avg_fuel_lph: 0.0,
            avg_speed_kmh: 0.0,
            samples: 0,
            speed_sum: 0.0,
        }
    }

    pub fn update(&mut self, speed_kmh: f64, fuel_lph: f64, dt: f64) {
        self.trip_distance_km += speed_kmh * dt / 3600.0;
        self.trip_fuel_l += fuel_lph * dt / 3600.0;
        self.trip_hours += dt / 3600.0;
        self.speed_sum += speed_kmh;
        self.samples += 1;
        self.avg_speed_kmh = self.speed_sum / self.samples.max(1) as f64;
        if self.trip_hours > 0.0 {
            self.avg_fuel_lph = self.trip_fuel_l / self.trip_hours;
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

// ── ICM ──────────────────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct EcuIcm {
    pub sa: u8,

    // ─ Gauges (updated from CAN messages) ───────────────────────────────────
    pub tachometer: Gauge,      // RPM
    pub speedometer: Gauge,     // km/h
    pub fuel_gauge: Gauge,      // %
    pub coolant_gauge: Gauge,   // °C
    pub oil_press_gauge: Gauge, // kPa
    pub boost_gauge: Gauge,     // kPa
    pub def_gauge: Gauge,       // %
    pub hours_gauge: Gauge,     // h
    pub battery_gauge: Gauge,   // V

    // ─ Warning lamps ─────────────────────────────────────────────────────────
    pub lamps: Vec<WarningLamp>,

    // ─ Trip computer ─────────────────────────────────────────────────────────
    pub trip: TripComputer,
    pub odometer_km: f64,

    // ─ Last known ECU data (aggregated from CAN) ─────────────────────────────
    pub engine_rpm: f64,
    pub vehicle_speed: f64,
    pub engine_load_pct: f64,
    pub coolant_temp: f64,
    pub oil_pressure_kpa: f64,
    pub fuel_level_pct: f64,
    pub def_level_pct: f64,
    pub engine_hours: f64,
    pub battery_volts: f64,
    pub boost_kpa: f64,
    pub trans_gear: String,
    pub active_dtc_count: u32,

    // ─ Lamp states (set by DM1 messages) ────────────────────────────────────
    pub mil_active: bool,
    pub amber_active: bool,
    pub red_active: bool,
    pub protect_active: bool,

    // ─ Self-test sequence ────────────────────────────────────────────────────
    pub self_test_active: bool,
    self_test_timer: f64,

    // ─ TX timer ─────────────────────────────────────────────────────────────
    t_heartbeat: f64,
}

impl Default for EcuIcm {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuIcm {
    pub fn new() -> Self {
        let lamps = vec![
            WarningLamp::new("RED_STOP", "RED STOP"),
            WarningLamp::new("AMBER_WARN", "AMBER WARN"),
            WarningLamp::new("MIL", "CHECK ENGINE"),
            WarningLamp::new("PROTECT", "PROTECT ENG"),
            WarningLamp::new("OIL_PRESS", "OIL PRESS"),
            WarningLamp::new("COOLANT_T", "COOLANT TEMP"),
            WarningLamp::new("DEF_LOW", "DEF LOW"),
            WarningLamp::new("DPF_REGEN", "DPF REGEN"),
            WarningLamp::new("FUEL_LOW", "FUEL LOW"),
            WarningLamp::new("TRANS_FAULT", "TRANS FAULT"),
            WarningLamp::new("CHARGE", "CHARGING"),
            WarningLamp::new("AIR_FILTER", "AIR FILTER"),
            WarningLamp::new("HYD_TEMP", "HYD TEMP"),
            WarningLamp::new("PTO_ACTV", "PTO ACTIVE"),
        ];

        EcuIcm {
            sa: ICM_SA,
            tachometer: Gauge::new("Engine RPM", 0.0, 3000.0, "rpm").with_warn_hi(2400.0, 2700.0),
            speedometer: Gauge::new("Speed", 0.0, 55.0, "km/h"),
            fuel_gauge: Gauge::new("Fuel", 0.0, 100.0, "%").with_warn_lo(10.0),
            coolant_gauge: Gauge::new("Coolant Temp", 0.0, 130.0, "°C").with_warn_hi(100.0, 108.0),
            oil_press_gauge: Gauge::new("Oil Pressure", 0.0, 700.0, "kPa").with_warn_lo(100.0),
            boost_gauge: Gauge::new("Boost", 0.0, 300.0, "kPa"),
            def_gauge: Gauge::new("DEF", 0.0, 100.0, "%").with_warn_lo(10.0),
            hours_gauge: Gauge::new("Hours", 0.0, 99999.0, "h"),
            battery_gauge: Gauge::new("Battery", 9.0, 16.0, "V").with_warn_lo(11.0),
            lamps,
            trip: TripComputer::new(),
            odometer_km: 12450.0,
            engine_rpm: 0.0,
            vehicle_speed: 0.0,
            engine_load_pct: 0.0,
            coolant_temp: 20.0,
            oil_pressure_kpa: 0.0,
            fuel_level_pct: 85.0,
            def_level_pct: 72.0,
            engine_hours: 0.0,
            battery_volts: 12.8,
            boost_kpa: 100.0,
            trans_gear: "N".into(),
            active_dtc_count: 0,
            mil_active: false,
            amber_active: false,
            red_active: false,
            protect_active: false,
            self_test_active: false,
            self_test_timer: 0.0,
            t_heartbeat: 0.0,
        }
    }

    /// Call after boot: runs a 3-second instrument self-test (all gauges sweep)
    pub fn start_self_test(&mut self) {
        self.self_test_active = true;
        self.self_test_timer = 0.0;
        for lamp in &mut self.lamps {
            lamp.color = LampColor::Yellow;
        }
    }

    /// Main ICM tick — processes received CAN frames, updates gauges.
    pub fn tick(&mut self, received: &[J1939Frame], dt: f64) -> Vec<J1939Frame> {
        // ─ Self-test ─────────────────────────────────────────────────────────
        if self.self_test_active {
            self.self_test_timer += dt;
            let phase = (self.self_test_timer / 3.0).min(1.0);
            // Sweep all gauges to max then back
            let sweep = if phase < 0.5 {
                phase * 2.0
            } else {
                2.0 - phase * 2.0
            };
            self.tachometer.update(self.tachometer.max * sweep);
            self.speedometer.update(self.speedometer.max * sweep);
            if self.self_test_timer >= 3.0 {
                self.self_test_active = false;
                for lamp in &mut self.lamps {
                    lamp.color = LampColor::Off;
                }
            }
            return Vec::new();
        }

        // ─ Process received J1939 frames ──────────────────────────────────────
        for frame in received {
            use crate::j1939::pgn;
            match frame.pgn {
                p if p == pgn::EEC1 => {
                    for v in &frame.decoded {
                        if v.spn == 190 {
                            self.engine_rpm = v.physical;
                        }
                        if v.spn == 513 {
                            self.engine_load_pct = (v.physical + 125.0).max(0.0);
                        }
                    }
                }
                p if p == pgn::EEC2 => {
                    for v in &frame.decoded {
                        if v.spn == 92 {
                            self.engine_load_pct = v.physical;
                        }
                    }
                }
                p if p == pgn::ET1 => {
                    for v in &frame.decoded {
                        if v.spn == 110 {
                            self.coolant_temp = v.physical;
                        }
                    }
                }
                p if p == pgn::EFL_P1 => {
                    for v in &frame.decoded {
                        if v.spn == 100 {
                            self.oil_pressure_kpa = v.physical;
                        }
                    }
                }
                p if p == pgn::IC1 => {
                    for v in &frame.decoded {
                        if v.spn == 102 {
                            self.boost_kpa = v.physical;
                        }
                    }
                }
                p if p == pgn::LFE => {
                    for v in &frame.decoded {
                        if v.spn == 183 {
                            /* fuel rate used in trip */
                            let _ = v.physical;
                        }
                    }
                }
                p if p == pgn::HOURS => {
                    for v in &frame.decoded {
                        if v.spn == 247 {
                            self.engine_hours = v.physical;
                        }
                    }
                }
                p if p == pgn::CCVS => {
                    for v in &frame.decoded {
                        if v.spn == 84 {
                            self.vehicle_speed = v.physical;
                        }
                    }
                }
                p if p == pgn::DM1 => {
                    // Read lamp bits from byte 0
                    let b0 = frame.data[0];
                    self.mil_active = (b0 & 0x40) != 0;
                    self.red_active = (b0 & 0x10) != 0;
                    self.amber_active = (b0 & 0x04) != 0;
                    self.protect_active = (b0 & 0x01) != 0;
                    self.active_dtc_count = if frame.data[2] != 0xFF { 1 } else { 0 };
                }
                _ => {}
            }
        }

        // ─ Update all gauges ──────────────────────────────────────────────────
        self.tachometer.update(self.engine_rpm);
        self.speedometer.update(self.vehicle_speed);
        self.fuel_gauge.update(self.fuel_level_pct);
        self.coolant_gauge.update(self.coolant_temp);
        self.oil_press_gauge.update(self.oil_pressure_kpa);
        self.boost_gauge.update(self.boost_kpa);
        self.def_gauge.update(self.def_level_pct);
        self.hours_gauge.update(self.engine_hours);
        self.battery_gauge.update(self.battery_volts);

        // ─ Warning lamps ──────────────────────────────────────────────────────
        self.update_lamps();

        // ─ Trip / odometer ───────────────────────────────────────────────────
        let fuel_lph = 0.0; // would come from LFE frame
        self.trip.update(self.vehicle_speed, fuel_lph, dt);
        self.odometer_km += self.vehicle_speed * dt / 3600.0;

        // ─ Heartbeat ─────────────────────────────────────────────────────────
        self.t_heartbeat += dt;
        let mut frames: Vec<J1939Frame> = Vec::new();
        if self.t_heartbeat >= 0.100 {
            self.t_heartbeat = 0.0;
            let mut data = [0u8; 8];
            data[0] = (if self.red_active { 0x01 } else { 0 })
                | (if self.amber_active { 0x02 } else { 0 })
                | (if self.mil_active { 0x04 } else { 0 });
            data[1] = (self.vehicle_speed * 10.0) as u8;
            let raw_id = J1939Frame::build_id(7, j1939::pgn::PROP_A + 1, self.sa, 0xFF);
            frames.push(J1939Frame::from_raw(self.t_heartbeat, raw_id, &data));
        }
        frames
    }

    fn update_lamps(&mut self) {
        for lamp in &mut self.lamps {
            lamp.color = match lamp.id {
                "RED_STOP" => {
                    if self.red_active {
                        LampColor::Red
                    } else {
                        LampColor::Off
                    }
                }
                "AMBER_WARN" => {
                    if self.amber_active {
                        LampColor::Yellow
                    } else {
                        LampColor::Off
                    }
                }
                "MIL" => {
                    if self.mil_active {
                        LampColor::Yellow
                    } else {
                        LampColor::Off
                    }
                }
                "PROTECT" => {
                    if self.protect_active {
                        LampColor::Yellow
                    } else {
                        LampColor::Off
                    }
                }
                "OIL_PRESS" => {
                    if self.oil_pressure_kpa < 100.0 && self.engine_rpm > 500.0 {
                        LampColor::Red
                    } else {
                        LampColor::Off
                    }
                }
                "COOLANT_T" => {
                    if self.coolant_temp > 105.0 {
                        LampColor::Red
                    } else {
                        LampColor::Off
                    }
                }
                "DEF_LOW" => {
                    if self.def_level_pct < 10.0 {
                        LampColor::Yellow
                    } else {
                        LampColor::Off
                    }
                }
                "FUEL_LOW" => {
                    if self.fuel_level_pct < 10.0 {
                        LampColor::Yellow
                    } else {
                        LampColor::Off
                    }
                }
                "CHARGE" => {
                    if self.battery_volts < 12.0 {
                        LampColor::Yellow
                    } else {
                        LampColor::Off
                    }
                }
                _ => LampColor::Off,
            };
        }
    }
}
