//! NVM — Non-Volatile Memory simulation (EEPROM / Flash emulation)
//!
//! In a real ECU, NVM stores:
//!   • Odometer and engine hours (survive power cuts)
//!   • Calibration parameters (survives firmware updates)
//!   • Diagnostic trouble codes (DTCs persist 40+ drive cycles)
//!   • Learned/adapted values (fuel trim, injector offsets, gear ratios)
//!   • Event counters (number of DPF regens, ABS events, etc.)
//!   • VIN and ECU identification
//!   • Boot counter and last-reset reason
//!
//! KAM (Keep Alive Memory): A subset of NVM powered by battery even with IGN off.
//!   Loses content only if battery is disconnected.
//!
//! Wear levelling: Real NVM has limited write cycles (~100K for EEPROM).
//!   This simulation tracks write counts per region.

// ── NVM Region tags ───────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NvmRegion {
    /// Odometer, engine hours — updated every minute
    Counters,
    /// Calibration parameters — updated only by diagnostic tool
    Calibration,
    /// Active DTCs — updated on fault set/clear
    FaultStorage,
    /// Adaptive values (fuel trim, injector, etc.) — updated periodically
    Adaptations,
    /// VIN, ECU ID, programming date — written at factory / reflash only
    Identification,
    /// Event log (ring buffer of recent faults/events)
    EventLog,
}

impl std::fmt::Display for NvmRegion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            NvmRegion::Counters => "COUNTERS   ",
            NvmRegion::Calibration => "CALIBRATION",
            NvmRegion::FaultStorage => "FAULT_STOR ",
            NvmRegion::Adaptations => "ADAPTATIONS",
            NvmRegion::Identification => "IDENT      ",
            NvmRegion::EventLog => "EVENT_LOG  ",
        };
        write!(f, "{}", s)
    }
}

// ── NVM Block ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct NvmBlock {
    pub region: NvmRegion,
    pub name: &'static str,
    /// Stored value (64-bit, multi-purpose)
    pub value_u64: u64,
    pub value_f64: f64,
    pub value_str: String,
    /// Write counter — for wear tracking
    pub write_count: u32,
    /// Max writes before wear-out (EEPROM: ~100K, Flash: ~10K)
    pub max_writes: u32,
    pub dirty: bool, // modified since last "flush"
}

impl NvmBlock {
    pub fn new_f64(region: NvmRegion, name: &'static str, initial: f64, max_wr: u32) -> Self {
        NvmBlock {
            region,
            name,
            value_u64: 0,
            value_f64: initial,
            value_str: String::new(),
            write_count: 0,
            dirty: false,
            max_writes: max_wr,
        }
    }
    pub fn new_u64(region: NvmRegion, name: &'static str, initial: u64, max_wr: u32) -> Self {
        NvmBlock {
            region,
            name,
            value_u64: initial,
            value_f64: 0.0,
            value_str: String::new(),
            write_count: 0,
            dirty: false,
            max_writes: max_wr,
        }
    }
    pub fn new_str(region: NvmRegion, name: &'static str, initial: &str, max_wr: u32) -> Self {
        NvmBlock {
            region,
            name,
            value_u64: 0,
            value_f64: 0.0,
            value_str: initial.into(),
            write_count: 0,
            dirty: false,
            max_writes: max_wr,
        }
    }
    pub fn write_f64(&mut self, v: f64) {
        self.value_f64 = v;
        self.write_count += 1;
        self.dirty = true;
    }
    pub fn write_u64(&mut self, v: u64) {
        self.value_u64 = v;
        self.write_count += 1;
        self.dirty = true;
    }
    pub fn write_str(&mut self, v: String) {
        self.value_str = v;
        self.write_count += 1;
        self.dirty = true;
    }
    pub fn is_worn(&self) -> bool {
        self.write_count >= self.max_writes
    }
    pub fn wear_pct(&self) -> f64 {
        self.write_count as f64 / self.max_writes as f64 * 100.0
    }
}

// ── Adaptation Values ─────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Adaptations {
    /// Long-term fuel trim (%) — learned correction to injector duration
    pub fuel_trim_lt_pct: f64,
    /// Short-term fuel trim
    pub fuel_trim_st_pct: f64,
    /// Cylinder balance offset per cylinder (injection time correction in µs)
    pub injector_offset_us: [f64; 6],
    /// Clutch fill time adaptation (ms) — learned from TCM
    pub clutch_fill_time_ms: f64,
    /// Gear ratio learned (for slip detection)
    pub gear_ratio_learned: [f64; 9],
    /// Idle speed learned correction
    pub idle_speed_correction: f64,
    /// DPF ash load (irreversible — only reduces with filter replacement)
    pub dpf_ash_load_pct: f64,
    /// Injector drift count per cylinder
    pub injector_drift: [u32; 6],
}

impl Adaptations {
    pub fn new() -> Self {
        Adaptations {
            fuel_trim_lt_pct: 0.0,
            fuel_trim_st_pct: 0.0,
            injector_offset_us: [0.0; 6],
            clutch_fill_time_ms: 120.0,
            gear_ratio_learned: [0.0, 4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667],
            idle_speed_correction: 0.0,
            dpf_ash_load_pct: 2.0,
            injector_drift: [0; 6],
        }
    }

    /// Slowly adapt fuel trim toward closed-loop target
    pub fn update_fuel_trim(&mut self, lambda_error_pct: f64, dt: f64) {
        self.fuel_trim_st_pct += lambda_error_pct * dt * 0.5;
        self.fuel_trim_st_pct = self.fuel_trim_st_pct.clamp(-25.0, 25.0);
        // Move ST trim to LT trim slowly
        self.fuel_trim_lt_pct += self.fuel_trim_st_pct * dt * 0.01;
        self.fuel_trim_lt_pct = self.fuel_trim_lt_pct.clamp(-25.0, 25.0);
    }
}

// ── DTC Persistence (NVM storage of fault history) ────────────────────────────

#[derive(Debug, Clone)]
pub struct StoredDtcRecord {
    pub spn: u32,
    pub fmi: u8,
    pub occurrence_count: u8,
    /// Drive cycles (key-on counts) since last active
    pub drive_cycles_ago: u16,
    /// Automatically clear after 40 drive cycles (legislated OBD rule)
    pub auto_clear_cycle: u16,
    pub confirmed: bool,
    pub pending: bool,
    pub test_passed: bool,
}

impl StoredDtcRecord {
    pub fn new(spn: u32, fmi: u8) -> Self {
        StoredDtcRecord {
            spn,
            fmi,
            occurrence_count: 1,
            drive_cycles_ago: 0,
            auto_clear_cycle: 40,
            confirmed: false,
            pending: true,
            test_passed: false,
        }
    }
    /// Called each key-on — increments cycle counter, returns true if should auto-clear
    pub fn increment_cycle(&mut self) -> bool {
        self.drive_cycles_ago += 1;
        self.drive_cycles_ago >= self.auto_clear_cycle
    }
}

// ── NVM Store ─────────────────────────────────────────────────────────────────

pub struct NvmStore {
    pub blocks: Vec<NvmBlock>,
    pub adaptations: Adaptations,
    pub stored_dtcs: Vec<StoredDtcRecord>,

    /// How many key-on cycles have occurred
    pub drive_cycle_count: u32,
    pub battery_disconnect_count: u32,
    /// Timestamp of last successful NVM flush (seconds uptime)
    last_flush_ts: f64,
    pub flush_interval_s: f64,

    /// Total writes across all blocks
    pub total_writes: u64,
}

impl NvmStore {
    pub fn new() -> Self {
        let blocks = vec![
            // Counters region (high write frequency)
            NvmBlock::new_f64(NvmRegion::Counters, "engine_hours", 2347.5, 500_000),
            NvmBlock::new_f64(NvmRegion::Counters, "odometer_km", 12450.0, 500_000),
            NvmBlock::new_f64(NvmRegion::Counters, "total_fuel_l", 28650.0, 500_000),
            NvmBlock::new_u64(NvmRegion::Counters, "key_on_count", 1247, 200_000),
            NvmBlock::new_u64(NvmRegion::Counters, "dpf_regen_count", 47, 100_000),
            NvmBlock::new_u64(NvmRegion::Counters, "abs_event_count", 8, 100_000),
            NvmBlock::new_u64(NvmRegion::Counters, "tcs_event_count", 12, 100_000),
            NvmBlock::new_u64(NvmRegion::Counters, "overheat_count", 0, 100_000),
            // Calibration region (low write frequency)
            NvmBlock::new_f64(NvmRegion::Calibration, "idle_rpm_cal", 800.0, 10_000),
            NvmBlock::new_f64(NvmRegion::Calibration, "rated_rpm_cal", 2200.0, 10_000),
            NvmBlock::new_f64(NvmRegion::Calibration, "max_torque_cal", 1050.0, 10_000),
            NvmBlock::new_f64(NvmRegion::Calibration, "dpf_regen_thr", 75.0, 10_000),
            NvmBlock::new_f64(NvmRegion::Calibration, "def_warn_pct", 10.0, 10_000),
            NvmBlock::new_u64(NvmRegion::Calibration, "fuel_map_sel", 0, 10_000),
            NvmBlock::new_f64(NvmRegion::Calibration, "svc_interval_h", 500.0, 10_000),
            // Identification (written only at factory / programming)
            NvmBlock::new_str(NvmRegion::Identification, "vin", "1HD1KEM16FB123456", 100),
            NvmBlock::new_str(NvmRegion::Identification, "sw_version", "SW_01.23.004", 500),
            NvmBlock::new_str(NvmRegion::Identification, "hw_version", "HW_02.00", 10),
            NvmBlock::new_str(NvmRegion::Identification, "cal_id", "CAL_TIER4_V3.1", 100),
            NvmBlock::new_str(
                NvmRegion::Identification,
                "ecu_serial",
                "ECU20240811001",
                10,
            ),
            NvmBlock::new_u64(NvmRegion::Identification, "flash_count", 3, 50),
        ];

        NvmStore {
            blocks,
            adaptations: Adaptations::new(),
            stored_dtcs: Vec::new(),
            drive_cycle_count: 1247,
            battery_disconnect_count: 0,
            last_flush_ts: 0.0,
            flush_interval_s: 60.0,
            total_writes: 0,
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Call every tick — writes dirty blocks to "storage" at flush_interval
    pub fn tick(&mut self, elapsed: f64, dt: f64) {
        // Periodic flush (simulates real EEPROM write cycles)
        if elapsed - self.last_flush_ts >= self.flush_interval_s {
            self.flush(elapsed);
        }

        // Update adaptations with small fuel trim drift
        self.adaptations.update_fuel_trim(0.1 * noise(elapsed), dt);
    }

    fn flush(&mut self, ts: f64) {
        for b in &mut self.blocks {
            if b.dirty {
                b.dirty = false;
                self.total_writes += 1;
            }
        }
        self.last_flush_ts = ts;
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Called on key-on — increments drive cycle counter, ages stored DTCs
    pub fn on_key_on(&mut self) {
        self.drive_cycle_count += 1;
        self.write_u64("key_on_count", self.drive_cycle_count as u64);

        // Age stored DTCs — auto-clear after 40 drive cycles
        // retain_mut not stable — use manual drain approach
        let mut i = 0;
        while i < self.stored_dtcs.len() {
            if self.stored_dtcs[i].increment_cycle() {
                self.stored_dtcs.remove(i);
            } else {
                i += 1;
            }
        }
    }

    /// Called on key-off — flush everything
    pub fn on_key_off(&mut self, elapsed: f64) {
        self.flush(elapsed);
    }

    // ─────────────────────────────────────────────────────────────────────────
    pub fn write_f64(&mut self, name: &str, value: f64) {
        if let Some(b) = self.blocks.iter_mut().find(|b| b.name == name) {
            b.write_f64(value);
        }
    }
    pub fn write_u64(&mut self, name: &str, value: u64) {
        if let Some(b) = self.blocks.iter_mut().find(|b| b.name == name) {
            b.write_u64(value);
        }
    }
    pub fn read_f64(&self, name: &str) -> Option<f64> {
        self.blocks
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.value_f64)
    }
    pub fn read_u64(&self, name: &str) -> Option<u64> {
        self.blocks
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.value_u64)
    }
    pub fn read_str(&self, name: &str) -> Option<&str> {
        self.blocks
            .iter()
            .find(|b| b.name == name)
            .map(|b| b.value_str.as_str())
    }

    pub fn worn_blocks(&self) -> impl Iterator<Item = &NvmBlock> {
        self.blocks.iter().filter(|b| b.is_worn())
    }

    pub fn add_stored_dtc(&mut self, spn: u32, fmi: u8) {
        if !self
            .stored_dtcs
            .iter()
            .any(|d| d.spn == spn && d.fmi == fmi)
        {
            self.stored_dtcs.push(StoredDtcRecord::new(spn, fmi));
        } else if let Some(d) = self
            .stored_dtcs
            .iter_mut()
            .find(|d| d.spn == spn && d.fmi == fmi)
        {
            d.occurrence_count = d.occurrence_count.saturating_add(1);
            d.drive_cycles_ago = 0;
        }
    }
}

fn noise(seed: f64) -> f64 {
    let x = (seed * 127.1 + 311.7).sin() * 43758.545;
    x - x.floor() - 0.5
}
