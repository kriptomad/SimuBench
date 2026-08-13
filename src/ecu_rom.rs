//! ECU ROM and calibration map editor.
//!
//! Simulates a Tier 4 Final diesel ECU calibration ROM (~64 KB) with:
//!   • 3D lookup tables — fuel, ignition, boost, lambda, VE, EGR
//!   • 2D curves       — idle speed, torque limit, injector timing, etc.
//!   • Raw ROM hex view with named regions and address bookmarks
//!   • Patch system    — DPF delete, EGR delete, speed limiter, etc.
//!   • Live-cursor tracking (ECM live data highlights active cell)

pub const ROM_SIZE: usize = 65536; // 64 KB virtual ROM
pub const MAP_RPM_BINS: usize = 16;
pub const MAP_LOAD_BINS: usize = 16;
pub const MAP_2D_BINS: usize = 16;

// ── RPM breakpoints — typical heavy diesel (800..2600 rpm) ───────────────────
pub const RPM_AXIS: [f64; MAP_RPM_BINS] = [
    500.0, 600.0, 700.0, 800.0, 900.0, 1000.0, 1100.0, 1200.0,
    1400.0, 1600.0, 1800.0, 2000.0, 2200.0, 2300.0, 2400.0, 2600.0,
];
// ── Load (throttle demand %) breakpoints ─────────────────────────────────────
pub const LOAD_AXIS: [f64; MAP_LOAD_BINS] = [
    0.0, 6.25, 12.5, 18.75, 25.0, 31.25, 37.5, 43.75,
    50.0, 56.25, 62.5, 68.75, 75.0, 81.25, 87.5, 100.0,
];

// ── Virtual ROM region map ────────────────────────────────────────────────────
pub const REGION_HEADER: u32 = 0x0000;
pub const REGION_ENGINE_BASE: u32 = 0x1000;
pub const REGION_FUEL_MAP: u32 = 0x2000;
pub const REGION_IGN_MAP: u32 = 0x3000;
pub const REGION_BOOST_MAP: u32 = 0x4000;
pub const REGION_LAMBDA_MAP: u32 = 0x5000;
pub const REGION_VE_MAP: u32 = 0x5800;
pub const REGION_EGR_MAP: u32 = 0x6000;
pub const REGION_2D_CURVES: u32 = 0x7000;
pub const REGION_AFTERTREATMENT: u32 = 0x8000;
pub const REGION_LIMITS: u32 = 0x9000;

// ── 3D calibration map ───────────────────────────────────────────────────────
#[derive(Clone)]
pub struct CalMap3D {
    pub name: &'static str,
    pub unit: &'static str,
    pub x_label: &'static str,
    pub y_label: &'static str,
    pub x_axis: [f64; MAP_RPM_BINS],
    pub y_axis: [f64; MAP_LOAD_BINS],
    pub data: [[f64; MAP_RPM_BINS]; MAP_LOAD_BINS],
    pub min: f64,
    pub max: f64,
    pub rom_base: u32,
    pub description: &'static str,
}

impl CalMap3D {
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row.min(MAP_LOAD_BINS - 1)][col.min(MAP_RPM_BINS - 1)]
    }

    pub fn set(&mut self, row: usize, col: usize, v: f64) {
        self.data[row.min(MAP_LOAD_BINS - 1)][col.min(MAP_RPM_BINS - 1)] =
            v.clamp(self.min, self.max);
    }

    /// Bilinear interpolation at (rpm, load).
    pub fn interpolate(&self, rpm: f64, load: f64) -> f64 {
        let xi = self.x_axis.partition_point(|&r| r <= rpm).saturating_sub(1).min(MAP_RPM_BINS - 2);
        let yi = self.y_axis.partition_point(|&l| l <= load).saturating_sub(1).min(MAP_LOAD_BINS - 2);
        let fx = if self.x_axis[xi + 1] > self.x_axis[xi] {
            (rpm - self.x_axis[xi]) / (self.x_axis[xi + 1] - self.x_axis[xi])
        } else { 0.0 }.clamp(0.0, 1.0);
        let fy = if self.y_axis[yi + 1] > self.y_axis[yi] {
            (load - self.y_axis[yi]) / (self.y_axis[yi + 1] - self.y_axis[yi])
        } else { 0.0 }.clamp(0.0, 1.0);
        let v00 = self.data[yi][xi];
        let v10 = self.data[yi][xi + 1];
        let v01 = self.data[yi + 1][xi];
        let v11 = self.data[yi + 1][xi + 1];
        v00 * (1.0 - fx) * (1.0 - fy)
            + v10 * fx * (1.0 - fy)
            + v01 * (1.0 - fx) * fy
            + v11 * fx * fy
    }

    /// Closest cell indices to (rpm, load).
    pub fn live_cell(&self, rpm: f64, load: f64) -> (usize, usize) {
        let xi = self.x_axis.iter().enumerate()
            .min_by(|(_, a), (_, b)| ((*a - rpm).abs()).partial_cmp(&((*b - rpm).abs())).unwrap())
            .map(|(i, _)| i).unwrap_or(0);
        let yi = self.y_axis.iter().enumerate()
            .min_by(|(_, a), (_, b)| ((*a - load).abs()).partial_cmp(&((*b - load).abs())).unwrap())
            .map(|(i, _)| i).unwrap_or(0);
        (yi, xi)
    }
}

// ── 2D calibration curve ─────────────────────────────────────────────────────
#[derive(Clone)]
pub struct CalMap2D {
    pub name: &'static str,
    pub unit: &'static str,
    pub x_label: &'static str,
    pub x_axis: [f64; MAP_2D_BINS],
    pub data: [f64; MAP_2D_BINS],
    pub min: f64,
    pub max: f64,
    pub rom_base: u32,
    pub description: &'static str,
}

impl CalMap2D {
    pub fn interpolate(&self, x: f64) -> f64 {
        let xi = self.x_axis.partition_point(|&v| v <= x).saturating_sub(1).min(MAP_2D_BINS - 2);
        let fx = if self.x_axis[xi + 1] > self.x_axis[xi] {
            (x - self.x_axis[xi]) / (self.x_axis[xi + 1] - self.x_axis[xi])
        } else { 0.0 }.clamp(0.0, 1.0);
        self.data[xi] * (1.0 - fx) + self.data[xi + 1] * fx
    }
}

// ── ROM address patch ─────────────────────────────────────────────────────────
#[derive(Clone)]
pub struct RomPatch {
    pub name: &'static str,
    pub description: &'static str,
    pub addr: u32,
    pub original: u8,
    pub patched: u8,
    pub enabled: bool,
    pub category: PatchCategory,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PatchCategory {
    Aftertreatment,
    Limits,
    Sensors,
    Diagnostics,
    Fuel,
    Other,
}

impl std::fmt::Display for PatchCategory {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Aftertreatment => write!(f, "Aftertreatment"),
            Self::Limits => write!(f, "Limits"),
            Self::Sensors => write!(f, "Sensors"),
            Self::Diagnostics => write!(f, "Diagnostics"),
            Self::Fuel => write!(f, "Fuel"),
            Self::Other => write!(f, "Other"),
        }
    }
}

// ── Named ROM region ──────────────────────────────────────────────────────────
pub struct RomRegion {
    pub name: &'static str,
    pub base: u32,
    pub size: u32,
    pub description: &'static str,
}

// ── Delete / inhibit flags (applied to ECM simulation) ───────────────────────
#[derive(Clone)]
pub struct DeleteFlags {
    pub dpf_regen: bool,
    pub dpf_soot_fault: bool,
    pub egr_valve: bool,
    pub egr_fault: bool,
    pub o2_lambda_plausibility: bool,
    pub nox_sensor: bool,
    pub adblue_def: bool,
    pub swirl_flap: bool,
    pub speed_limiter: bool,
    pub speed_limit_kmh: f64,
    pub torque_derate: bool,
    pub throttle_remap: bool,
    pub dtc_mask: Vec<u32>,
}

impl Default for DeleteFlags {
    fn default() -> Self {
        Self {
            dpf_regen: false,
            dpf_soot_fault: false,
            egr_valve: false,
            egr_fault: false,
            o2_lambda_plausibility: false,
            nox_sensor: false,
            adblue_def: false,
            swirl_flap: false,
            speed_limiter: false,
            speed_limit_kmh: 40.0,
            torque_derate: false,
            throttle_remap: false,
            dtc_mask: Vec::new(),
        }
    }
}

// ── ROM bit field descriptor ──────────────────────────────────────────────────
#[derive(Clone)]
pub struct RomBitField {
    pub name: &'static str,
    pub description: &'static str,
    pub addr: u32,
    pub bit_mask: u8,
    pub active_high: bool,
    pub category: PatchCategory,
}

// ── Main ECU ROM struct ───────────────────────────────────────────────────────
pub struct EcuRom {
    // Raw virtual ROM bytes
    pub rom: Vec<u8>,
    // 3D calibration maps
    pub fuel_map: CalMap3D,
    pub ignition_map: CalMap3D,
    pub boost_map: CalMap3D,
    pub lambda_map: CalMap3D,
    pub ve_map: CalMap3D,
    pub egr_map: CalMap3D,
    // 2D calibration curves
    pub idle_speed: CalMap2D,
    pub torque_limit: CalMap2D,
    pub boost_limit: CalMap2D,
    pub injector_timing: CalMap2D,
    pub fuel_pressure_target: CalMap2D,
    pub start_advance: CalMap2D,
    // ROM patches
    pub patches: Vec<RomPatch>,
    // Bit fields
    pub bit_fields: Vec<RomBitField>,
    // Named regions
    pub regions: Vec<RomRegion>,
    // Delete flags applied to ECM simulation
    pub deletes: DeleteFlags,
    // Unsaved flag
    pub dirty: bool,
    // ECU metadata
    pub ecu_part_number: String,
    pub ecu_sw_version: String,
    pub ecu_hw_version: String,
    pub calibration_id: String,
    pub checksum: u32,
}

impl EcuRom {
    pub fn new() -> Self {
        let mut rom = vec![0xFFu8; ROM_SIZE];
        // Write ECU header
        let header = b"CATERPILLAR-ECM-C7.2-T4F-SW3.14-CAL9.8.1\x00";
        rom[REGION_HEADER as usize..REGION_HEADER as usize + header.len()]
            .copy_from_slice(header);

        let mut ecm = Self {
            rom,
            fuel_map: make_fuel_map(),
            ignition_map: make_ignition_map(),
            boost_map: make_boost_map(),
            lambda_map: make_lambda_map(),
            ve_map: make_ve_map(),
            egr_map: make_egr_map(),
            idle_speed: make_idle_speed_curve(),
            torque_limit: make_torque_limit_curve(),
            boost_limit: make_boost_limit_curve(),
            injector_timing: make_injector_timing_curve(),
            fuel_pressure_target: make_fuel_pressure_curve(),
            start_advance: make_start_advance_curve(),
            patches: make_patches(),
            bit_fields: make_bit_fields(),
            regions: make_regions(),
            deletes: DeleteFlags::default(),
            dirty: false,
            ecu_part_number: "CAT-439-5743-02".into(),
            ecu_sw_version: "SW-3.14.2".into(),
            ecu_hw_version: "HW-1.0.5".into(),
            calibration_id: "CAL-9.8.1-T4FINAL".into(),
            checksum: 0xDEAD_BEEF,
        };
        ecm.sync_maps_to_rom();
        ecm
    }

    /// Serialise all map data into the virtual ROM bytes.
    pub fn sync_maps_to_rom(&mut self) {
        encode_map3d(&self.fuel_map, &mut self.rom);
        encode_map3d(&self.ignition_map, &mut self.rom);
        encode_map3d(&self.boost_map, &mut self.rom);
        encode_map3d(&self.lambda_map, &mut self.rom);
        encode_map3d(&self.ve_map, &mut self.rom);
        encode_map3d(&self.egr_map, &mut self.rom);
        encode_map2d(&self.idle_speed, &mut self.rom);
        encode_map2d(&self.torque_limit, &mut self.rom);
        encode_map2d(&self.boost_limit, &mut self.rom);
        encode_map2d(&self.injector_timing, &mut self.rom);
        encode_map2d(&self.fuel_pressure_target, &mut self.rom);
        encode_map2d(&self.start_advance, &mut self.rom);
    }

    /// Read a ROM byte, applying any enabled patches.
    pub fn read_byte(&self, addr: u32) -> u8 {
        for p in &self.patches {
            if p.enabled && p.addr == addr {
                return p.patched;
            }
        }
        self.rom.get(addr as usize).copied().unwrap_or(0xFF)
    }

    /// Write a byte to the virtual ROM (marks dirty).
    pub fn write_byte(&mut self, addr: u32, val: u8) {
        if let Some(b) = self.rom.get_mut(addr as usize) {
            *b = val;
            self.dirty = true;
        }
    }

    /// Apply/remove a patch by index.
    pub fn toggle_patch(&mut self, idx: usize) {
        if let Some(p) = self.patches.get_mut(idx) {
            p.enabled = !p.enabled;
            self.dirty = true;
        }
    }

    /// Toggle a bit field in the ROM.
    pub fn toggle_bit_field(&mut self, idx: usize) {
        if let Some(bf) = self.bit_fields.get(idx) {
            let addr = bf.addr as usize;
            if addr < ROM_SIZE {
                self.rom[addr] ^= bf.bit_mask;
                self.dirty = true;
            }
        }
    }

    pub fn bit_field_active(&self, idx: usize) -> bool {
        if let Some(bf) = self.bit_fields.get(idx) {
            let val = self.rom.get(bf.addr as usize).copied().unwrap_or(0);
            let set = (val & bf.bit_mask) != 0;
            if bf.active_high { set } else { !set }
        } else {
            false
        }
    }

    pub fn region_name_at(&self, addr: u32) -> &'static str {
        for r in &self.regions {
            if addr >= r.base && addr < r.base + r.size {
                return r.name;
            }
        }
        "Unallocated"
    }
}

// ─ Encode helpers ─────────────────────────────────────────────────────────────

fn encode_map3d(map: &CalMap3D, rom: &mut Vec<u8>) {
    let base = map.rom_base as usize;
    let range = map.max - map.min;
    for row in 0..MAP_LOAD_BINS {
        for col in 0..MAP_RPM_BINS {
            let off = base + (row * MAP_RPM_BINS + col) * 2;
            if off + 1 < ROM_SIZE {
                let norm = if range > 0.0 {
                    ((map.data[row][col] - map.min) / range * 65535.0) as u16
                } else { 0 };
                rom[off] = (norm >> 8) as u8;
                rom[off + 1] = (norm & 0xFF) as u8;
            }
        }
    }
}

fn encode_map2d(map: &CalMap2D, rom: &mut Vec<u8>) {
    let base = map.rom_base as usize;
    let range = map.max - map.min;
    for i in 0..MAP_2D_BINS {
        let off = base + i * 2;
        if off + 1 < ROM_SIZE {
            let norm = if range > 0.0 {
                ((map.data[i] - map.min) / range * 65535.0) as u16
            } else { 0 };
            rom[off] = (norm >> 8) as u8;
            rom[off + 1] = (norm & 0xFF) as u8;
        }
    }
}

// ─ Map factory functions ──────────────────────────────────────────────────────

fn make_fuel_map() -> CalMap3D {
    // Injection quantity mm³/stroke, 16×16 RPM×Load
    // Realistic Tier 4 diesel: peaks ~115 mm³ at 1400 RPM / 100% load
    let mut data = [[0.0f64; MAP_RPM_BINS]; MAP_LOAD_BINS];
    for (ri, &load) in LOAD_AXIS.iter().enumerate() {
        for (ci, &rpm) in RPM_AXIS.iter().enumerate() {
            let load_f = load / 100.0;
            // Torque peak ~1200-1600 RPM
            let rpm_factor = {
                let r = rpm / 1400.0;
                if r < 1.0 { 0.6 + 0.4 * r } else { 1.0 - (r - 1.0) * 0.35 }.clamp(0.0, 1.0)
            };
            // High-load injection ceiling ~115 mm³
            let qty = load_f * 115.0 * rpm_factor + (1.0 - load_f) * 1.2;
            data[ri][ci] = qty.clamp(0.5, 120.0);
        }
    }
    CalMap3D {
        name: "Fuel Injection Quantity",
        unit: "mm³/stroke",
        x_label: "RPM",
        y_label: "Load %",
        x_axis: RPM_AXIS,
        y_axis: LOAD_AXIS,
        data,
        min: 0.0,
        max: 120.0,
        rom_base: REGION_FUEL_MAP,
        description: "Main fuel injection quantity map. Directly controls injected fuel per stroke. Higher values = more power/smoke.",
    }
}

fn make_ignition_map() -> CalMap3D {
    // Injection advance degrees BTDC, 16×16
    let mut data = [[0.0f64; MAP_RPM_BINS]; MAP_LOAD_BINS];
    for (ri, &load) in LOAD_AXIS.iter().enumerate() {
        for (ci, &rpm) in RPM_AXIS.iter().enumerate() {
            let load_f = load / 100.0;
            // Advance peaks ~18° at mid-range, retards under high load (combustion noise)
            let base = 8.0 + (rpm / 2600.0).min(1.0) * 10.0;
            let retard = load_f * 4.5;
            data[ri][ci] = (base - retard).clamp(-5.0, 25.0);
        }
    }
    CalMap3D {
        name: "Injection Advance",
        unit: "deg BTDC",
        x_label: "RPM",
        y_label: "Load %",
        x_axis: RPM_AXIS,
        y_axis: LOAD_AXIS,
        data,
        min: -5.0,
        max: 25.0,
        rom_base: REGION_IGN_MAP,
        description: "Injection timing advance. Higher = earlier injection = more power. Too high → knock. Too low → smoke.",
    }
}

fn make_boost_map() -> CalMap3D {
    // VGT boost target kPa absolute, 16×16
    let mut data = [[0.0f64; MAP_RPM_BINS]; MAP_LOAD_BINS];
    for (ri, &load) in LOAD_AXIS.iter().enumerate() {
        for (ci, &rpm) in RPM_AXIS.iter().enumerate() {
            let load_f = load / 100.0;
            // Peak boost 220 kPa at 1400 RPM / full load; falls off at idle/high RPM
            let rpm_f = {
                let r = rpm / 1400.0;
                if r < 1.0 { r } else { 1.0 - (r - 1.0) * 0.2 }.clamp(0.0, 1.0)
            };
            let boost = 101.0 + load_f * rpm_f * 119.0;
            data[ri][ci] = boost.clamp(101.0, 250.0);
        }
    }
    CalMap3D {
        name: "VGT Boost Target",
        unit: "kPa abs",
        x_label: "RPM",
        y_label: "Load %",
        x_axis: RPM_AXIS,
        y_axis: LOAD_AXIS,
        data,
        min: 90.0,
        max: 280.0,
        rom_base: REGION_BOOST_MAP,
        description: "Target boost pressure. Controls VGT vane position. Higher = more air = more power but more heat.",
    }
}

fn make_lambda_map() -> CalMap3D {
    // Lambda target (excess air ratio), 16×16 — diesel always > 1.0
    let mut data = [[0.0f64; MAP_RPM_BINS]; MAP_LOAD_BINS];
    for (ri, &load) in LOAD_AXIS.iter().enumerate() {
        for (ci, &rpm) in RPM_AXIS.iter().enumerate() {
            let load_f = load / 100.0;
            // Idle: very lean (~5.0); full load: ~1.2 (just above stoich)
            let _ = rpm;
            let lam = 5.0 - load_f * 3.8;
            data[ri][ci] = lam.clamp(1.05, 8.0);
        }
    }
    CalMap3D {
        name: "Lambda Target (Air/Fuel)",
        unit: "λ",
        x_label: "RPM",
        y_label: "Load %",
        x_axis: RPM_AXIS,
        y_axis: LOAD_AXIS,
        data,
        min: 1.0,
        max: 8.0,
        rom_base: REGION_LAMBDA_MAP,
        description: "Target lambda. Diesel always lean (>1). At full load, minimum ~1.15 for smoke limit. Affects EGR and fuelling.",
    }
}

fn make_ve_map() -> CalMap3D {
    // Volumetric Efficiency %, 16×16
    let mut data = [[0.0f64; MAP_RPM_BINS]; MAP_LOAD_BINS];
    for (ri, _) in LOAD_AXIS.iter().enumerate() {
        for (ci, &rpm) in RPM_AXIS.iter().enumerate() {
            // VE peaks ~105% at 1200-1600 RPM (supercharging region)
            let r = rpm / 1400.0;
            let ve = if r < 1.0 { 75.0 + 30.0 * r } else { 105.0 - (r - 1.0) * 18.0 };
            data[ri][ci] = ve.clamp(60.0, 115.0);
        }
    }
    CalMap3D {
        name: "Volumetric Efficiency",
        unit: "%",
        x_label: "RPM",
        y_label: "Load %",
        x_axis: RPM_AXIS,
        y_axis: LOAD_AXIS,
        data,
        min: 55.0,
        max: 120.0,
        rom_base: REGION_VE_MAP,
        description: "Cylinder filling efficiency. Used for MAF-based fuelling. Affects accuracy of air mass calculation.",
    }
}

fn make_egr_map() -> CalMap3D {
    // EGR valve position command %, 16×16
    let mut data = [[0.0f64; MAP_RPM_BINS]; MAP_LOAD_BINS];
    for (ri, &load) in LOAD_AXIS.iter().enumerate() {
        for (ci, &rpm) in RPM_AXIS.iter().enumerate() {
            let load_f = load / 100.0;
            // EGR active only at part load (emissions, NOx reduction)
            // Closed at idle (<10% load) and high load (>75%)
            let rpm_f = ((rpm - 800.0) / 1000.0).clamp(0.0, 1.0);
            let egr = if load_f < 0.1 || load_f > 0.75 { 0.0 }
                else { (1.0 - (2.0 * load_f - 0.85).powi(2)) * 35.0 * rpm_f };
            data[ri][ci] = egr.clamp(0.0, 45.0);
        }
    }
    CalMap3D {
        name: "EGR Valve Position",
        unit: "% open",
        x_label: "RPM",
        y_label: "Load %",
        x_axis: RPM_AXIS,
        y_axis: LOAD_AXIS,
        data,
        min: 0.0,
        max: 50.0,
        rom_base: REGION_EGR_MAP,
        description: "EGR valve opening command. Higher = more exhaust recirculation = less NOx but more PM and heat. Zero = EGR delete.",
    }
}

// ─ 2D curve factories ─────────────────────────────────────────────────────────

fn rpm_axis_2d() -> [f64; MAP_2D_BINS] {
    [500.0, 600.0, 700.0, 800.0, 1000.0, 1200.0, 1400.0, 1600.0,
     1800.0, 2000.0, 2100.0, 2200.0, 2300.0, 2400.0, 2500.0, 2600.0]
}

fn make_idle_speed_curve() -> CalMap2D {
    // Idle speed target rpm vs coolant temp
    let x_axis = [-20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0,
                   60.0, 70.0, 80.0, 90.0, 95.0, 100.0, 105.0, 110.0];
    let data = [1050.0, 1000.0, 950.0, 900.0, 870.0, 850.0, 840.0, 830.0,
                820.0, 810.0, 800.0, 800.0, 800.0, 800.0, 800.0, 800.0];
    CalMap2D {
        name: "Idle Speed Target",
        unit: "rpm",
        x_label: "Coolant Temp °C",
        x_axis,
        data,
        min: 650.0,
        max: 1200.0,
        rom_base: REGION_2D_CURVES,
        description: "Idle RPM target as a function of coolant temperature. Cold starts require higher idle.",
    }
}

fn make_torque_limit_curve() -> CalMap2D {
    // Max torque Nm vs RPM
    let data = [400.0, 500.0, 650.0, 800.0, 950.0, 1020.0, 1050.0, 1050.0,
                1000.0, 950.0, 880.0, 800.0, 720.0, 640.0, 560.0, 480.0];
    CalMap2D {
        name: "Torque Limit",
        unit: "Nm",
        x_label: "RPM",
        x_axis: rpm_axis_2d(),
        data,
        min: 0.0,
        max: 1200.0,
        rom_base: REGION_2D_CURVES + 0x40,
        description: "Maximum engine torque vs RPM. Acts as a hard ceiling for fuelling. Derate may reduce this limit.",
    }
}

fn make_boost_limit_curve() -> CalMap2D {
    // Max boost kPa vs RPM
    let data = [120.0, 130.0, 145.0, 160.0, 185.0, 210.0, 235.0, 250.0,
                245.0, 235.0, 225.0, 215.0, 205.0, 195.0, 185.0, 175.0];
    CalMap2D {
        name: "Boost Pressure Limit",
        unit: "kPa abs",
        x_label: "RPM",
        x_axis: rpm_axis_2d(),
        data,
        min: 100.0,
        max: 300.0,
        rom_base: REGION_2D_CURVES + 0x80,
        description: "Maximum allowable boost pressure vs RPM. Protects intercooler and inlet manifold.",
    }
}

fn make_injector_timing_curve() -> CalMap2D {
    // Injector energise timing µs (pilot injection pulse width) vs RPM
    let data = [1200.0, 1150.0, 1100.0, 1060.0, 1020.0, 980.0, 950.0, 920.0,
                900.0, 880.0, 860.0, 840.0, 820.0, 800.0, 780.0, 760.0];
    CalMap2D {
        name: "Pilot Injection Width",
        unit: "µs",
        x_label: "RPM",
        x_axis: rpm_axis_2d(),
        data,
        min: 400.0,
        max: 1500.0,
        rom_base: REGION_2D_CURVES + 0xC0,
        description: "Pilot (pre) injection pulse width. Controls NVH and combustion noise. Disable to remove diesel clatter.",
    }
}

fn make_fuel_pressure_curve() -> CalMap2D {
    // Common rail target pressure MPa vs RPM
    let data = [60.0, 70.0, 80.0, 90.0, 105.0, 120.0, 140.0, 155.0,
                165.0, 170.0, 170.0, 165.0, 160.0, 155.0, 150.0, 145.0];
    CalMap2D {
        name: "Rail Pressure Target",
        unit: "MPa",
        x_label: "RPM",
        x_axis: rpm_axis_2d(),
        data,
        min: 20.0,
        max: 200.0,
        rom_base: REGION_2D_CURVES + 0x100,
        description: "Common rail target pressure. Higher = better atomisation = more power. Too high at low RPM = pump stress.",
    }
}

fn make_start_advance_curve() -> CalMap2D {
    // Start-of-injection advance deg vs coolant temp
    let x_axis = [-20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0, 50.0,
                   60.0, 70.0, 80.0, 90.0, 95.0, 100.0, 105.0, 110.0];
    let data = [24.0, 22.0, 20.0, 18.0, 16.0, 15.0, 14.0, 13.5,
                13.0, 12.5, 12.0, 11.5, 11.0, 10.5, 10.0, 9.5];
    CalMap2D {
        name: "Cold Start Advance",
        unit: "deg BTDC",
        x_label: "Coolant Temp °C",
        x_axis,
        data,
        min: 5.0,
        max: 30.0,
        rom_base: REGION_2D_CURVES + 0x140,
        description: "Additional injection advance for cold start. More advance = easier start in cold conditions.",
    }
}

// ─ Patch list ─────────────────────────────────────────────────────────────────

fn make_patches() -> Vec<RomPatch> {
    vec![
        RomPatch { name: "DPF Regen Disable",
            description: "Inhibits active DPF regeneration cycle. Prevents uncontrolled regen events.",
            addr: REGION_AFTERTREATMENT + 0x00, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Aftertreatment },
        RomPatch { name: "DPF Soot Level Mask",
            description: "Forces soot level read to 0% — prevents soot fault codes.",
            addr: REGION_AFTERTREATMENT + 0x02, original: 0x80, patched: 0x00,
            enabled: false, category: PatchCategory::Aftertreatment },
        RomPatch { name: "EGR Valve Force Closed",
            description: "Forces EGR valve command to 0% at all operating points.",
            addr: REGION_EGR_MAP as u32 + 0x00, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Aftertreatment },
        RomPatch { name: "EGR Fault Mask",
            description: "Prevents EGR position fault codes (P0403, P0406, P0404).",
            addr: REGION_AFTERTREATMENT + 0x10, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Diagnostics },
        RomPatch { name: "O2/Lambda Plausibility Disable",
            description: "Removes O2 sensor plausibility check. Prevents P0136/P0137 when sensor removed.",
            addr: REGION_AFTERTREATMENT + 0x20, original: 0xFF, patched: 0x00,
            enabled: false, category: PatchCategory::Sensors },
        RomPatch { name: "NOx Sensor Delete",
            description: "Disables NOx sensor check. Prevents P2200/P2201 after sensor delete.",
            addr: REGION_AFTERTREATMENT + 0x22, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Sensors },
        RomPatch { name: "AdBlue/DEF Pressure Delete",
            description: "Disables DEF dosing fault and low-level derate. Prevents SCR fault shutdown.",
            addr: REGION_AFTERTREATMENT + 0x30, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Aftertreatment },
        RomPatch { name: "Swirl Flap Delete",
            description: "Forces swirl flaps fully open. Prevents P2004/P2005 after flap removal.",
            addr: REGION_ENGINE_BASE + 0x50, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Other },
        RomPatch { name: "Speed Limiter Remove",
            description: "Removes ECU-enforced vehicle speed limit. Only modify legal off-road vehicles.",
            addr: REGION_LIMITS + 0x00, original: 0x32, patched: 0xFF,
            enabled: false, category: PatchCategory::Limits },
        RomPatch { name: "Torque Derate Disable",
            description: "Prevents torque reduction under thermal derate conditions.",
            addr: REGION_LIMITS + 0x10, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Limits },
        RomPatch { name: "Fuel Cutoff RPM Raise",
            description: "Raises over-rev fuel cut from 2600 to 2800 RPM.",
            addr: REGION_ENGINE_BASE + 0x20, original: 0x0A, patched: 0x0B,
            enabled: false, category: PatchCategory::Fuel },
        RomPatch { name: "Pilot Injection Disable",
            description: "Removes pilot (pre) injection. Reduces NVH, may increase noise at cold.",
            addr: REGION_2D_CURVES + 0xC0, original: 0x04, patched: 0x00,
            enabled: false, category: PatchCategory::Fuel },
        RomPatch { name: "Start/Stop Inhibit",
            description: "Disables automatic engine stop-start system.",
            addr: REGION_ENGINE_BASE + 0x60, original: 0x01, patched: 0x00,
            enabled: false, category: PatchCategory::Other },
        RomPatch { name: "Throttle Sensitivity Remap",
            description: "Linearises throttle pedal response (removes drive-by-wire dead zone).",
            addr: REGION_ENGINE_BASE + 0x70, original: 0x14, patched: 0x0A,
            enabled: false, category: PatchCategory::Fuel },
        RomPatch { name: "Cold Start Enrichment Boost +10%",
            description: "Increases cold start fuelling by 10% for easier cold cranking.",
            addr: REGION_ENGINE_BASE + 0x80, original: 0x1E, patched: 0x21,
            enabled: false, category: PatchCategory::Fuel },
        RomPatch { name: "Boost Limit +20 kPa",
            description: "Raises boost ceiling by 20 kPa across all RPM.",
            addr: REGION_2D_CURVES + 0x80, original: 0xFA, patched: 0xFF,
            enabled: false, category: PatchCategory::Limits },
    ]
}

// ─ Bit field list ─────────────────────────────────────────────────────────────

fn make_bit_fields() -> Vec<RomBitField> {
    vec![
        RomBitField { name: "DPF Regen Enable", description: "Bit 0 of AT_CTRL: enables active DPF regen",
            addr: REGION_AFTERTREATMENT, bit_mask: 0x01, active_high: true, category: PatchCategory::Aftertreatment },
        RomBitField { name: "EGR Enable", description: "Bit 0 of EGR_CTRL: enables EGR valve",
            addr: REGION_EGR_MAP as u32, bit_mask: 0x01, active_high: true, category: PatchCategory::Aftertreatment },
        RomBitField { name: "SCR/DEF Enable", description: "Bit 4 of AT_CTRL: enables SCR dosing",
            addr: REGION_AFTERTREATMENT, bit_mask: 0x10, active_high: true, category: PatchCategory::Aftertreatment },
        RomBitField { name: "O2 Plausibility Check", description: "Bit 0 of SENSOR_PLAUS",
            addr: REGION_AFTERTREATMENT + 0x20, bit_mask: 0xFF, active_high: true, category: PatchCategory::Sensors },
        RomBitField { name: "Speed Limiter Active", description: "Bit 0 of LIMIT_FLAGS",
            addr: REGION_LIMITS, bit_mask: 0x01, active_high: true, category: PatchCategory::Limits },
        RomBitField { name: "Torque Derate Active", description: "Bit 4 of LIMIT_FLAGS",
            addr: REGION_LIMITS, bit_mask: 0x10, active_high: true, category: PatchCategory::Limits },
        RomBitField { name: "Pilot Injection Enable", description: "Bit 2 of FUEL_FLAGS",
            addr: REGION_2D_CURVES + 0xC0, bit_mask: 0x04, active_high: true, category: PatchCategory::Fuel },
        RomBitField { name: "Cold Start Enrichment", description: "Bit 0 of START_FLAGS",
            addr: REGION_ENGINE_BASE + 0x80, bit_mask: 0x01, active_high: true, category: PatchCategory::Fuel },
        RomBitField { name: "Stop/Start System", description: "Bit 0 of IDLE_FLAGS",
            addr: REGION_ENGINE_BASE + 0x60, bit_mask: 0x01, active_high: true, category: PatchCategory::Other },
        RomBitField { name: "NOx Sensor Enable", description: "Bit 0 of NOX_FLAGS",
            addr: REGION_AFTERTREATMENT + 0x22, bit_mask: 0x01, active_high: true, category: PatchCategory::Sensors },
        RomBitField { name: "Swirl Flap Enable", description: "Bit 0 of INTAKE_FLAGS",
            addr: REGION_ENGINE_BASE + 0x50, bit_mask: 0x01, active_high: true, category: PatchCategory::Other },
        RomBitField { name: "Fuel Cut High RPM", description: "Bit 3 of FUEL_FLAGS: enables over-rev cut",
            addr: REGION_ENGINE_BASE + 0x20, bit_mask: 0x08, active_high: true, category: PatchCategory::Fuel },
        RomBitField { name: "Throttle Linear Mode", description: "Bit 0 of DBW_FLAGS",
            addr: REGION_ENGINE_BASE + 0x70, bit_mask: 0x01, active_high: true, category: PatchCategory::Fuel },
    ]
}

// ─ Region descriptors ─────────────────────────────────────────────────────────

fn make_regions() -> Vec<RomRegion> {
    vec![
        RomRegion { name: "ECU Header", base: REGION_HEADER, size: 0x1000, description: "Part number, software ID, calibration ID, checksums" },
        RomRegion { name: "Engine Base Config", base: REGION_ENGINE_BASE, size: 0x1000, description: "Base engine parameters, governor settings, limits" },
        RomRegion { name: "Fuel Injection Map (3D)", base: REGION_FUEL_MAP, size: 0x1000, description: "Main 16×16 fuel injection quantity map" },
        RomRegion { name: "Injection Timing (3D)", base: REGION_IGN_MAP, size: 0x1000, description: "Injection advance map (degrees BTDC)" },
        RomRegion { name: "VGT Boost Target (3D)", base: REGION_BOOST_MAP, size: 0x1000, description: "Variable geometry turbo boost target map" },
        RomRegion { name: "Lambda Target (3D)", base: REGION_LAMBDA_MAP, size: 0x0800, description: "Target lambda / air-fuel ratio map" },
        RomRegion { name: "VE Map (3D)", base: REGION_VE_MAP, size: 0x0800, description: "Volumetric efficiency table for MAF correction" },
        RomRegion { name: "EGR Valve Map (3D)", base: REGION_EGR_MAP, size: 0x1000, description: "EGR valve opening command map" },
        RomRegion { name: "2D Calibration Curves", base: REGION_2D_CURVES, size: 0x1000, description: "Idle speed, torque limit, boost limit, injector timing" },
        RomRegion { name: "Aftertreatment Cal", base: REGION_AFTERTREATMENT, size: 0x1000, description: "DPF/SCR/EGR delete flags and thresholds" },
        RomRegion { name: "Operating Limits", base: REGION_LIMITS, size: 0x1000, description: "Speed limiter, torque derate, temperature protection" },
    ]
}
