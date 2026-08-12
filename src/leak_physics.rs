//! O-ring leak / rupture physics for hydraulic, oil and refrigerant circuits.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LeakModelCoefficients {
    pub damage_rate_scale: f64,
    pub extrusion_rate_scale: f64,
    pub thermal_rate_scale: f64,
    pub flow_rate_scale: f64,
    pub rupture_area_scale: f64,
}

impl Default for LeakModelCoefficients {
    fn default() -> Self {
        Self {
            damage_rate_scale: 1.0,
            extrusion_rate_scale: 1.0,
            thermal_rate_scale: 1.0,
            flow_rate_scale: 1.0,
            rupture_area_scale: 1.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationCsvSample {
    pub timestamp_s: Option<f64>,
    pub circuit_name: String,
    pub pressure_bar: f64,
    pub delta_p_bar: f64,
    pub temp_c: f64,
    pub cycles_per_s: f64,
    pub duty_01: f64,
    pub fluid_density_kg_m3: f64,
    pub measured_leak_lpm: f64,
    pub observed_rupture: Option<bool>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationCircuitReport {
    pub circuit_name: String,
    pub material: String,
    pub oil: String,
    pub sample_count: usize,
    pub rmse_leak_lpm: f64,
    pub mape_leak_pct: f64,
    pub max_abs_error_lpm: f64,
    pub rupture_accuracy_pct: f64,
    pub fitted: LeakModelCoefficients,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationGroupReport {
    pub group: String,
    pub sample_count: usize,
    pub mean_rmse_lpm: f64,
    pub mean_mape_pct: f64,
    pub mean_rupture_accuracy_pct: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CalibrationReport {
    pub total_samples: usize,
    pub calibrated_circuits: usize,
    pub circuit_reports: Vec<CalibrationCircuitReport>,
    pub by_material: Vec<CalibrationGroupReport>,
    pub by_oil: Vec<CalibrationGroupReport>,
}

fn aggregate_group_report<F>(
    items: &[CalibrationCircuitReport],
    mut key_fn: F,
) -> Vec<CalibrationGroupReport>
where
    F: FnMut(&CalibrationCircuitReport) -> String,
{
    use std::collections::BTreeMap;
    let mut acc: BTreeMap<String, (usize, f64, f64, f64, usize)> = BTreeMap::new();
    for r in items {
        let key = key_fn(r);
        let entry = acc.entry(key).or_insert((0, 0.0, 0.0, 0.0, 0));
        entry.0 += 1;
        entry.1 += r.rmse_leak_lpm;
        entry.2 += r.mape_leak_pct;
        entry.3 += r.rupture_accuracy_pct;
        entry.4 += r.sample_count;
    }

    let mut out = Vec::with_capacity(acc.len());
    for (group, (count, rmse, mape, rupture_acc, sample_count)) in acc {
        let n = count.max(1) as f64;
        out.push(CalibrationGroupReport {
            group,
            sample_count,
            mean_rmse_lpm: rmse / n,
            mean_mape_pct: mape / n,
            mean_rupture_accuracy_pct: rupture_acc / n,
        });
    }
    out
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum LeakAlertLevel {
    Normal,
    Watch,
    Warning,
    Critical,
    Ruptured,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PressureBand {
    BelowMinimum,
    LowBand,
    MidBand,
    HighBand,
    OverMaximum,
    AtRupture,
}

impl std::fmt::Display for PressureBand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            PressureBand::BelowMinimum => "below_min",
            PressureBand::LowBand => "low",
            PressureBand::MidBand => "mid",
            PressureBand::HighBand => "high",
            PressureBand::OverMaximum => "over_max",
            PressureBand::AtRupture => "at_rupture",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CircuitComponent {
    Oring,
    Seal,
    AcHose,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OilType {
    HydraulicIso22,
    HydraulicIso32,
    HydraulicIso46,
    HydraulicIso68,
    HydraulicIso100,
    HydraulicIso150,
    HydraulicIso220,
    Aw32,
    Aw46,
    Aw68,
    Utto10w30,
    Utto80w,
    Gear75w90,
    Gear80w90,
    AtfDexronIii,
    AtfDexronVi,
    Engine0w20,
    Engine5w30,
    Engine5w40,
    Engine10w30,
    Engine15w40,
    Engine20w50,
    TransformerInhibited,
    Turbine32,
    Turbine46,
    Compressor46,
    Compressor68,
    BiodegradableHees46,
    BiodegradableHepr46,
    FireResistantHfdu46,
    BrakeDot3,
    BrakeDot4,
    BrakeDot51,
    SyntheticEsters100,
    RefrigerationMineral3gs,
    Pag46,
    Pag100,
    Pag150,
    Poe32,
    Poe46,
    Poe68,
    Poe100,
    VacuumPump100,
    Custom,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct OilRuntimeProfile {
    pub label: &'static str,
    pub density_kg_m3: f64,
    pub viscosity_index: f64,
    pub min_oper_temp_c: f64,
    pub max_oper_temp_c: f64,
}

const OIL_TYPE_VARIANTS: [OilType; 42] = [
    OilType::HydraulicIso22,
    OilType::HydraulicIso32,
    OilType::HydraulicIso46,
    OilType::HydraulicIso68,
    OilType::HydraulicIso100,
    OilType::HydraulicIso150,
    OilType::HydraulicIso220,
    OilType::Aw32,
    OilType::Aw46,
    OilType::Aw68,
    OilType::Utto10w30,
    OilType::Utto80w,
    OilType::Gear75w90,
    OilType::Gear80w90,
    OilType::AtfDexronIii,
    OilType::AtfDexronVi,
    OilType::Engine0w20,
    OilType::Engine5w30,
    OilType::Engine5w40,
    OilType::Engine10w30,
    OilType::Engine15w40,
    OilType::Engine20w50,
    OilType::TransformerInhibited,
    OilType::Turbine32,
    OilType::Turbine46,
    OilType::Compressor46,
    OilType::Compressor68,
    OilType::BiodegradableHees46,
    OilType::BiodegradableHepr46,
    OilType::FireResistantHfdu46,
    OilType::BrakeDot3,
    OilType::BrakeDot4,
    OilType::BrakeDot51,
    OilType::SyntheticEsters100,
    OilType::RefrigerationMineral3gs,
    OilType::Pag46,
    OilType::Pag100,
    OilType::Pag150,
    OilType::Poe32,
    OilType::Poe46,
    OilType::Poe68,
    OilType::Poe100,
];

impl OilType {
    pub fn all() -> &'static [OilType] {
        &OIL_TYPE_VARIANTS
    }

    pub fn name(self) -> &'static str {
        self.runtime_profile().label
    }

    pub fn runtime_profile(self) -> OilRuntimeProfile {
        match self {
            OilType::HydraulicIso22 => OilRuntimeProfile { label: "Hydraulic ISO VG 22", density_kg_m3: 852.0, viscosity_index: 108.0, min_oper_temp_c: -20.0, max_oper_temp_c: 85.0 },
            OilType::HydraulicIso32 => OilRuntimeProfile { label: "Hydraulic ISO VG 32", density_kg_m3: 858.0, viscosity_index: 106.0, min_oper_temp_c: -18.0, max_oper_temp_c: 90.0 },
            OilType::HydraulicIso46 => OilRuntimeProfile { label: "Hydraulic ISO VG 46", density_kg_m3: 865.0, viscosity_index: 105.0, min_oper_temp_c: -15.0, max_oper_temp_c: 95.0 },
            OilType::HydraulicIso68 => OilRuntimeProfile { label: "Hydraulic ISO VG 68", density_kg_m3: 875.0, viscosity_index: 102.0, min_oper_temp_c: -10.0, max_oper_temp_c: 100.0 },
            OilType::HydraulicIso100 => OilRuntimeProfile { label: "Hydraulic ISO VG 100", density_kg_m3: 882.0, viscosity_index: 100.0, min_oper_temp_c: -8.0, max_oper_temp_c: 105.0 },
            OilType::HydraulicIso150 => OilRuntimeProfile { label: "Hydraulic ISO VG 150", density_kg_m3: 890.0, viscosity_index: 98.0, min_oper_temp_c: -5.0, max_oper_temp_c: 110.0 },
            OilType::HydraulicIso220 => OilRuntimeProfile { label: "Hydraulic ISO VG 220", density_kg_m3: 896.0, viscosity_index: 97.0, min_oper_temp_c: 0.0, max_oper_temp_c: 115.0 },
            OilType::Aw32 => OilRuntimeProfile { label: "AW Hydraulic 32", density_kg_m3: 860.0, viscosity_index: 110.0, min_oper_temp_c: -20.0, max_oper_temp_c: 95.0 },
            OilType::Aw46 => OilRuntimeProfile { label: "AW Hydraulic 46", density_kg_m3: 868.0, viscosity_index: 108.0, min_oper_temp_c: -18.0, max_oper_temp_c: 100.0 },
            OilType::Aw68 => OilRuntimeProfile { label: "AW Hydraulic 68", density_kg_m3: 878.0, viscosity_index: 105.0, min_oper_temp_c: -12.0, max_oper_temp_c: 105.0 },
            OilType::Utto10w30 => OilRuntimeProfile { label: "UTTO 10W-30", density_kg_m3: 872.0, viscosity_index: 142.0, min_oper_temp_c: -30.0, max_oper_temp_c: 115.0 },
            OilType::Utto80w => OilRuntimeProfile { label: "UTTO 80W", density_kg_m3: 885.0, viscosity_index: 128.0, min_oper_temp_c: -20.0, max_oper_temp_c: 120.0 },
            OilType::Gear75w90 => OilRuntimeProfile { label: "Gear Oil 75W-90", density_kg_m3: 875.0, viscosity_index: 155.0, min_oper_temp_c: -40.0, max_oper_temp_c: 140.0 },
            OilType::Gear80w90 => OilRuntimeProfile { label: "Gear Oil 80W-90", density_kg_m3: 890.0, viscosity_index: 135.0, min_oper_temp_c: -26.0, max_oper_temp_c: 140.0 },
            OilType::AtfDexronIii => OilRuntimeProfile { label: "ATF Dexron III", density_kg_m3: 850.0, viscosity_index: 180.0, min_oper_temp_c: -40.0, max_oper_temp_c: 130.0 },
            OilType::AtfDexronVi => OilRuntimeProfile { label: "ATF Dexron VI", density_kg_m3: 846.0, viscosity_index: 165.0, min_oper_temp_c: -45.0, max_oper_temp_c: 135.0 },
            OilType::Engine0w20 => OilRuntimeProfile { label: "Engine 0W-20", density_kg_m3: 845.0, viscosity_index: 170.0, min_oper_temp_c: -40.0, max_oper_temp_c: 125.0 },
            OilType::Engine5w30 => OilRuntimeProfile { label: "Engine 5W-30", density_kg_m3: 855.0, viscosity_index: 165.0, min_oper_temp_c: -35.0, max_oper_temp_c: 130.0 },
            OilType::Engine5w40 => OilRuntimeProfile { label: "Engine 5W-40", density_kg_m3: 862.0, viscosity_index: 162.0, min_oper_temp_c: -35.0, max_oper_temp_c: 135.0 },
            OilType::Engine10w30 => OilRuntimeProfile { label: "Engine 10W-30", density_kg_m3: 870.0, viscosity_index: 150.0, min_oper_temp_c: -30.0, max_oper_temp_c: 130.0 },
            OilType::Engine15w40 => OilRuntimeProfile { label: "Engine 15W-40", density_kg_m3: 880.0, viscosity_index: 140.0, min_oper_temp_c: -25.0, max_oper_temp_c: 135.0 },
            OilType::Engine20w50 => OilRuntimeProfile { label: "Engine 20W-50", density_kg_m3: 892.0, viscosity_index: 132.0, min_oper_temp_c: -15.0, max_oper_temp_c: 140.0 },
            OilType::TransformerInhibited => OilRuntimeProfile { label: "Transformer Oil Inhibited", density_kg_m3: 885.0, viscosity_index: 95.0, min_oper_temp_c: -30.0, max_oper_temp_c: 110.0 },
            OilType::Turbine32 => OilRuntimeProfile { label: "Turbine Oil 32", density_kg_m3: 855.0, viscosity_index: 102.0, min_oper_temp_c: -10.0, max_oper_temp_c: 120.0 },
            OilType::Turbine46 => OilRuntimeProfile { label: "Turbine Oil 46", density_kg_m3: 865.0, viscosity_index: 100.0, min_oper_temp_c: -8.0, max_oper_temp_c: 125.0 },
            OilType::Compressor46 => OilRuntimeProfile { label: "Compressor Oil 46", density_kg_m3: 862.0, viscosity_index: 108.0, min_oper_temp_c: -15.0, max_oper_temp_c: 150.0 },
            OilType::Compressor68 => OilRuntimeProfile { label: "Compressor Oil 68", density_kg_m3: 872.0, viscosity_index: 105.0, min_oper_temp_c: -10.0, max_oper_temp_c: 155.0 },
            OilType::BiodegradableHees46 => OilRuntimeProfile { label: "HEES Biodegradable 46", density_kg_m3: 920.0, viscosity_index: 190.0, min_oper_temp_c: -35.0, max_oper_temp_c: 90.0 },
            OilType::BiodegradableHepr46 => OilRuntimeProfile { label: "HEPR Biodegradable 46", density_kg_m3: 900.0, viscosity_index: 165.0, min_oper_temp_c: -30.0, max_oper_temp_c: 105.0 },
            OilType::FireResistantHfdu46 => OilRuntimeProfile { label: "HFDU Fire Resistant 46", density_kg_m3: 930.0, viscosity_index: 150.0, min_oper_temp_c: -20.0, max_oper_temp_c: 160.0 },
            OilType::BrakeDot3 => OilRuntimeProfile { label: "Brake Fluid DOT3", density_kg_m3: 1035.0, viscosity_index: 220.0, min_oper_temp_c: -40.0, max_oper_temp_c: 205.0 },
            OilType::BrakeDot4 => OilRuntimeProfile { label: "Brake Fluid DOT4", density_kg_m3: 1060.0, viscosity_index: 230.0, min_oper_temp_c: -45.0, max_oper_temp_c: 230.0 },
            OilType::BrakeDot51 => OilRuntimeProfile { label: "Brake Fluid DOT5.1", density_kg_m3: 1045.0, viscosity_index: 240.0, min_oper_temp_c: -50.0, max_oper_temp_c: 240.0 },
            OilType::SyntheticEsters100 => OilRuntimeProfile { label: "Synthetic Ester 100", density_kg_m3: 915.0, viscosity_index: 175.0, min_oper_temp_c: -35.0, max_oper_temp_c: 180.0 },
            OilType::RefrigerationMineral3gs => OilRuntimeProfile { label: "Refrigeration Mineral 3GS", density_kg_m3: 895.0, viscosity_index: 85.0, min_oper_temp_c: -40.0, max_oper_temp_c: 80.0 },
            OilType::Pag46 => OilRuntimeProfile { label: "PAG 46", density_kg_m3: 995.0, viscosity_index: 130.0, min_oper_temp_c: -40.0, max_oper_temp_c: 170.0 },
            OilType::Pag100 => OilRuntimeProfile { label: "PAG 100", density_kg_m3: 1002.0, viscosity_index: 135.0, min_oper_temp_c: -35.0, max_oper_temp_c: 175.0 },
            OilType::Pag150 => OilRuntimeProfile { label: "PAG 150", density_kg_m3: 1010.0, viscosity_index: 140.0, min_oper_temp_c: -30.0, max_oper_temp_c: 180.0 },
            OilType::Poe32 => OilRuntimeProfile { label: "POE 32", density_kg_m3: 970.0, viscosity_index: 140.0, min_oper_temp_c: -45.0, max_oper_temp_c: 170.0 },
            OilType::Poe46 => OilRuntimeProfile { label: "POE 46", density_kg_m3: 975.0, viscosity_index: 143.0, min_oper_temp_c: -40.0, max_oper_temp_c: 175.0 },
            OilType::Poe68 => OilRuntimeProfile { label: "POE 68", density_kg_m3: 980.0, viscosity_index: 145.0, min_oper_temp_c: -35.0, max_oper_temp_c: 180.0 },
            OilType::Poe100 => OilRuntimeProfile { label: "POE 100", density_kg_m3: 988.0, viscosity_index: 148.0, min_oper_temp_c: -30.0, max_oper_temp_c: 185.0 },
            OilType::VacuumPump100 => OilRuntimeProfile { label: "Vacuum Pump 100", density_kg_m3: 885.0, viscosity_index: 98.0, min_oper_temp_c: -10.0, max_oper_temp_c: 135.0 },
            OilType::Custom => OilRuntimeProfile { label: "Custom", density_kg_m3: 900.0, viscosity_index: 120.0, min_oper_temp_c: -20.0, max_oper_temp_c: 120.0 },
        }
    }
}

impl OilType {
    pub fn density_kg_m3(self) -> f64 {
        self.runtime_profile().density_kg_m3
    }

    pub fn viscosity_index(self) -> f64 {
        self.runtime_profile().viscosity_index
    }
}

impl std::fmt::Display for LeakAlertLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            LeakAlertLevel::Normal => "NORMAL",
            LeakAlertLevel::Watch => "WATCH",
            LeakAlertLevel::Warning => "WARNING",
            LeakAlertLevel::Critical => "CRITICAL",
            LeakAlertLevel::Ruptured => "RUPTURED",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OringMaterial {
    Nbr,
    Hnbr,
    Fkm,
    Epdm,
    Silicone,
    Fluorosilicone,
    Aflas,
    Ffkm,
    Ptfe,
    Polyurethane,
    Acm,
    Cr,
    Vmq,
    Fvmq,
    Sbr,
    Ecor,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MaterialRuntimeProfile {
    pub label: &'static str,
    pub min_temp_c: f64,
    pub max_temp_c: f64,
    pub gas_permeability_factor: f64,
}

const ORING_MATERIAL_VARIANTS: [OringMaterial; 12] = [
    OringMaterial::Nbr,
    OringMaterial::Hnbr,
    OringMaterial::Fkm,
    OringMaterial::Epdm,
    OringMaterial::Silicone,
    OringMaterial::Fluorosilicone,
    OringMaterial::Aflas,
    OringMaterial::Ffkm,
    OringMaterial::Ptfe,
    OringMaterial::Polyurethane,
    OringMaterial::Acm,
    OringMaterial::Cr,
];

impl OringMaterial {
    pub fn all() -> &'static [OringMaterial] {
        &ORING_MATERIAL_VARIANTS
    }

    pub fn name(self) -> &'static str {
        self.runtime_profile().label
    }

    pub fn runtime_profile(self) -> MaterialRuntimeProfile {
        match self {
            OringMaterial::Nbr => MaterialRuntimeProfile { label: "NBR", min_temp_c: -35.0, max_temp_c: 110.0, gas_permeability_factor: 1.00 },
            OringMaterial::Hnbr => MaterialRuntimeProfile { label: "HNBR", min_temp_c: -40.0, max_temp_c: 150.0, gas_permeability_factor: 0.92 },
            OringMaterial::Fkm => MaterialRuntimeProfile { label: "FKM", min_temp_c: -20.0, max_temp_c: 200.0, gas_permeability_factor: 0.72 },
            OringMaterial::Epdm => MaterialRuntimeProfile { label: "EPDM", min_temp_c: -45.0, max_temp_c: 140.0, gas_permeability_factor: 1.10 },
            OringMaterial::Silicone => MaterialRuntimeProfile { label: "VMQ Silicone", min_temp_c: -60.0, max_temp_c: 210.0, gas_permeability_factor: 1.35 },
            OringMaterial::Fluorosilicone => MaterialRuntimeProfile { label: "FVMQ Fluorosilicone", min_temp_c: -58.0, max_temp_c: 190.0, gas_permeability_factor: 1.18 },
            OringMaterial::Aflas => MaterialRuntimeProfile { label: "AFLAS", min_temp_c: -10.0, max_temp_c: 230.0, gas_permeability_factor: 0.80 },
            OringMaterial::Ffkm => MaterialRuntimeProfile { label: "FFKM", min_temp_c: -15.0, max_temp_c: 320.0, gas_permeability_factor: 0.52 },
            OringMaterial::Ptfe => MaterialRuntimeProfile { label: "PTFE", min_temp_c: -80.0, max_temp_c: 260.0, gas_permeability_factor: 0.58 },
            OringMaterial::Polyurethane => MaterialRuntimeProfile { label: "PU", min_temp_c: -35.0, max_temp_c: 105.0, gas_permeability_factor: 0.88 },
            OringMaterial::Acm => MaterialRuntimeProfile { label: "ACM", min_temp_c: -25.0, max_temp_c: 165.0, gas_permeability_factor: 0.86 },
            OringMaterial::Cr => MaterialRuntimeProfile { label: "CR", min_temp_c: -40.0, max_temp_c: 115.0, gas_permeability_factor: 1.08 },
            OringMaterial::Vmq => MaterialRuntimeProfile { label: "VMQ", min_temp_c: -60.0, max_temp_c: 205.0, gas_permeability_factor: 1.35 },
            OringMaterial::Fvmq => MaterialRuntimeProfile { label: "FVMQ", min_temp_c: -58.0, max_temp_c: 190.0, gas_permeability_factor: 1.18 },
            OringMaterial::Sbr => MaterialRuntimeProfile { label: "SBR", min_temp_c: -40.0, max_temp_c: 100.0, gas_permeability_factor: 1.22 },
            OringMaterial::Ecor => MaterialRuntimeProfile { label: "ECO", min_temp_c: -35.0, max_temp_c: 140.0, gas_permeability_factor: 0.95 },
        }
    }

    pub fn temp_window_c(self) -> (f64, f64) {
        let p = self.runtime_profile();
        (p.min_temp_c, p.max_temp_c)
    }

    pub fn gas_permeability_factor(self) -> f64 {
        self.runtime_profile().gas_permeability_factor
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialCatalogEntry {
    pub id: &'static str,
    pub family: &'static str,
    pub designation: &'static str,
    pub hardness_shore: f64,
    pub tensile_strength_mpa: f64,
    pub elongation_pct: f64,
    pub compression_set_pct_70h: f64,
    pub tear_strength_kn_m: f64,
    pub density_g_cm3: f64,
    pub temp_min_c: f64,
    pub temp_max_c: f64,
    pub abrasion_index: f64,
    pub corrosion_resistance_index: f64,
    pub hydraulic_oil_compat: f64,
    pub engine_oil_compat: f64,
    pub refrigerant_oil_compat: f64,
    pub notes: &'static str,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OilCatalogEntry {
    pub id: &'static str,
    pub family: &'static str,
    pub grade: &'static str,
    pub base_stock: &'static str,
    pub density_15c_kg_m3: f64,
    pub viscosity_40c_cst: f64,
    pub viscosity_100c_cst: f64,
    pub viscosity_index: f64,
    pub pour_point_c: f64,
    pub flash_point_c: f64,
    pub tan_mg_koh_g: f64,
    pub water_content_ppm_limit: f64,
    pub demulsibility_min: f64,
    pub copper_corrosion_level: f64,
    pub oxidation_stability_h: f64,
    pub recommended_min_temp_c: f64,
    pub recommended_max_temp_c: f64,
    pub anti_wear_index: f64,
    pub corrosion_inhibition_index: f64,
    pub notes: &'static str,
}

pub fn engineering_material_catalog() -> Vec<MaterialCatalogEntry> {
    vec![
        MaterialCatalogEntry { id: "OR-NBR-70", family: "O-RING", designation: "NBR 70", hardness_shore: 70.0, tensile_strength_mpa: 12.0, elongation_pct: 280.0, compression_set_pct_70h: 18.0, tear_strength_kn_m: 28.0, density_g_cm3: 1.20, temp_min_c: -35.0, temp_max_c: 110.0, abrasion_index: 0.72, corrosion_resistance_index: 0.64, hydraulic_oil_compat: 0.92, engine_oil_compat: 0.86, refrigerant_oil_compat: 0.41, notes: "General hydraulic service" },
        MaterialCatalogEntry { id: "OR-NBR-90", family: "O-RING", designation: "NBR 90", hardness_shore: 90.0, tensile_strength_mpa: 14.0, elongation_pct: 220.0, compression_set_pct_70h: 16.0, tear_strength_kn_m: 34.0, density_g_cm3: 1.22, temp_min_c: -30.0, temp_max_c: 110.0, abrasion_index: 0.84, corrosion_resistance_index: 0.66, hydraulic_oil_compat: 0.95, engine_oil_compat: 0.88, refrigerant_oil_compat: 0.38, notes: "High-pressure extrusion resistance" },
        MaterialCatalogEntry { id: "OR-HNBR-80", family: "O-RING", designation: "HNBR 80", hardness_shore: 80.0, tensile_strength_mpa: 17.0, elongation_pct: 260.0, compression_set_pct_70h: 13.0, tear_strength_kn_m: 36.0, density_g_cm3: 1.18, temp_min_c: -40.0, temp_max_c: 150.0, abrasion_index: 0.82, corrosion_resistance_index: 0.71, hydraulic_oil_compat: 0.94, engine_oil_compat: 0.91, refrigerant_oil_compat: 0.65, notes: "Preferred for thermal cycling" },
        MaterialCatalogEntry { id: "OR-FKM-75", family: "O-RING", designation: "FKM 75", hardness_shore: 75.0, tensile_strength_mpa: 13.0, elongation_pct: 220.0, compression_set_pct_70h: 12.0, tear_strength_kn_m: 24.0, density_g_cm3: 1.85, temp_min_c: -20.0, temp_max_c: 200.0, abrasion_index: 0.63, corrosion_resistance_index: 0.88, hydraulic_oil_compat: 0.86, engine_oil_compat: 0.98, refrigerant_oil_compat: 0.89, notes: "Strong hydrocarbon and heat resistance" },
        MaterialCatalogEntry { id: "OR-EPDM-70", family: "O-RING", designation: "EPDM 70", hardness_shore: 70.0, tensile_strength_mpa: 11.0, elongation_pct: 320.0, compression_set_pct_70h: 20.0, tear_strength_kn_m: 26.0, density_g_cm3: 1.14, temp_min_c: -45.0, temp_max_c: 140.0, abrasion_index: 0.58, corrosion_resistance_index: 0.79, hydraulic_oil_compat: 0.30, engine_oil_compat: 0.25, refrigerant_oil_compat: 0.78, notes: "Steam/water dominant service" },
        MaterialCatalogEntry { id: "OR-AFLAS-80", family: "O-RING", designation: "AFLAS 80", hardness_shore: 80.0, tensile_strength_mpa: 14.0, elongation_pct: 200.0, compression_set_pct_70h: 15.0, tear_strength_kn_m: 27.0, density_g_cm3: 1.62, temp_min_c: -10.0, temp_max_c: 230.0, abrasion_index: 0.66, corrosion_resistance_index: 0.91, hydraulic_oil_compat: 0.88, engine_oil_compat: 0.94, refrigerant_oil_compat: 0.80, notes: "Amines/sour gas tolerant" },
        MaterialCatalogEntry { id: "OR-FFKM-75", family: "O-RING", designation: "FFKM 75", hardness_shore: 75.0, tensile_strength_mpa: 16.0, elongation_pct: 170.0, compression_set_pct_70h: 10.0, tear_strength_kn_m: 20.0, density_g_cm3: 1.95, temp_min_c: -15.0, temp_max_c: 320.0, abrasion_index: 0.55, corrosion_resistance_index: 0.97, hydraulic_oil_compat: 0.96, engine_oil_compat: 0.99, refrigerant_oil_compat: 0.94, notes: "Extreme high-temp chemical duty" },
        MaterialCatalogEntry { id: "OR-SIL-70", family: "O-RING", designation: "VMQ 70", hardness_shore: 70.0, tensile_strength_mpa: 9.0, elongation_pct: 350.0, compression_set_pct_70h: 24.0, tear_strength_kn_m: 18.0, density_g_cm3: 1.10, temp_min_c: -60.0, temp_max_c: 210.0, abrasion_index: 0.42, corrosion_resistance_index: 0.76, hydraulic_oil_compat: 0.48, engine_oil_compat: 0.38, refrigerant_oil_compat: 0.72, notes: "Low-temp flexibility priority" },
        MaterialCatalogEntry { id: "OR-FSIL-70", family: "O-RING", designation: "FVMQ 70", hardness_shore: 70.0, tensile_strength_mpa: 10.0, elongation_pct: 280.0, compression_set_pct_70h: 20.0, tear_strength_kn_m: 19.0, density_g_cm3: 1.34, temp_min_c: -58.0, temp_max_c: 190.0, abrasion_index: 0.45, corrosion_resistance_index: 0.80, hydraulic_oil_compat: 0.70, engine_oil_compat: 0.74, refrigerant_oil_compat: 0.88, notes: "Aviation fuel and oil compatibility" },
        MaterialCatalogEntry { id: "OR-PTFE", family: "O-RING", designation: "PTFE Virgin", hardness_shore: 60.0, tensile_strength_mpa: 25.0, elongation_pct: 220.0, compression_set_pct_70h: 6.0, tear_strength_kn_m: 14.0, density_g_cm3: 2.16, temp_min_c: -80.0, temp_max_c: 260.0, abrasion_index: 0.39, corrosion_resistance_index: 0.96, hydraulic_oil_compat: 0.98, engine_oil_compat: 0.98, refrigerant_oil_compat: 0.95, notes: "Low friction, needs backup ring" },
        MaterialCatalogEntry { id: "OR-PU-93", family: "O-RING", designation: "PU 93A", hardness_shore: 93.0, tensile_strength_mpa: 42.0, elongation_pct: 430.0, compression_set_pct_70h: 22.0, tear_strength_kn_m: 75.0, density_g_cm3: 1.21, temp_min_c: -35.0, temp_max_c: 105.0, abrasion_index: 0.95, corrosion_resistance_index: 0.65, hydraulic_oil_compat: 0.90, engine_oil_compat: 0.80, refrigerant_oil_compat: 0.52, notes: "Excellent abrasion and extrusion resistance" },
        MaterialCatalogEntry { id: "OR-ACM-70", family: "O-RING", designation: "ACM 70", hardness_shore: 70.0, tensile_strength_mpa: 11.5, elongation_pct: 210.0, compression_set_pct_70h: 14.0, tear_strength_kn_m: 21.0, density_g_cm3: 1.24, temp_min_c: -25.0, temp_max_c: 165.0, abrasion_index: 0.60, corrosion_resistance_index: 0.82, hydraulic_oil_compat: 0.81, engine_oil_compat: 0.93, refrigerant_oil_compat: 0.59, notes: "Hot engine oil applications" },
        MaterialCatalogEntry { id: "HS-NBR-TUBE", family: "HOSE_INNER_TUBE", designation: "NBR Tube", hardness_shore: 75.0, tensile_strength_mpa: 14.0, elongation_pct: 250.0, compression_set_pct_70h: 18.0, tear_strength_kn_m: 29.0, density_g_cm3: 1.18, temp_min_c: -30.0, temp_max_c: 120.0, abrasion_index: 0.74, corrosion_resistance_index: 0.68, hydraulic_oil_compat: 0.94, engine_oil_compat: 0.89, refrigerant_oil_compat: 0.40, notes: "General hydraulic hoses" },
        MaterialCatalogEntry { id: "HS-HNBR-TUBE", family: "HOSE_INNER_TUBE", designation: "HNBR Tube", hardness_shore: 78.0, tensile_strength_mpa: 17.0, elongation_pct: 260.0, compression_set_pct_70h: 14.0, tear_strength_kn_m: 34.0, density_g_cm3: 1.20, temp_min_c: -40.0, temp_max_c: 150.0, abrasion_index: 0.81, corrosion_resistance_index: 0.74, hydraulic_oil_compat: 0.95, engine_oil_compat: 0.92, refrigerant_oil_compat: 0.62, notes: "High-temp pressure hose" },
        MaterialCatalogEntry { id: "HS-CSM-COVER", family: "HOSE_COVER", designation: "CSM Cover", hardness_shore: 68.0, tensile_strength_mpa: 12.0, elongation_pct: 300.0, compression_set_pct_70h: 22.0, tear_strength_kn_m: 23.0, density_g_cm3: 1.30, temp_min_c: -25.0, temp_max_c: 125.0, abrasion_index: 0.70, corrosion_resistance_index: 0.77, hydraulic_oil_compat: 0.76, engine_oil_compat: 0.71, refrigerant_oil_compat: 0.63, notes: "Weather/ozone resistance" },
        MaterialCatalogEntry { id: "HS-PTFE-TUBE", family: "HOSE_INNER_TUBE", designation: "PTFE Hose Tube", hardness_shore: 62.0, tensile_strength_mpa: 24.0, elongation_pct: 220.0, compression_set_pct_70h: 7.0, tear_strength_kn_m: 16.0, density_g_cm3: 2.15, temp_min_c: -60.0, temp_max_c: 260.0, abrasion_index: 0.40, corrosion_resistance_index: 0.96, hydraulic_oil_compat: 0.99, engine_oil_compat: 0.99, refrigerant_oil_compat: 0.95, notes: "Chemical extreme hose assemblies" },
        MaterialCatalogEntry { id: "HS-ARAMID-BRAID", family: "HOSE_REINFORCEMENT", designation: "Aramid Braid", hardness_shore: 0.0, tensile_strength_mpa: 3000.0, elongation_pct: 3.0, compression_set_pct_70h: 0.0, tear_strength_kn_m: 0.0, density_g_cm3: 1.44, temp_min_c: -40.0, temp_max_c: 180.0, abrasion_index: 0.88, corrosion_resistance_index: 0.90, hydraulic_oil_compat: 0.95, engine_oil_compat: 0.95, refrigerant_oil_compat: 0.95, notes: "High burst-pressure reinforcement" },
        MaterialCatalogEntry { id: "HS-STEEL-WIRE", family: "HOSE_REINFORCEMENT", designation: "High-tensile Steel Wire", hardness_shore: 0.0, tensile_strength_mpa: 2500.0, elongation_pct: 2.0, compression_set_pct_70h: 0.0, tear_strength_kn_m: 0.0, density_g_cm3: 7.85, temp_min_c: -50.0, temp_max_c: 200.0, abrasion_index: 0.94, corrosion_resistance_index: 0.74, hydraulic_oil_compat: 0.98, engine_oil_compat: 0.98, refrigerant_oil_compat: 0.98, notes: "Spiral-wire hose reinforcement" },
        MaterialCatalogEntry { id: "SL-PTFE-LIP", family: "DYNAMIC_SEAL", designation: "PTFE Lip Seal", hardness_shore: 63.0, tensile_strength_mpa: 22.0, elongation_pct: 210.0, compression_set_pct_70h: 8.0, tear_strength_kn_m: 15.0, density_g_cm3: 2.12, temp_min_c: -70.0, temp_max_c: 250.0, abrasion_index: 0.44, corrosion_resistance_index: 0.95, hydraulic_oil_compat: 0.99, engine_oil_compat: 0.99, refrigerant_oil_compat: 0.92, notes: "Low-friction dynamic shaft sealing" },
        MaterialCatalogEntry { id: "SL-PU-LIP", family: "DYNAMIC_SEAL", designation: "PU Lip Seal", hardness_shore: 92.0, tensile_strength_mpa: 40.0, elongation_pct: 410.0, compression_set_pct_70h: 20.0, tear_strength_kn_m: 72.0, density_g_cm3: 1.20, temp_min_c: -35.0, temp_max_c: 110.0, abrasion_index: 0.96, corrosion_resistance_index: 0.66, hydraulic_oil_compat: 0.92, engine_oil_compat: 0.84, refrigerant_oil_compat: 0.50, notes: "Heavy-duty rod/piston seals" },
        MaterialCatalogEntry { id: "SL-NBR-LIP", family: "DYNAMIC_SEAL", designation: "NBR Lip", hardness_shore: 75.0, tensile_strength_mpa: 13.0, elongation_pct: 260.0, compression_set_pct_70h: 17.0, tear_strength_kn_m: 29.0, density_g_cm3: 1.19, temp_min_c: -35.0, temp_max_c: 120.0, abrasion_index: 0.76, corrosion_resistance_index: 0.66, hydraulic_oil_compat: 0.93, engine_oil_compat: 0.87, refrigerant_oil_compat: 0.42, notes: "Cost-effective rotating shaft seal" },
        MaterialCatalogEntry { id: "BK-PTFE-GLASS", family: "BACKUP_RING", designation: "PTFE 25% Glass", hardness_shore: 64.0, tensile_strength_mpa: 28.0, elongation_pct: 120.0, compression_set_pct_70h: 6.0, tear_strength_kn_m: 12.0, density_g_cm3: 2.22, temp_min_c: -70.0, temp_max_c: 260.0, abrasion_index: 0.52, corrosion_resistance_index: 0.96, hydraulic_oil_compat: 0.99, engine_oil_compat: 0.99, refrigerant_oil_compat: 0.94, notes: "Anti-extrusion backup rings" },
        MaterialCatalogEntry { id: "BK-PA66", family: "BACKUP_RING", designation: "PA66", hardness_shore: 80.0, tensile_strength_mpa: 78.0, elongation_pct: 60.0, compression_set_pct_70h: 10.0, tear_strength_kn_m: 0.0, density_g_cm3: 1.14, temp_min_c: -30.0, temp_max_c: 120.0, abrasion_index: 0.78, corrosion_resistance_index: 0.72, hydraulic_oil_compat: 0.90, engine_oil_compat: 0.80, refrigerant_oil_compat: 0.57, notes: "Split backup rings for hydraulics" },
        MaterialCatalogEntry { id: "BK-PEEK", family: "BACKUP_RING", designation: "PEEK", hardness_shore: 85.0, tensile_strength_mpa: 95.0, elongation_pct: 45.0, compression_set_pct_70h: 5.0, tear_strength_kn_m: 0.0, density_g_cm3: 1.32, temp_min_c: -50.0, temp_max_c: 250.0, abrasion_index: 0.84, corrosion_resistance_index: 0.94, hydraulic_oil_compat: 0.98, engine_oil_compat: 0.97, refrigerant_oil_compat: 0.89, notes: "High-pressure/high-temp anti-extrusion" },
        MaterialCatalogEntry { id: "MS-CARBON", family: "MECH_SEAL_FACE", designation: "Carbon Graphite", hardness_shore: 0.0, tensile_strength_mpa: 70.0, elongation_pct: 0.5, compression_set_pct_70h: 0.0, tear_strength_kn_m: 0.0, density_g_cm3: 1.75, temp_min_c: -50.0, temp_max_c: 260.0, abrasion_index: 0.66, corrosion_resistance_index: 0.84, hydraulic_oil_compat: 0.97, engine_oil_compat: 0.95, refrigerant_oil_compat: 0.88, notes: "Mechanical seal rotating face" },
        MaterialCatalogEntry { id: "MS-SIC", family: "MECH_SEAL_FACE", designation: "Silicon Carbide", hardness_shore: 0.0, tensile_strength_mpa: 380.0, elongation_pct: 0.1, compression_set_pct_70h: 0.0, tear_strength_kn_m: 0.0, density_g_cm3: 3.10, temp_min_c: -80.0, temp_max_c: 300.0, abrasion_index: 0.98, corrosion_resistance_index: 0.93, hydraulic_oil_compat: 0.99, engine_oil_compat: 0.99, refrigerant_oil_compat: 0.96, notes: "Extreme wear mechanical seal face" },
        MaterialCatalogEntry { id: "MS-TC", family: "MECH_SEAL_FACE", designation: "Tungsten Carbide", hardness_shore: 0.0, tensile_strength_mpa: 500.0, elongation_pct: 0.1, compression_set_pct_70h: 0.0, tear_strength_kn_m: 0.0, density_g_cm3: 14.5, temp_min_c: -80.0, temp_max_c: 400.0, abrasion_index: 0.99, corrosion_resistance_index: 0.89, hydraulic_oil_compat: 0.99, engine_oil_compat: 0.99, refrigerant_oil_compat: 0.95, notes: "High-load slurry-capable face" },
        MaterialCatalogEntry { id: "WR-POM", family: "WEAR_RING", designation: "POM Wear Ring", hardness_shore: 83.0, tensile_strength_mpa: 65.0, elongation_pct: 20.0, compression_set_pct_70h: 9.0, tear_strength_kn_m: 0.0, density_g_cm3: 1.42, temp_min_c: -40.0, temp_max_c: 110.0, abrasion_index: 0.80, corrosion_resistance_index: 0.70, hydraulic_oil_compat: 0.92, engine_oil_compat: 0.84, refrigerant_oil_compat: 0.58, notes: "Hydraulic cylinder guidance" },
        MaterialCatalogEntry { id: "WR-PTFE-BRONZE", family: "WEAR_RING", designation: "PTFE Bronze Filled", hardness_shore: 65.0, tensile_strength_mpa: 26.0, elongation_pct: 130.0, compression_set_pct_70h: 7.0, tear_strength_kn_m: 0.0, density_g_cm3: 2.35, temp_min_c: -70.0, temp_max_c: 250.0, abrasion_index: 0.72, corrosion_resistance_index: 0.93, hydraulic_oil_compat: 0.99, engine_oil_compat: 0.98, refrigerant_oil_compat: 0.90, notes: "Low friction wear ring" },
        MaterialCatalogEntry { id: "WR-PHENOLIC", family: "WEAR_RING", designation: "Fabric Phenolic", hardness_shore: 88.0, tensile_strength_mpa: 120.0, elongation_pct: 2.0, compression_set_pct_70h: 8.0, tear_strength_kn_m: 0.0, density_g_cm3: 1.35, temp_min_c: -40.0, temp_max_c: 135.0, abrasion_index: 0.86, corrosion_resistance_index: 0.75, hydraulic_oil_compat: 0.94, engine_oil_compat: 0.90, refrigerant_oil_compat: 0.62, notes: "High load bearing guidance" },
        MaterialCatalogEntry { id: "OR-ECO-80", family: "O-RING", designation: "ECO 80", hardness_shore: 80.0, tensile_strength_mpa: 12.5, elongation_pct: 240.0, compression_set_pct_70h: 16.0, tear_strength_kn_m: 24.0, density_g_cm3: 1.28, temp_min_c: -35.0, temp_max_c: 140.0, abrasion_index: 0.68, corrosion_resistance_index: 0.78, hydraulic_oil_compat: 0.88, engine_oil_compat: 0.90, refrigerant_oil_compat: 0.66, notes: "Fuel/oil resistant elastomer" },
        MaterialCatalogEntry { id: "OR-CR-70", family: "O-RING", designation: "CR 70", hardness_shore: 70.0, tensile_strength_mpa: 11.0, elongation_pct: 300.0, compression_set_pct_70h: 21.0, tear_strength_kn_m: 22.0, density_g_cm3: 1.24, temp_min_c: -40.0, temp_max_c: 115.0, abrasion_index: 0.60, corrosion_resistance_index: 0.73, hydraulic_oil_compat: 0.76, engine_oil_compat: 0.72, refrigerant_oil_compat: 0.70, notes: "Weather and moderate oil service" },
        MaterialCatalogEntry { id: "OR-VMQ-50", family: "O-RING", designation: "VMQ 50", hardness_shore: 50.0, tensile_strength_mpa: 8.0, elongation_pct: 420.0, compression_set_pct_70h: 28.0, tear_strength_kn_m: 14.0, density_g_cm3: 1.09, temp_min_c: -65.0, temp_max_c: 200.0, abrasion_index: 0.36, corrosion_resistance_index: 0.74, hydraulic_oil_compat: 0.44, engine_oil_compat: 0.36, refrigerant_oil_compat: 0.70, notes: "High flexibility, low pressure sealing" },
    ]
}

pub fn engineering_oil_catalog() -> Vec<OilCatalogEntry> {
    vec![
        OilCatalogEntry { id: "HYD-ISO-22", family: "HYDRAULIC", grade: "ISO VG 22", base_stock: "Mineral", density_15c_kg_m3: 852.0, viscosity_40c_cst: 22.0, viscosity_100c_cst: 4.5, viscosity_index: 108.0, pour_point_c: -30.0, flash_point_c: 205.0, tan_mg_koh_g: 0.7, water_content_ppm_limit: 300.0, demulsibility_min: 20.0, copper_corrosion_level: 1.0, oxidation_stability_h: 1800.0, recommended_min_temp_c: -20.0, recommended_max_temp_c: 85.0, anti_wear_index: 0.78, corrosion_inhibition_index: 0.74, notes: "Fine servo hydraulic systems" },
        OilCatalogEntry { id: "HYD-ISO-32", family: "HYDRAULIC", grade: "ISO VG 32", base_stock: "Mineral", density_15c_kg_m3: 858.0, viscosity_40c_cst: 32.0, viscosity_100c_cst: 5.4, viscosity_index: 106.0, pour_point_c: -27.0, flash_point_c: 215.0, tan_mg_koh_g: 0.7, water_content_ppm_limit: 300.0, demulsibility_min: 20.0, copper_corrosion_level: 1.0, oxidation_stability_h: 1900.0, recommended_min_temp_c: -18.0, recommended_max_temp_c: 90.0, anti_wear_index: 0.80, corrosion_inhibition_index: 0.76, notes: "General mobile hydraulics" },
        OilCatalogEntry { id: "HYD-ISO-46", family: "HYDRAULIC", grade: "ISO VG 46", base_stock: "Mineral", density_15c_kg_m3: 865.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 6.8, viscosity_index: 105.0, pour_point_c: -24.0, flash_point_c: 225.0, tan_mg_koh_g: 0.8, water_content_ppm_limit: 300.0, demulsibility_min: 25.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2200.0, recommended_min_temp_c: -15.0, recommended_max_temp_c: 95.0, anti_wear_index: 0.82, corrosion_inhibition_index: 0.78, notes: "Standard heavy equipment hydraulic oil" },
        OilCatalogEntry { id: "HYD-ISO-68", family: "HYDRAULIC", grade: "ISO VG 68", base_stock: "Mineral", density_15c_kg_m3: 875.0, viscosity_40c_cst: 68.0, viscosity_100c_cst: 8.7, viscosity_index: 102.0, pour_point_c: -21.0, flash_point_c: 235.0, tan_mg_koh_g: 0.8, water_content_ppm_limit: 300.0, demulsibility_min: 25.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2300.0, recommended_min_temp_c: -10.0, recommended_max_temp_c: 100.0, anti_wear_index: 0.83, corrosion_inhibition_index: 0.79, notes: "High-load hydraulic circuits" },
        OilCatalogEntry { id: "HYD-ISO-100", family: "HYDRAULIC", grade: "ISO VG 100", base_stock: "Mineral", density_15c_kg_m3: 882.0, viscosity_40c_cst: 100.0, viscosity_100c_cst: 11.0, viscosity_index: 100.0, pour_point_c: -15.0, flash_point_c: 240.0, tan_mg_koh_g: 0.9, water_content_ppm_limit: 300.0, demulsibility_min: 30.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2400.0, recommended_min_temp_c: -8.0, recommended_max_temp_c: 105.0, anti_wear_index: 0.84, corrosion_inhibition_index: 0.80, notes: "Slow-speed high pressure actuators" },
        OilCatalogEntry { id: "AW-46", family: "HYDRAULIC_AW", grade: "AW 46", base_stock: "Mineral", density_15c_kg_m3: 868.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 6.9, viscosity_index: 108.0, pour_point_c: -27.0, flash_point_c: 228.0, tan_mg_koh_g: 0.75, water_content_ppm_limit: 250.0, demulsibility_min: 20.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2600.0, recommended_min_temp_c: -18.0, recommended_max_temp_c: 100.0, anti_wear_index: 0.90, corrosion_inhibition_index: 0.82, notes: "ZDDP anti-wear package" },
        OilCatalogEntry { id: "HEES-46", family: "BIODEGRADABLE", grade: "HEES 46", base_stock: "Synthetic Ester", density_15c_kg_m3: 920.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 9.3, viscosity_index: 190.0, pour_point_c: -45.0, flash_point_c: 300.0, tan_mg_koh_g: 1.2, water_content_ppm_limit: 150.0, demulsibility_min: 35.0, copper_corrosion_level: 1.0, oxidation_stability_h: 1500.0, recommended_min_temp_c: -35.0, recommended_max_temp_c: 90.0, anti_wear_index: 0.88, corrosion_inhibition_index: 0.84, notes: "Environmentally sensitive sites" },
        OilCatalogEntry { id: "HEPR-46", family: "BIODEGRADABLE", grade: "HEPR 46", base_stock: "PAO", density_15c_kg_m3: 900.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 8.6, viscosity_index: 165.0, pour_point_c: -48.0, flash_point_c: 260.0, tan_mg_koh_g: 0.8, water_content_ppm_limit: 200.0, demulsibility_min: 30.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2200.0, recommended_min_temp_c: -30.0, recommended_max_temp_c: 105.0, anti_wear_index: 0.86, corrosion_inhibition_index: 0.83, notes: "Longer life bio-hydraulic oil" },
        OilCatalogEntry { id: "HFDU-46", family: "FIRE_RESISTANT", grade: "HFDU 46", base_stock: "Synthetic Ester", density_15c_kg_m3: 930.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 8.5, viscosity_index: 150.0, pour_point_c: -27.0, flash_point_c: 320.0, tan_mg_koh_g: 1.5, water_content_ppm_limit: 150.0, demulsibility_min: 40.0, copper_corrosion_level: 1.0, oxidation_stability_h: 1200.0, recommended_min_temp_c: -20.0, recommended_max_temp_c: 160.0, anti_wear_index: 0.84, corrosion_inhibition_index: 0.88, notes: "Steel plant/high fire risk" },
        OilCatalogEntry { id: "ENG-0W20", family: "ENGINE", grade: "0W-20", base_stock: "Group III/PAO", density_15c_kg_m3: 845.0, viscosity_40c_cst: 43.0, viscosity_100c_cst: 8.5, viscosity_index: 170.0, pour_point_c: -45.0, flash_point_c: 228.0, tan_mg_koh_g: 1.5, water_content_ppm_limit: 200.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 400.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 125.0, anti_wear_index: 0.82, corrosion_inhibition_index: 0.78, notes: "Fuel economy modern engines" },
        OilCatalogEntry { id: "ENG-5W30", family: "ENGINE", grade: "5W-30", base_stock: "Group II/III", density_15c_kg_m3: 855.0, viscosity_40c_cst: 61.0, viscosity_100c_cst: 10.5, viscosity_index: 165.0, pour_point_c: -39.0, flash_point_c: 232.0, tan_mg_koh_g: 1.6, water_content_ppm_limit: 200.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 380.0, recommended_min_temp_c: -35.0, recommended_max_temp_c: 130.0, anti_wear_index: 0.84, corrosion_inhibition_index: 0.79, notes: "Broad fleet use" },
        OilCatalogEntry { id: "ENG-10W30", family: "ENGINE", grade: "10W-30", base_stock: "Group II", density_15c_kg_m3: 870.0, viscosity_40c_cst: 72.0, viscosity_100c_cst: 11.5, viscosity_index: 150.0, pour_point_c: -33.0, flash_point_c: 235.0, tan_mg_koh_g: 1.7, water_content_ppm_limit: 250.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 350.0, recommended_min_temp_c: -30.0, recommended_max_temp_c: 130.0, anti_wear_index: 0.86, corrosion_inhibition_index: 0.80, notes: "Standard diesel engine oil" },
        OilCatalogEntry { id: "ENG-15W40", family: "ENGINE", grade: "15W-40", base_stock: "Group II", density_15c_kg_m3: 880.0, viscosity_40c_cst: 108.0, viscosity_100c_cst: 14.5, viscosity_index: 140.0, pour_point_c: -27.0, flash_point_c: 238.0, tan_mg_koh_g: 1.8, water_content_ppm_limit: 250.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 330.0, recommended_min_temp_c: -25.0, recommended_max_temp_c: 135.0, anti_wear_index: 0.88, corrosion_inhibition_index: 0.82, notes: "Heavy duty diesel baseline" },
        OilCatalogEntry { id: "ENG-20W50", family: "ENGINE", grade: "20W-50", base_stock: "Group II", density_15c_kg_m3: 892.0, viscosity_40c_cst: 155.0, viscosity_100c_cst: 18.0, viscosity_index: 132.0, pour_point_c: -21.0, flash_point_c: 245.0, tan_mg_koh_g: 1.9, water_content_ppm_limit: 250.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 320.0, recommended_min_temp_c: -15.0, recommended_max_temp_c: 140.0, anti_wear_index: 0.90, corrosion_inhibition_index: 0.82, notes: "High temp/worn engine support" },
        OilCatalogEntry { id: "ATF-DIII", family: "TRANSMISSION", grade: "Dexron III", base_stock: "Mineral/Synthetic", density_15c_kg_m3: 850.0, viscosity_40c_cst: 35.0, viscosity_100c_cst: 7.2, viscosity_index: 180.0, pour_point_c: -48.0, flash_point_c: 210.0, tan_mg_koh_g: 1.0, water_content_ppm_limit: 200.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 500.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 130.0, anti_wear_index: 0.80, corrosion_inhibition_index: 0.78, notes: "Legacy automatic transmissions" },
        OilCatalogEntry { id: "ATF-DVI", family: "TRANSMISSION", grade: "Dexron VI", base_stock: "Synthetic", density_15c_kg_m3: 846.0, viscosity_40c_cst: 30.0, viscosity_100c_cst: 6.0, viscosity_index: 165.0, pour_point_c: -51.0, flash_point_c: 220.0, tan_mg_koh_g: 0.9, water_content_ppm_limit: 200.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 600.0, recommended_min_temp_c: -45.0, recommended_max_temp_c: 135.0, anti_wear_index: 0.82, corrosion_inhibition_index: 0.80, notes: "Lower viscosity high-efficiency ATF" },
        OilCatalogEntry { id: "GEAR-75W90", family: "GEAR", grade: "75W-90", base_stock: "PAO", density_15c_kg_m3: 875.0, viscosity_40c_cst: 95.0, viscosity_100c_cst: 15.0, viscosity_index: 155.0, pour_point_c: -45.0, flash_point_c: 210.0, tan_mg_koh_g: 1.4, water_content_ppm_limit: 250.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 450.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 140.0, anti_wear_index: 0.93, corrosion_inhibition_index: 0.84, notes: "Final drive and gearboxes" },
        OilCatalogEntry { id: "GEAR-80W90", family: "GEAR", grade: "80W-90", base_stock: "Mineral", density_15c_kg_m3: 890.0, viscosity_40c_cst: 135.0, viscosity_100c_cst: 14.5, viscosity_index: 135.0, pour_point_c: -30.0, flash_point_c: 215.0, tan_mg_koh_g: 1.5, water_content_ppm_limit: 250.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 380.0, recommended_min_temp_c: -26.0, recommended_max_temp_c: 140.0, anti_wear_index: 0.94, corrosion_inhibition_index: 0.84, notes: "Heavy axle service" },
        OilCatalogEntry { id: "PAG-46", family: "REFRIGERATION", grade: "PAG 46", base_stock: "Polyalkylene Glycol", density_15c_kg_m3: 995.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 8.3, viscosity_index: 130.0, pour_point_c: -48.0, flash_point_c: 210.0, tan_mg_koh_g: 0.05, water_content_ppm_limit: 50.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 700.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 170.0, anti_wear_index: 0.62, corrosion_inhibition_index: 0.70, notes: "R134a compressor lubricant" },
        OilCatalogEntry { id: "PAG-100", family: "REFRIGERATION", grade: "PAG 100", base_stock: "Polyalkylene Glycol", density_15c_kg_m3: 1002.0, viscosity_40c_cst: 100.0, viscosity_100c_cst: 14.0, viscosity_index: 135.0, pour_point_c: -45.0, flash_point_c: 220.0, tan_mg_koh_g: 0.06, water_content_ppm_limit: 50.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 760.0, recommended_min_temp_c: -35.0, recommended_max_temp_c: 175.0, anti_wear_index: 0.64, corrosion_inhibition_index: 0.72, notes: "Higher viscosity compressor oil" },
        OilCatalogEntry { id: "PAG-150", family: "REFRIGERATION", grade: "PAG 150", base_stock: "Polyalkylene Glycol", density_15c_kg_m3: 1010.0, viscosity_40c_cst: 150.0, viscosity_100c_cst: 18.0, viscosity_index: 140.0, pour_point_c: -40.0, flash_point_c: 225.0, tan_mg_koh_g: 0.07, water_content_ppm_limit: 50.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 780.0, recommended_min_temp_c: -30.0, recommended_max_temp_c: 180.0, anti_wear_index: 0.66, corrosion_inhibition_index: 0.73, notes: "High load compressor application" },
        OilCatalogEntry { id: "POE-32", family: "REFRIGERATION", grade: "POE 32", base_stock: "Polyol Ester", density_15c_kg_m3: 970.0, viscosity_40c_cst: 32.0, viscosity_100c_cst: 5.9, viscosity_index: 140.0, pour_point_c: -54.0, flash_point_c: 245.0, tan_mg_koh_g: 0.05, water_content_ppm_limit: 35.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 900.0, recommended_min_temp_c: -45.0, recommended_max_temp_c: 170.0, anti_wear_index: 0.68, corrosion_inhibition_index: 0.76, notes: "HFC/POE refrigeration systems" },
        OilCatalogEntry { id: "POE-46", family: "REFRIGERATION", grade: "POE 46", base_stock: "Polyol Ester", density_15c_kg_m3: 975.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 7.4, viscosity_index: 143.0, pour_point_c: -50.0, flash_point_c: 250.0, tan_mg_koh_g: 0.05, water_content_ppm_limit: 35.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 950.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 175.0, anti_wear_index: 0.69, corrosion_inhibition_index: 0.77, notes: "Common automotive POE" },
        OilCatalogEntry { id: "POE-68", family: "REFRIGERATION", grade: "POE 68", base_stock: "Polyol Ester", density_15c_kg_m3: 980.0, viscosity_40c_cst: 68.0, viscosity_100c_cst: 9.2, viscosity_index: 145.0, pour_point_c: -45.0, flash_point_c: 255.0, tan_mg_koh_g: 0.06, water_content_ppm_limit: 35.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 980.0, recommended_min_temp_c: -35.0, recommended_max_temp_c: 180.0, anti_wear_index: 0.70, corrosion_inhibition_index: 0.78, notes: "High-temp compressor operation" },
        OilCatalogEntry { id: "POE-100", family: "REFRIGERATION", grade: "POE 100", base_stock: "Polyol Ester", density_15c_kg_m3: 988.0, viscosity_40c_cst: 100.0, viscosity_100c_cst: 11.5, viscosity_index: 148.0, pour_point_c: -42.0, flash_point_c: 260.0, tan_mg_koh_g: 0.06, water_content_ppm_limit: 35.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 1000.0, recommended_min_temp_c: -30.0, recommended_max_temp_c: 185.0, anti_wear_index: 0.71, corrosion_inhibition_index: 0.78, notes: "Very high compressor loading" },
        OilCatalogEntry { id: "TRF-INH", family: "ELECTRICAL", grade: "Transformer Inhibited", base_stock: "Mineral", density_15c_kg_m3: 885.0, viscosity_40c_cst: 9.0, viscosity_100c_cst: 2.4, viscosity_index: 95.0, pour_point_c: -45.0, flash_point_c: 150.0, tan_mg_koh_g: 0.03, water_content_ppm_limit: 20.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 164.0, recommended_min_temp_c: -30.0, recommended_max_temp_c: 110.0, anti_wear_index: 0.20, corrosion_inhibition_index: 0.88, notes: "Insulating fluid, not hydraulic" },
        OilCatalogEntry { id: "TURB-32", family: "TURBINE", grade: "ISO 32", base_stock: "Mineral", density_15c_kg_m3: 855.0, viscosity_40c_cst: 32.0, viscosity_100c_cst: 5.3, viscosity_index: 102.0, pour_point_c: -15.0, flash_point_c: 220.0, tan_mg_koh_g: 0.2, water_content_ppm_limit: 200.0, demulsibility_min: 15.0, copper_corrosion_level: 1.0, oxidation_stability_h: 4000.0, recommended_min_temp_c: -10.0, recommended_max_temp_c: 120.0, anti_wear_index: 0.50, corrosion_inhibition_index: 0.86, notes: "Long-life oxidation resistance" },
        OilCatalogEntry { id: "TURB-46", family: "TURBINE", grade: "ISO 46", base_stock: "Mineral", density_15c_kg_m3: 865.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 6.8, viscosity_index: 100.0, pour_point_c: -12.0, flash_point_c: 225.0, tan_mg_koh_g: 0.2, water_content_ppm_limit: 200.0, demulsibility_min: 15.0, copper_corrosion_level: 1.0, oxidation_stability_h: 4200.0, recommended_min_temp_c: -8.0, recommended_max_temp_c: 125.0, anti_wear_index: 0.52, corrosion_inhibition_index: 0.86, notes: "Steam/gas turbine lubrication" },
        OilCatalogEntry { id: "COMP-46", family: "COMPRESSOR", grade: "ISO 46", base_stock: "Mineral", density_15c_kg_m3: 862.0, viscosity_40c_cst: 46.0, viscosity_100c_cst: 7.2, viscosity_index: 108.0, pour_point_c: -30.0, flash_point_c: 230.0, tan_mg_koh_g: 0.5, water_content_ppm_limit: 250.0, demulsibility_min: 25.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2500.0, recommended_min_temp_c: -15.0, recommended_max_temp_c: 150.0, anti_wear_index: 0.74, corrosion_inhibition_index: 0.80, notes: "Rotary screw compressor" },
        OilCatalogEntry { id: "COMP-68", family: "COMPRESSOR", grade: "ISO 68", base_stock: "Mineral", density_15c_kg_m3: 872.0, viscosity_40c_cst: 68.0, viscosity_100c_cst: 9.2, viscosity_index: 105.0, pour_point_c: -24.0, flash_point_c: 235.0, tan_mg_koh_g: 0.5, water_content_ppm_limit: 250.0, demulsibility_min: 25.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2600.0, recommended_min_temp_c: -10.0, recommended_max_temp_c: 155.0, anti_wear_index: 0.76, corrosion_inhibition_index: 0.81, notes: "High discharge temperature compressors" },
        OilCatalogEntry { id: "DOT3", family: "BRAKE", grade: "DOT 3", base_stock: "Glycol Ether", density_15c_kg_m3: 1035.0, viscosity_40c_cst: 5.0, viscosity_100c_cst: 2.0, viscosity_index: 220.0, pour_point_c: -60.0, flash_point_c: 120.0, tan_mg_koh_g: 1.0, water_content_ppm_limit: 500.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 0.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 205.0, anti_wear_index: 0.35, corrosion_inhibition_index: 0.75, notes: "Dry boiling around 205C" },
        OilCatalogEntry { id: "DOT4", family: "BRAKE", grade: "DOT 4", base_stock: "Borate Ester", density_15c_kg_m3: 1060.0, viscosity_40c_cst: 6.0, viscosity_100c_cst: 2.2, viscosity_index: 230.0, pour_point_c: -60.0, flash_point_c: 150.0, tan_mg_koh_g: 1.0, water_content_ppm_limit: 500.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 0.0, recommended_min_temp_c: -45.0, recommended_max_temp_c: 230.0, anti_wear_index: 0.37, corrosion_inhibition_index: 0.78, notes: "Higher boiling than DOT3" },
        OilCatalogEntry { id: "DOT5.1", family: "BRAKE", grade: "DOT 5.1", base_stock: "Borate Ester", density_15c_kg_m3: 1045.0, viscosity_40c_cst: 6.2, viscosity_100c_cst: 2.3, viscosity_index: 240.0, pour_point_c: -65.0, flash_point_c: 155.0, tan_mg_koh_g: 1.0, water_content_ppm_limit: 500.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 0.0, recommended_min_temp_c: -50.0, recommended_max_temp_c: 240.0, anti_wear_index: 0.38, corrosion_inhibition_index: 0.79, notes: "Low-temp ABS response" },
        OilCatalogEntry { id: "UTTO-10W30", family: "TRACTOR", grade: "UTTO 10W-30", base_stock: "Mineral", density_15c_kg_m3: 872.0, viscosity_40c_cst: 64.0, viscosity_100c_cst: 10.5, viscosity_index: 142.0, pour_point_c: -39.0, flash_point_c: 220.0, tan_mg_koh_g: 1.2, water_content_ppm_limit: 300.0, demulsibility_min: 20.0, copper_corrosion_level: 1.0, oxidation_stability_h: 900.0, recommended_min_temp_c: -30.0, recommended_max_temp_c: 115.0, anti_wear_index: 0.85, corrosion_inhibition_index: 0.80, notes: "Combined hydraulic/transmission tractors" },
        OilCatalogEntry { id: "UTTO-80W", family: "TRACTOR", grade: "UTTO 80W", base_stock: "Mineral", density_15c_kg_m3: 885.0, viscosity_40c_cst: 90.0, viscosity_100c_cst: 12.0, viscosity_index: 128.0, pour_point_c: -30.0, flash_point_c: 225.0, tan_mg_koh_g: 1.3, water_content_ppm_limit: 300.0, demulsibility_min: 20.0, copper_corrosion_level: 1.0, oxidation_stability_h: 850.0, recommended_min_temp_c: -20.0, recommended_max_temp_c: 120.0, anti_wear_index: 0.86, corrosion_inhibition_index: 0.80, notes: "Older tractor drivetrain systems" },
        OilCatalogEntry { id: "VAC-100", family: "VACUUM", grade: "ISO 100", base_stock: "Mineral", density_15c_kg_m3: 885.0, viscosity_40c_cst: 100.0, viscosity_100c_cst: 11.2, viscosity_index: 98.0, pour_point_c: -18.0, flash_point_c: 240.0, tan_mg_koh_g: 0.4, water_content_ppm_limit: 200.0, demulsibility_min: 30.0, copper_corrosion_level: 1.0, oxidation_stability_h: 1400.0, recommended_min_temp_c: -10.0, recommended_max_temp_c: 135.0, anti_wear_index: 0.60, corrosion_inhibition_index: 0.82, notes: "Rotary vane vacuum pumps" },
        OilCatalogEntry { id: "ESTER-100", family: "SYNTHETIC", grade: "ISO 100", base_stock: "Synthetic Ester", density_15c_kg_m3: 915.0, viscosity_40c_cst: 100.0, viscosity_100c_cst: 14.5, viscosity_index: 175.0, pour_point_c: -45.0, flash_point_c: 285.0, tan_mg_koh_g: 0.9, water_content_ppm_limit: 150.0, demulsibility_min: 35.0, copper_corrosion_level: 1.0, oxidation_stability_h: 2000.0, recommended_min_temp_c: -35.0, recommended_max_temp_c: 180.0, anti_wear_index: 0.88, corrosion_inhibition_index: 0.87, notes: "High temp synthetic chain lubrication" },
        OilCatalogEntry { id: "REF-MIN-3GS", family: "REFRIGERATION", grade: "3GS", base_stock: "Mineral Naphthenic", density_15c_kg_m3: 895.0, viscosity_40c_cst: 32.0, viscosity_100c_cst: 4.8, viscosity_index: 85.0, pour_point_c: -42.0, flash_point_c: 180.0, tan_mg_koh_g: 0.05, water_content_ppm_limit: 50.0, demulsibility_min: 0.0, copper_corrosion_level: 1.0, oxidation_stability_h: 600.0, recommended_min_temp_c: -40.0, recommended_max_temp_c: 80.0, anti_wear_index: 0.50, corrosion_inhibition_index: 0.68, notes: "Legacy ammonia/CFC systems" },
    ]
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PressureEnvelope {
    pub min_bar: f64,
    pub mean_bar: f64,
    pub ideal_bar: f64,
    pub max_bar: f64,
    pub rupture_bar: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OringSpec {
    pub id_tag: String,
    pub material: OringMaterial,
    pub shore_a: f64,
    pub cross_section_mm: f64,
    pub squeeze_pct: f64,
    pub extrusion_gap_mm: f64,
    pub compression_set_pct: f64,
    pub design_life_hours: f64,
    pub base_leak_area_mm2: f64,
    pub discharge_coeff: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CircuitInput {
    pub pressure_bar: f64,
    pub delta_p_bar: f64,
    pub temp_c: f64,
    pub cycles_per_s: f64,
    pub duty_01: f64,
    pub fluid_density_kg_m3: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CircuitResult {
    pub name: String,
    pub alert: LeakAlertLevel,
    pub pressure_band: PressureBand,
    pub current_pressure_bar: f64,
    pub rupture_probability_pct: f64,
    pub predicted_hours_to_rupture: Option<f64>,
    pub rupture_pressure_bar: Option<f64>,
    pub rupture_elapsed_h: Option<f64>,
    pub leak_area_mm2: f64,
    pub leak_lpm: f64,
    pub pressure_min_bar: f64,
    pub pressure_mean_bar: f64,
    pub pressure_max_bar: f64,
    pub recommended_hold_min_bar: f64,
    pub recommended_hold_target_bar: f64,
    pub recommended_hold_max_bar: f64,
    pub margin_to_max_bar: f64,
    pub margin_to_rupture_bar: f64,
    pub rca_hint: String,
    pub pca_recommendation: String,
    pub rupture_confirmed: bool,
    pub warning_text: String,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ManualCircuitParams {
    pub oil_type: Option<OilType>,
    pub piston_pressure_bar: Option<f64>,
    pub operation_pressure_bar: Option<f64>,
    pub pressure_min_bar: Option<f64>,
    pub pressure_mean_bar: Option<f64>,
    pub pressure_ideal_bar: Option<f64>,
    pub pressure_max_bar: Option<f64>,
    pub pressure_rupture_bar: Option<f64>,
    pub oring_squeeze_pct: Option<f64>,
    pub compression_set_pct: Option<f64>,
    pub base_leak_area_mm2: Option<f64>,
    pub max_supported_temp_c: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioPrediction {
    pub circuit_name: String,
    pub scenario_name: String,
    pub peak_pressure_bar: f64,
    pub final_alert: LeakAlertLevel,
    pub final_rupture_probability_pct: f64,
    pub hours_to_rupture: Option<f64>,
    pub likely_failure_mode: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LeakAlert {
    pub circuit_name: String,
    pub level: LeakAlertLevel,
    pub message: String,
    pub predicted_hours_to_rupture: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
struct PressureStats {
    min_bar: f64,
    max_bar: f64,
    mean_bar: f64,
}

impl PressureStats {
    fn new(seed_bar: f64) -> Self {
        Self {
            min_bar: seed_bar,
            max_bar: seed_bar,
            mean_bar: seed_bar,
        }
    }

    fn update(&mut self, p_bar: f64) {
        self.min_bar = self.min_bar.min(p_bar);
        self.max_bar = self.max_bar.max(p_bar);
        self.mean_bar = self.mean_bar * 0.992 + p_bar * 0.008;
    }
}

#[derive(Debug, Clone, Copy)]
struct SealState {
    damage_index: f64,
    extrusion_index: f64,
    thermal_damage: f64,
    cumulative_overpressure_s: f64,
    total_operating_h: f64,
    leak_area_mm2: f64,
    leak_lpm: f64,
    rupture_confirmed: bool,
    rupture_pressure_bar: Option<f64>,
    rupture_elapsed_h: Option<f64>,
    rupture_probability_pct: f64,
}

impl SealState {
    fn new(base_area_mm2: f64) -> Self {
        Self {
            damage_index: 0.0,
            extrusion_index: 0.0,
            thermal_damage: 0.0,
            cumulative_overpressure_s: 0.0,
            total_operating_h: 0.0,
            leak_area_mm2: base_area_mm2,
            leak_lpm: 0.0,
            rupture_confirmed: false,
            rupture_pressure_bar: None,
            rupture_elapsed_h: None,
            rupture_probability_pct: 0.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LeakCircuit {
    pub name: String,
    pub application: String,
    pub component: CircuitComponent,
    pub oil_type: OilType,
    pub seal_count: u32,
    pub piston_pressure_bar: f64,
    pub operation_pressure_bar: f64,
    pub pressure: PressureEnvelope,
    pub spec: OringSpec,
    pub coeffs: LeakModelCoefficients,
    pub reservoir_volume_l: f64,
    pub pressure_support_lpm: f64,
    stats: PressureStats,
    state: SealState,
}

impl LeakCircuit {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        name: &str,
        application: &str,
        component: CircuitComponent,
        oil_type: OilType,
        seal_count: u32,
        piston_pressure_bar: f64,
        operation_pressure_bar: f64,
        pressure: PressureEnvelope,
        spec: OringSpec,
        reservoir_volume_l: f64,
        pressure_support_lpm: f64,
    ) -> Self {
        Self {
            name: name.to_string(),
            application: application.to_string(),
            component,
            oil_type,
            seal_count,
            piston_pressure_bar,
            operation_pressure_bar,
            pressure,
            coeffs: LeakModelCoefficients::default(),
            stats: PressureStats::new(pressure.mean_bar),
            state: SealState::new(spec.base_leak_area_mm2),
            spec,
            reservoir_volume_l,
            pressure_support_lpm,
        }
    }

    pub fn step(&mut self, input: CircuitInput, dt: f64) -> CircuitResult {
        let dt = dt.max(1e-6);
        self.stats.update(input.pressure_bar.max(0.0));
        self.state.total_operating_h += dt / 3600.0;

        let (t_min, t_max) = self.spec.material.temp_window_c();
        let temp_excess = if input.temp_c > t_max {
            (input.temp_c - t_max) / 20.0
        } else if input.temp_c < t_min {
            (t_min - input.temp_c) / 25.0
        } else {
            0.0
        };

        let pressure_ratio_max = input.pressure_bar / self.pressure.max_bar.max(1e-3);
        let pressure_ratio_rupt = input.pressure_bar / self.pressure.rupture_bar.max(1e-3);

        if input.pressure_bar > self.pressure.max_bar {
            self.state.cumulative_overpressure_s += dt;
        }

        let squeeze_target = 0.20;
        let squeeze = (self.spec.squeeze_pct / 100.0).clamp(0.05, 0.45);
        let squeeze_penalty = ((squeeze - squeeze_target).abs() / squeeze_target).clamp(0.0, 2.0);

        let hardness_penalty = ((80.0 - self.spec.shore_a).max(0.0) / 40.0).clamp(0.0, 1.0);
        let gap_penalty = (self.spec.extrusion_gap_mm / 0.25).clamp(0.2, 3.0);
        let age_ratio =
            (self.state.total_operating_h / self.spec.design_life_hours.max(1.0)).clamp(0.0, 4.0);
        let cycle_factor = (input.cycles_per_s / 2.0).clamp(0.0, 4.0);
        let duty = input.duty_01.clamp(0.0, 1.0);
        let oil_vi = self.oil_type.viscosity_index();
        let vi_factor = (130.0 / oil_vi.max(60.0)).clamp(0.7, 1.8);
        let piston_overload =
            (self.piston_pressure_bar / self.pressure.ideal_bar.max(1e-3)).clamp(0.4, 2.2);
        let operation_stress =
            (self.operation_pressure_bar / self.pressure.ideal_bar.max(1e-3)).clamp(0.4, 2.5);

        let mut damage_rate = 2.2e-6 * self.coeffs.damage_rate_scale.max(0.05);
        damage_rate *= 1.0 + 1.9 * (pressure_ratio_max - 0.85).max(0.0).powi(2);
        damage_rate *= 1.0 + 0.75 * temp_excess.max(0.0);
        damage_rate *= 1.0 + 0.45 * squeeze_penalty;
        damage_rate *= 1.0 + 0.35 * cycle_factor;
        damage_rate *= 1.0 + 0.28 * age_ratio;
        damage_rate *= 0.8 + 0.6 * duty;
        damage_rate *= vi_factor;
        damage_rate *= 0.75 + 0.25 * piston_overload;
        damage_rate *= 0.75 + 0.25 * operation_stress;

        self.state.damage_index = (self.state.damage_index + damage_rate * dt).clamp(0.0, 1.6);

        let extrusion_drive =
            (pressure_ratio_max - 0.9).max(0.0) * (1.0 + hardness_penalty) * gap_penalty;
        self.state.extrusion_index = (self.state.extrusion_index
            + extrusion_drive * dt * 1.8e-4 * self.coeffs.extrusion_rate_scale.max(0.05))
            .clamp(0.0, 1.5);

        let thermal_rate = (temp_excess.max(0.0) * (0.9 + 0.4 * duty))
            * 2.5e-5
            * self.coeffs.thermal_rate_scale.max(0.05);
        self.state.thermal_damage = (self.state.thermal_damage + thermal_rate * dt).clamp(0.0, 1.2);

        if matches!(self.component, CircuitComponent::AcHose) {
            self.state.extrusion_index = (self.state.extrusion_index
                + (pressure_ratio_max - 0.75).max(0.0)
                    * dt
                    * 2.2e-4
                    * self.coeffs.extrusion_rate_scale.max(0.05))
                .clamp(0.0, 1.8);
        }

        if !self.state.rupture_confirmed {
            let immediate_burst = input.pressure_bar >= self.pressure.rupture_bar;
            let fatigue_burst = self.state.damage_index > 1.0 && self.state.extrusion_index > 0.8;
            let thermal_burst = self.state.thermal_damage > 1.0 && pressure_ratio_max > 0.95;
            if immediate_burst || fatigue_burst || thermal_burst {
                self.state.rupture_confirmed = true;
                self.state.rupture_pressure_bar = Some(input.pressure_bar);
                self.state.rupture_elapsed_h = Some(self.state.total_operating_h);
            }
        }

        let micro_area = self.spec.base_leak_area_mm2
            * (1.0 + 0.65 * squeeze_penalty + 0.012 * self.spec.compression_set_pct)
            * self.spec.material.gas_permeability_factor();
        let fatigue_area = self.spec.cross_section_mm * self.state.damage_index.powi(2) * 0.003;
        let extrusion_area = self.spec.cross_section_mm * self.state.extrusion_index * 0.0025;
        let thermal_area = self.spec.cross_section_mm * self.state.thermal_damage * 0.0018;
        let rupture_area = if self.state.rupture_confirmed {
            if matches!(self.component, CircuitComponent::AcHose) {
                self.spec.cross_section_mm
                    * self.spec.cross_section_mm
                    * 0.14
                    * self.coeffs.rupture_area_scale.max(0.05)
            } else {
                self.spec.cross_section_mm
                    * self.spec.cross_section_mm
                    * 0.06
                    * self.coeffs.rupture_area_scale.max(0.05)
            }
        } else {
            0.0
        };

        self.state.leak_area_mm2 =
            (micro_area + fatigue_area + extrusion_area + thermal_area + rupture_area)
                * self.seal_count.max(1) as f64;

        let dp_pa = (input.delta_p_bar.max(input.pressure_bar).max(0.0)) * 1.0e5;
        let rho = input.fluid_density_kg_m3.max(120.0);
        let area_m2 = self.state.leak_area_mm2 * 1.0e-6;
        let q_m3_s = self.spec.discharge_coeff.clamp(0.05, 1.0)
            * self.coeffs.flow_rate_scale.max(0.05)
            * area_m2
            * (2.0 * dp_pa / rho).sqrt();
        self.state.leak_lpm = q_m3_s * 60000.0;

        let risk = 34.0 * (pressure_ratio_max - 0.8).max(0.0)
            + 26.0 * (pressure_ratio_rupt - 0.7).max(0.0)
            + 18.0 * self.state.damage_index
            + 12.0 * self.state.extrusion_index
            + 10.0 * self.state.thermal_damage;
        self.state.rupture_probability_pct = risk.clamp(0.0, 100.0);

        let rate_to_failure = damage_rate
            + extrusion_drive * 1.8e-4 * self.coeffs.extrusion_rate_scale.max(0.05)
            + thermal_rate;
        let pred_h = if self.state.rupture_confirmed {
            Some(0.0)
        } else if rate_to_failure > 1e-8 {
            Some(((1.0 - self.state.damage_index).max(0.0) / rate_to_failure) / 3600.0)
        } else {
            None
        };

        let alert = if self.state.rupture_confirmed {
            LeakAlertLevel::Ruptured
        } else if pressure_ratio_rupt > 0.9 || self.state.rupture_probability_pct > 85.0 {
            LeakAlertLevel::Critical
        } else if pressure_ratio_max > 1.0 || self.state.rupture_probability_pct > 65.0 {
            LeakAlertLevel::Warning
        } else if pressure_ratio_max > 0.85 || self.state.rupture_probability_pct > 40.0 {
            LeakAlertLevel::Watch
        } else {
            LeakAlertLevel::Normal
        };

        let pressure_band = if input.pressure_bar >= self.pressure.rupture_bar {
            PressureBand::AtRupture
        } else if input.pressure_bar > self.pressure.max_bar {
            PressureBand::OverMaximum
        } else if input.pressure_bar < self.pressure.min_bar {
            PressureBand::BelowMinimum
        } else {
            let span = (self.pressure.max_bar - self.pressure.min_bar).max(1e-6);
            let ratio = (input.pressure_bar - self.pressure.min_bar) / span;
            if ratio < 0.33 {
                PressureBand::LowBand
            } else if ratio < 0.66 {
                PressureBand::MidBand
            } else {
                PressureBand::HighBand
            }
        };

        let recommended_hold_min_bar = (self.pressure.ideal_bar * 0.65).max(self.pressure.min_bar);
        let recommended_hold_target_bar = self.pressure.ideal_bar;
        let recommended_hold_max_bar = (self.pressure.max_bar * 0.88).min(self.pressure.rupture_bar * 0.78);
        let margin_to_max_bar = self.pressure.max_bar - input.pressure_bar;
        let margin_to_rupture_bar = self.pressure.rupture_bar - input.pressure_bar;

        let rca_hint = if self.state.rupture_confirmed {
            format!(
                "ruptured_at={:.2}bar; damage={:.3}; extrusion={:.3}; thermal={:.3}",
                self.state.rupture_pressure_bar.unwrap_or(input.pressure_bar),
                self.state.damage_index,
                self.state.extrusion_index,
                self.state.thermal_damage
            )
        } else {
            format!(
                "band={}; margin_to_max={:.2}bar; margin_to_rupture={:.2}bar",
                pressure_band, margin_to_max_bar, margin_to_rupture_bar
            )
        };

        let pca_recommendation = if self.state.rupture_confirmed {
            format!(
                "keep_between_{:.1}_and_{:.1}_bar; reduce_cycles_or_temp; review_material={}",
                recommended_hold_min_bar,
                recommended_hold_max_bar,
                self.spec.material.name()
            )
        } else if matches!(pressure_band, PressureBand::OverMaximum | PressureBand::AtRupture) {
            format!(
                "immediate_derate_to_{:.1}_bar_target_{:.1}_bar",
                recommended_hold_max_bar, recommended_hold_target_bar
            )
        } else {
            format!(
                "hold_{:.1}..{:.1}_bar_target_{:.1}_bar",
                recommended_hold_min_bar, recommended_hold_max_bar, recommended_hold_target_bar
            )
        };

        let warning_text = match alert {
            LeakAlertLevel::Ruptured => format!(
                "{}: ruptura detectada em {:.2} bar ({:.3} h); vazamento {:.2} L/min; manter {:.1}-{:.1} bar",
                self.application,
                self.state.rupture_pressure_bar.unwrap_or(input.pressure_bar),
                self.state.rupture_elapsed_h.unwrap_or(self.state.total_operating_h),
                self.state.leak_lpm,
                recommended_hold_min_bar,
                recommended_hold_max_bar
            ),
            LeakAlertLevel::Critical => format!(
                "{}: risco critico ({:.0}%); ponto={} p={:.1}bar; margem ruptura {:.1}bar",
                self.application,
                self.state.rupture_probability_pct,
                pressure_band,
                input.pressure_bar,
                margin_to_rupture_bar
            ),
            LeakAlertLevel::Warning => format!(
                "{}: sobrecarga progressiva; ponto={} p={:.1}bar; manter <= {:.1}bar",
                self.application,
                pressure_band,
                input.pressure_bar,
                recommended_hold_max_bar
            ),
            LeakAlertLevel::Watch => format!(
                "{}: degradacao progressiva; ponto={} p={:.1}bar; alvo {:.1}bar",
                self.application,
                pressure_band,
                input.pressure_bar,
                recommended_hold_target_bar
            ),
            LeakAlertLevel::Normal => format!(
                "{}: nominal; ponto={} p={:.1}bar",
                self.application,
                pressure_band,
                input.pressure_bar
            ),
        };

        CircuitResult {
            name: self.name.clone(),
            alert,
            pressure_band,
            current_pressure_bar: input.pressure_bar,
            rupture_probability_pct: self.state.rupture_probability_pct,
            predicted_hours_to_rupture: pred_h,
            rupture_pressure_bar: self.state.rupture_pressure_bar,
            rupture_elapsed_h: self.state.rupture_elapsed_h,
            leak_area_mm2: self.state.leak_area_mm2,
            leak_lpm: self.state.leak_lpm,
            pressure_min_bar: self.stats.min_bar,
            pressure_mean_bar: self.stats.mean_bar,
            pressure_max_bar: self.stats.max_bar,
            recommended_hold_min_bar,
            recommended_hold_target_bar,
            recommended_hold_max_bar,
            margin_to_max_bar,
            margin_to_rupture_bar,
            rca_hint,
            pca_recommendation,
            rupture_confirmed: self.state.rupture_confirmed,
            warning_text,
        }
    }

    pub fn reset(&mut self) {
        self.stats = PressureStats::new(self.pressure.mean_bar);
        self.state = SealState::new(self.spec.base_leak_area_mm2);
    }

    pub fn apply_manual_params(&mut self, p: ManualCircuitParams) {
        if let Some(v) = p.oil_type {
            self.oil_type = v;
        }
        if let Some(v) = p.piston_pressure_bar {
            self.piston_pressure_bar = v.max(0.0);
        }
        if let Some(v) = p.operation_pressure_bar {
            self.operation_pressure_bar = v.max(0.0);
        }
        if let Some(v) = p.pressure_min_bar {
            self.pressure.min_bar = v.max(0.0);
        }
        if let Some(v) = p.pressure_mean_bar {
            self.pressure.mean_bar = v.max(0.0);
        }
        if let Some(v) = p.pressure_ideal_bar {
            self.pressure.ideal_bar = v.max(0.0);
        }
        if let Some(v) = p.pressure_max_bar {
            self.pressure.max_bar = v.max(0.0);
        }
        if let Some(v) = p.pressure_rupture_bar {
            self.pressure.rupture_bar = v.max(self.pressure.max_bar + 0.1);
        }
        if let Some(v) = p.oring_squeeze_pct {
            self.spec.squeeze_pct = v.clamp(5.0, 45.0);
        }
        if let Some(v) = p.compression_set_pct {
            self.spec.compression_set_pct = v.clamp(0.0, 80.0);
        }
        if let Some(v) = p.base_leak_area_mm2 {
            self.spec.base_leak_area_mm2 = v.max(0.0);
        }
        if let Some(v) = p.max_supported_temp_c {
            let (tmin, _) = self.spec.material.temp_window_c();
            let _ = tmin;
            // user-specific temp ceiling shifts modeled via compression set drift
            let over = (self.operation_pressure_bar * 0.0 + 1.0) * (v / 180.0).clamp(0.4, 1.4);
            self.spec.compression_set_pct =
                (self.spec.compression_set_pct * (2.0 - over)).clamp(0.0, 80.0);
        }
    }
}

#[derive(Debug, Clone)]
pub struct LeakPhysicsRig {
    pub circuits: Vec<LeakCircuit>,
    pub alerts: Vec<LeakAlert>,
    pub total_leak_lpm: f64,
    pub last_results: Vec<CircuitResult>,
}

impl Default for LeakPhysicsRig {
    fn default() -> Self {
        Self::new()
    }
}

impl LeakPhysicsRig {
    pub fn new() -> Self {
        Self {
            circuits: Vec::new(),
            alerts: Vec::new(),
            total_leak_lpm: 0.0,
            last_results: Vec::new(),
        }
    }

    pub fn with_default_machine_presets() -> Self {
        let mut rig = Self::new();

        rig.circuits.push(LeakCircuit::new(
            "HYD_MAIN",
            "O-ring circuito hidraulico principal",
            CircuitComponent::Oring,
            OilType::HydraulicIso46,
            8,
            170.0,
            140.0,
            PressureEnvelope {
                min_bar: 45.0,
                mean_bar: 135.0,
                ideal_bar: 160.0,
                max_bar: 230.0,
                rupture_bar: 285.0,
            },
            OringSpec {
                id_tag: "AS568-228 NBR90".to_string(),
                material: OringMaterial::Nbr,
                shore_a: 90.0,
                cross_section_mm: 5.33,
                squeeze_pct: 18.0,
                extrusion_gap_mm: 0.20,
                compression_set_pct: 12.0,
                design_life_hours: 12000.0,
                base_leak_area_mm2: 0.0007,
                discharge_coeff: 0.62,
            },
            90.0,
            180.0,
        ));

        rig.circuits.push(LeakCircuit::new(
            "ENG_OIL",
            "O-ring pressao de oleo do motor",
            CircuitComponent::Seal,
            OilType::Engine15w40,
            3,
            4.4,
            3.4,
            PressureEnvelope {
                min_bar: 1.2,
                mean_bar: 3.2,
                ideal_bar: 4.0,
                max_bar: 5.2,
                rupture_bar: 7.0,
            },
            OringSpec {
                id_tag: "AS568-214 FKM75".to_string(),
                material: OringMaterial::Fkm,
                shore_a: 75.0,
                cross_section_mm: 3.53,
                squeeze_pct: 20.0,
                extrusion_gap_mm: 0.08,
                compression_set_pct: 10.0,
                design_life_hours: 10000.0,
                base_leak_area_mm2: 0.0003,
                discharge_coeff: 0.58,
            },
            28.0,
            45.0,
        ));

        rig.circuits.push(LeakCircuit::new(
            "AC_HIGH",
            "O-ring alta do ar condicionado",
            CircuitComponent::AcHose,
            OilType::Pag46,
            6,
            16.0,
            12.5,
            PressureEnvelope {
                min_bar: 7.0,
                mean_bar: 12.0,
                ideal_bar: 14.0,
                max_bar: 20.0,
                rupture_bar: 28.0,
            },
            OringSpec {
                id_tag: "AS568-210 HNBR80".to_string(),
                material: OringMaterial::Hnbr,
                shore_a: 80.0,
                cross_section_mm: 3.53,
                squeeze_pct: 17.0,
                extrusion_gap_mm: 0.10,
                compression_set_pct: 14.0,
                design_life_hours: 9000.0,
                base_leak_area_mm2: 0.0005,
                discharge_coeff: 0.55,
            },
            1.1,
            12.0,
        ));

        rig
    }

    pub fn add_circuit(&mut self, circuit: LeakCircuit) {
        self.circuits.push(circuit);
    }

    pub fn circuit_mut(&mut self, name: &str) -> Option<&mut LeakCircuit> {
        self.circuits.iter_mut().find(|c| c.name == name)
    }

    pub fn oil_density_for(&self, name: &str) -> f64 {
        self.circuits
            .iter()
            .find(|c| c.name == name)
            .map(|c| c.oil_type.density_kg_m3())
            .unwrap_or(900.0)
    }

    pub fn set_runtime_pressure_state(
        &mut self,
        name: &str,
        piston_pressure_bar: f64,
        operation_pressure_bar: f64,
    ) {
        if let Some(c) = self.circuit_mut(name) {
            c.piston_pressure_bar = piston_pressure_bar.max(0.0);
            c.operation_pressure_bar = operation_pressure_bar.max(0.0);
        }
    }

    pub fn apply_manual_params(&mut self, name: &str, params: ManualCircuitParams) -> bool {
        if let Some(c) = self.circuit_mut(name) {
            c.apply_manual_params(params);
            true
        } else {
            false
        }
    }

    pub fn step(&mut self, inputs: &[(&str, CircuitInput)], dt: f64) -> Vec<CircuitResult> {
        self.alerts.clear();
        self.total_leak_lpm = 0.0;

        let mut results = Vec::new();
        for (name, input) in inputs {
            if let Some(circuit) = self.circuits.iter_mut().find(|c| c.name == *name) {
                let out = circuit.step(*input, dt);
                self.total_leak_lpm += out.leak_lpm;
                if out.alert >= LeakAlertLevel::Watch {
                    self.alerts.push(LeakAlert {
                        circuit_name: out.name.clone(),
                        level: out.alert,
                        message: out.warning_text.clone(),
                        predicted_hours_to_rupture: out.predicted_hours_to_rupture,
                    });
                }
                results.push(out);
            }
        }

        self.last_results = results.clone();
        results
    }

    pub fn reset(&mut self) {
        self.total_leak_lpm = 0.0;
        self.alerts.clear();
        self.last_results.clear();
        for c in &mut self.circuits {
            c.reset();
        }
    }

    pub fn export_last_results_json<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let payload = serde_json::to_string_pretty(&self.last_results)
            .map_err(|e| std::io::Error::other(format!("json serialization failed: {e}")))?;
        fs::write(path_ref, payload)
    }

    pub fn export_last_results_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let mut out = String::from(
            "name,alert,pressure_band,current_pressure_bar,rupture_probability_pct,predicted_hours_to_rupture,rupture_pressure_bar,rupture_elapsed_h,leak_area_mm2,leak_lpm,pressure_min_bar,pressure_mean_bar,pressure_max_bar,recommended_hold_min_bar,recommended_hold_target_bar,recommended_hold_max_bar,margin_to_max_bar,margin_to_rupture_bar,rca_hint,pca_recommendation,rupture_confirmed,warning_text\n",
        );
        for r in &self.last_results {
            let warn = r.warning_text.replace('"', "''").replace(',', ";");
            let rca = r.rca_hint.replace('"', "''").replace(',', ";");
            let pca = r.pca_recommendation.replace('"', "''").replace(',', ";");
            out.push_str(&format!(
                "{},{},{},{:.6},{:.6},{},{},{},{:.9},{:.9},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},\"{}\",\"{}\",{},\"{}\"\n",
                r.name,
                r.alert,
                r.pressure_band,
                r.current_pressure_bar,
                r.rupture_probability_pct,
                r.predicted_hours_to_rupture
                    .map(|v| format!("{v:.6}"))
                    .unwrap_or_default(),
                r.rupture_pressure_bar
                    .map(|v| format!("{v:.6}"))
                    .unwrap_or_default(),
                r.rupture_elapsed_h
                    .map(|v| format!("{v:.6}"))
                    .unwrap_or_default(),
                r.leak_area_mm2,
                r.leak_lpm,
                r.pressure_min_bar,
                r.pressure_mean_bar,
                r.pressure_max_bar,
                r.recommended_hold_min_bar,
                r.recommended_hold_target_bar,
                r.recommended_hold_max_bar,
                r.margin_to_max_bar,
                r.margin_to_rupture_bar,
                rca,
                pca,
                r.rupture_confirmed,
                warn
            ));
        }
        fs::write(path_ref, out)
    }

    pub fn export_predictions_json<P: AsRef<Path>>(
        &self,
        path: P,
        predictions: &[ScenarioPrediction],
    ) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let payload = serde_json::to_string_pretty(predictions)
            .map_err(|e| std::io::Error::other(format!("json serialization failed: {e}")))?;
        fs::write(path_ref, payload)
    }

    pub fn export_predictions_csv<P: AsRef<Path>>(
        &self,
        path: P,
        predictions: &[ScenarioPrediction],
    ) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let mut out = String::from("circuit_name,scenario_name,peak_pressure_bar,final_alert,final_rupture_probability_pct,hours_to_rupture,likely_failure_mode\n");
        for p in predictions {
            let mode = p.likely_failure_mode.replace('"', "''").replace(',', ";");
            out.push_str(&format!(
                "{},{},{:.6},{},{:.6},{},\"{}\"\n",
                p.circuit_name,
                p.scenario_name,
                p.peak_pressure_bar,
                p.final_alert,
                p.final_rupture_probability_pct,
                p.hours_to_rupture
                    .map(|v| format!("{v:.6}"))
                    .unwrap_or_default(),
                mode
            ));
        }
        fs::write(path_ref, out)
    }

    pub fn export_material_catalog_json<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let payload = serde_json::to_string_pretty(&engineering_material_catalog())
            .map_err(|e| std::io::Error::other(format!("json serialization failed: {e}")))?;
        fs::write(path_ref, payload)
    }

    pub fn export_material_catalog_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let mut out = String::from("id,family,designation,hardness_shore,tensile_strength_mpa,elongation_pct,compression_set_pct_70h,tear_strength_kn_m,density_g_cm3,temp_min_c,temp_max_c,abrasion_index,corrosion_resistance_index,hydraulic_oil_compat,engine_oil_compat,refrigerant_oil_compat,notes\n");
        for m in engineering_material_catalog() {
            let notes = m.notes.replace('"', "''").replace(',', ";");
            out.push_str(&format!(
                "{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},\"{}\"\n",
                m.id,
                m.family,
                m.designation,
                m.hardness_shore,
                m.tensile_strength_mpa,
                m.elongation_pct,
                m.compression_set_pct_70h,
                m.tear_strength_kn_m,
                m.density_g_cm3,
                m.temp_min_c,
                m.temp_max_c,
                m.abrasion_index,
                m.corrosion_resistance_index,
                m.hydraulic_oil_compat,
                m.engine_oil_compat,
                m.refrigerant_oil_compat,
                notes,
            ));
        }
        fs::write(path_ref, out)
    }

    pub fn export_oil_catalog_json<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let payload = serde_json::to_string_pretty(&engineering_oil_catalog())
            .map_err(|e| std::io::Error::other(format!("json serialization failed: {e}")))?;
        fs::write(path_ref, payload)
    }

    pub fn export_oil_catalog_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let mut out = String::from("id,family,grade,base_stock,density_15c_kg_m3,viscosity_40c_cst,viscosity_100c_cst,viscosity_index,pour_point_c,flash_point_c,tan_mg_koh_g,water_content_ppm_limit,demulsibility_min,copper_corrosion_level,oxidation_stability_h,recommended_min_temp_c,recommended_max_temp_c,anti_wear_index,corrosion_inhibition_index,notes\n");
        for o in engineering_oil_catalog() {
            let notes = o.notes.replace('"', "''").replace(',', ";");
            out.push_str(&format!(
                "{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},\"{}\"\n",
                o.id,
                o.family,
                o.grade,
                o.base_stock,
                o.density_15c_kg_m3,
                o.viscosity_40c_cst,
                o.viscosity_100c_cst,
                o.viscosity_index,
                o.pour_point_c,
                o.flash_point_c,
                o.tan_mg_koh_g,
                o.water_content_ppm_limit,
                o.demulsibility_min,
                o.copper_corrosion_level,
                o.oxidation_stability_h,
                o.recommended_min_temp_c,
                o.recommended_max_temp_c,
                o.anti_wear_index,
                o.corrosion_inhibition_index,
                notes,
            ));
        }
        fs::write(path_ref, out)
    }

    pub fn calibrate_from_csv<P: AsRef<Path>>(
        &mut self,
        csv_path: P,
    ) -> std::io::Result<CalibrationReport> {
        #[derive(Debug, Deserialize)]
        struct CsvRow {
            timestamp_s: Option<f64>,
            circuit_name: String,
            pressure_bar: f64,
            delta_p_bar: Option<f64>,
            temp_c: f64,
            cycles_per_s: f64,
            duty_01: f64,
            fluid_density_kg_m3: Option<f64>,
            measured_leak_lpm: f64,
            observed_rupture: Option<bool>,
        }

        let mut rdr = csv::ReaderBuilder::new()
            .has_headers(true)
            .flexible(true)
            .trim(csv::Trim::All)
            .from_path(csv_path.as_ref())
            .map_err(|e| std::io::Error::other(format!("open csv failed: {e}")))?;

        let mut samples_by_circuit: std::collections::BTreeMap<String, Vec<CalibrationCsvSample>> =
            std::collections::BTreeMap::new();

        for rec in rdr.deserialize::<CsvRow>() {
            let row = rec.map_err(|e| std::io::Error::other(format!("csv parse failed: {e}")))?;
            let circuit = row.circuit_name.trim().to_string();
            let density = row.fluid_density_kg_m3.unwrap_or(900.0);
            samples_by_circuit
                .entry(circuit.clone())
                .or_default()
                .push(CalibrationCsvSample {
                    timestamp_s: row.timestamp_s,
                    circuit_name: circuit,
                    pressure_bar: row.pressure_bar,
                    delta_p_bar: row.delta_p_bar.unwrap_or(row.pressure_bar),
                    temp_c: row.temp_c,
                    cycles_per_s: row.cycles_per_s,
                    duty_01: row.duty_01,
                    fluid_density_kg_m3: density,
                    measured_leak_lpm: row.measured_leak_lpm,
                    observed_rupture: row.observed_rupture,
                });
        }

        let mut circuit_reports = Vec::new();

        for circuit in &mut self.circuits {
            let Some(samples) = samples_by_circuit.get(&circuit.name) else {
                continue;
            };
            if samples.is_empty() {
                continue;
            }

            let mut best = circuit.coeffs;
            let mut best_score = f64::INFINITY;

            let grid_damage = [0.6, 0.8, 1.0, 1.2, 1.4];
            let grid_extr = [0.6, 0.85, 1.0, 1.2, 1.4];
            let grid_thermal = [0.6, 0.85, 1.0, 1.2, 1.4];
            let grid_flow = [0.7, 0.85, 1.0, 1.15, 1.3];

            for ds in grid_damage {
                for es in grid_extr {
                    for ts in grid_thermal {
                        for fs in grid_flow {
                            let mut sim = circuit.clone();
                            sim.reset();
                            sim.coeffs = LeakModelCoefficients {
                                damage_rate_scale: ds,
                                extrusion_rate_scale: es,
                                thermal_rate_scale: ts,
                                flow_rate_scale: fs,
                                rupture_area_scale: 1.0,
                            };

                            let mut score = 0.0;
                            let mut prev_t: Option<f64> = None;
                            for s in samples {
                                let dt = match (prev_t, s.timestamp_s) {
                                    (Some(p), Some(t)) => (t - p).max(0.01),
                                    _ => 0.05,
                                };
                                prev_t = s.timestamp_s;

                                let out = sim.step(
                                    CircuitInput {
                                        pressure_bar: s.pressure_bar,
                                        delta_p_bar: s.delta_p_bar,
                                        temp_c: s.temp_c,
                                        cycles_per_s: s.cycles_per_s,
                                        duty_01: s.duty_01,
                                        fluid_density_kg_m3: s.fluid_density_kg_m3,
                                    },
                                    dt,
                                );

                                let abs_err = (out.leak_lpm - s.measured_leak_lpm).abs();
                                let rel = abs_err / s.measured_leak_lpm.abs().max(0.05);
                                score += rel;
                                if let Some(obs_rupt) = s.observed_rupture {
                                    if obs_rupt != out.rupture_confirmed {
                                        score += 2.5;
                                    }
                                }
                            }

                            if score < best_score {
                                best_score = score;
                                best = sim.coeffs;
                            }
                        }
                    }
                }
            }

            circuit.coeffs = best;

            let mut eval = circuit.clone();
            eval.reset();
            let mut prev_t: Option<f64> = None;
            let mut se = 0.0;
            let mut ape = 0.0;
            let mut max_abs: f64 = 0.0;
            let mut rupture_hits = 0usize;
            let mut rupture_total = 0usize;
            for s in samples {
                let dt = match (prev_t, s.timestamp_s) {
                    (Some(p), Some(t)) => (t - p).max(0.01),
                    _ => 0.05,
                };
                prev_t = s.timestamp_s;

                let out = eval.step(
                    CircuitInput {
                        pressure_bar: s.pressure_bar,
                        delta_p_bar: s.delta_p_bar,
                        temp_c: s.temp_c,
                        cycles_per_s: s.cycles_per_s,
                        duty_01: s.duty_01,
                        fluid_density_kg_m3: s.fluid_density_kg_m3,
                    },
                    dt,
                );
                let err = out.leak_lpm - s.measured_leak_lpm;
                let abs_err = err.abs();
                se += err * err;
                ape += abs_err / s.measured_leak_lpm.abs().max(0.05);
                max_abs = max_abs.max(abs_err);
                if let Some(obs) = s.observed_rupture {
                    rupture_total += 1;
                    if obs == out.rupture_confirmed {
                        rupture_hits += 1;
                    }
                }
            }

            let n = samples.len() as f64;
            let rmse = (se / n.max(1.0)).sqrt();
            let mape = (ape / n.max(1.0)) * 100.0;
            let rupture_acc = if rupture_total > 0 {
                rupture_hits as f64 / rupture_total as f64 * 100.0
            } else {
                100.0
            };

            circuit_reports.push(CalibrationCircuitReport {
                circuit_name: circuit.name.clone(),
                material: circuit.spec.material.name().to_string(),
                oil: circuit.oil_type.name().to_string(),
                sample_count: samples.len(),
                rmse_leak_lpm: rmse,
                mape_leak_pct: mape,
                max_abs_error_lpm: max_abs,
                rupture_accuracy_pct: rupture_acc,
                fitted: best,
            });
        }

        let by_material = aggregate_group_report(&circuit_reports, |r| r.material.clone());
        let by_oil = aggregate_group_report(&circuit_reports, |r| r.oil.clone());
        let total_samples = circuit_reports.iter().map(|r| r.sample_count).sum();

        Ok(CalibrationReport {
            total_samples,
            calibrated_circuits: circuit_reports.len(),
            circuit_reports,
            by_material,
            by_oil,
        })
    }

    pub fn export_calibration_report_json<P: AsRef<Path>>(
        &self,
        path: P,
        report: &CalibrationReport,
    ) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        let payload = serde_json::to_string_pretty(report)
            .map_err(|e| std::io::Error::other(format!("json serialization failed: {e}")))?;
        fs::write(path_ref, payload)
    }

    pub fn export_calibration_report_csv<P: AsRef<Path>>(
        &self,
        path: P,
        report: &CalibrationReport,
    ) -> std::io::Result<()> {
        let path_ref = path.as_ref();
        if let Some(parent) = path_ref.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }

        let mut out = String::from("section,key,sample_count,rmse_leak_lpm,mape_leak_pct,max_abs_error_lpm,rupture_accuracy_pct,damage_rate_scale,extrusion_rate_scale,thermal_rate_scale,flow_rate_scale,rupture_area_scale\n");

        for r in &report.circuit_reports {
            out.push_str(&format!(
                "circuit,{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6}\n",
                r.circuit_name,
                r.sample_count,
                r.rmse_leak_lpm,
                r.mape_leak_pct,
                r.max_abs_error_lpm,
                r.rupture_accuracy_pct,
                r.fitted.damage_rate_scale,
                r.fitted.extrusion_rate_scale,
                r.fitted.thermal_rate_scale,
                r.fitted.flow_rate_scale,
                r.fitted.rupture_area_scale,
            ));
        }

        for g in &report.by_material {
            out.push_str(&format!(
                "material,{},{},{:.6},{:.6},,{:.6},,,,,\n",
                g.group,
                g.sample_count,
                g.mean_rmse_lpm,
                g.mean_mape_pct,
                g.mean_rupture_accuracy_pct,
            ));
        }

        for g in &report.by_oil {
            out.push_str(&format!(
                "oil,{},{},{:.6},{:.6},,{:.6},,,,,\n",
                g.group,
                g.sample_count,
                g.mean_rmse_lpm,
                g.mean_mape_pct,
                g.mean_rupture_accuracy_pct,
            ));
        }
        fs::write(path_ref, out)
    }

    pub fn run_monte_carlo(
        &self,
        runs: usize,
        horizon_s: f64,
        dt: f64,
        variation_pct: f64,
    ) -> Vec<ScenarioPrediction> {
        let mut out = Vec::new();
        let n = runs.max(1);
        let spread = (variation_pct / 100.0).clamp(0.0, 0.95);
        for base in &self.circuits {
            for i in 0..n {
                let mut c = base.clone();
                c.reset();
                let phi = ((i as f64 * 1.618_033_988_75).fract() - 0.5) * 2.0;
                let factor = 1.0 + phi * spread;
                c.operation_pressure_bar *= factor;
                c.piston_pressure_bar *= 1.0 + phi * spread * 0.7;
                c.spec.squeeze_pct =
                    (c.spec.squeeze_pct * (1.0 - phi * spread * 0.2)).clamp(5.0, 45.0);
                c.spec.compression_set_pct = (c.spec.compression_set_pct
                    * (1.0 + phi.abs() * spread * 0.3))
                    .clamp(0.0, 80.0);

                let mut t = 0.0;
                let mut last = CircuitResult {
                    name: c.name.clone(),
                    alert: LeakAlertLevel::Normal,
                    pressure_band: PressureBand::MidBand,
                    current_pressure_bar: c.pressure.mean_bar,
                    rupture_probability_pct: 0.0,
                    predicted_hours_to_rupture: None,
                    rupture_pressure_bar: None,
                    rupture_elapsed_h: None,
                    leak_area_mm2: 0.0,
                    leak_lpm: 0.0,
                    pressure_min_bar: c.pressure.min_bar,
                    pressure_mean_bar: c.pressure.mean_bar,
                    pressure_max_bar: c.pressure.max_bar,
                    recommended_hold_min_bar: c.pressure.ideal_bar * 0.65,
                    recommended_hold_target_bar: c.pressure.ideal_bar,
                    recommended_hold_max_bar: c.pressure.max_bar * 0.88,
                    margin_to_max_bar: c.pressure.max_bar - c.pressure.mean_bar,
                    margin_to_rupture_bar: c.pressure.rupture_bar - c.pressure.mean_bar,
                    rca_hint: String::new(),
                    pca_recommendation: String::new(),
                    rupture_confirmed: false,
                    warning_text: String::new(),
                };

                while t < horizon_s {
                    let p = c.pressure.mean_bar * (1.0 + (t * 0.27).sin() * 0.1) * factor;
                    last = c.step(
                        CircuitInput {
                            pressure_bar: p,
                            delta_p_bar: (p - c.pressure.min_bar).max(0.0),
                            temp_c: 40.0 + (t * 0.05).sin() * 8.0,
                            cycles_per_s: 1.2 + (t * 0.11).sin().abs() * 1.8,
                            duty_01: 0.55 + (t * 0.07).sin().abs() * 0.35,
                            fluid_density_kg_m3: c.oil_type.density_kg_m3(),
                        },
                        dt.max(0.005),
                    );
                    t += dt.max(0.005);
                    if last.alert == LeakAlertLevel::Ruptured {
                        break;
                    }
                }

                out.push(ScenarioPrediction {
                    circuit_name: c.name.clone(),
                    scenario_name: format!("MC_RUN_{:04}", i),
                    peak_pressure_bar: c.stats.max_bar,
                    final_alert: last.alert,
                    final_rupture_probability_pct: last.rupture_probability_pct,
                    hours_to_rupture: last.predicted_hours_to_rupture,
                    likely_failure_mode: if last.rupture_confirmed {
                        "MonteCarlo rupture".into()
                    } else {
                        "MonteCarlo degradation".into()
                    },
                });
            }
        }
        out.sort_by(|a, b| {
            b.final_rupture_probability_pct
                .total_cmp(&a.final_rupture_probability_pct)
        });
        out
    }

    pub fn predict_scenarios(&self, horizon_s: f64, dt: f64) -> Vec<ScenarioPrediction> {
        let scenarios: [(&str, f64, f64, f64, f64); 4] = [
            ("Nominal", 1.00, 1.00, 1.00, 1.00),
            ("Sobrepressao Ciclica", 1.25, 1.15, 1.20, 1.10),
            ("Choque Termico", 1.05, 1.35, 1.30, 1.10),
            ("Pico + Vibracao", 1.35, 1.10, 1.50, 1.20),
        ];

        let mut out = Vec::new();
        for base in &self.circuits {
            for (name, p_mul, t_mul, cyc_mul, duty_mul) in scenarios {
                let mut c = base.clone();
                c.reset();
                let mut t = 0.0;
                let mut last = CircuitResult {
                    name: c.name.clone(),
                    alert: LeakAlertLevel::Normal,
                    pressure_band: PressureBand::MidBand,
                    current_pressure_bar: c.pressure.mean_bar,
                    rupture_probability_pct: 0.0,
                    predicted_hours_to_rupture: None,
                    rupture_pressure_bar: None,
                    rupture_elapsed_h: None,
                    leak_area_mm2: 0.0,
                    leak_lpm: 0.0,
                    pressure_min_bar: c.pressure.min_bar,
                    pressure_mean_bar: c.pressure.mean_bar,
                    pressure_max_bar: c.pressure.max_bar,
                    recommended_hold_min_bar: c.pressure.ideal_bar * 0.65,
                    recommended_hold_target_bar: c.pressure.ideal_bar,
                    recommended_hold_max_bar: c.pressure.max_bar * 0.88,
                    margin_to_max_bar: c.pressure.max_bar - c.pressure.mean_bar,
                    margin_to_rupture_bar: c.pressure.rupture_bar - c.pressure.mean_bar,
                    rca_hint: String::new(),
                    pca_recommendation: String::new(),
                    rupture_confirmed: false,
                    warning_text: String::new(),
                };

                while t < horizon_s {
                    let phase = (t / 4.5).sin() * 0.08 + 1.0;
                    let p = c.pressure.mean_bar * p_mul * phase;
                    let input = CircuitInput {
                        pressure_bar: p,
                        delta_p_bar: (p - c.pressure.min_bar).max(0.0),
                        temp_c: 42.0 * t_mul,
                        cycles_per_s: 1.6 * cyc_mul,
                        duty_01: (0.62 * duty_mul).clamp(0.0, 1.0),
                        fluid_density_kg_m3: c.oil_type.density_kg_m3(),
                    };
                    last = c.step(input, dt);
                    t += dt;
                    if last.alert == LeakAlertLevel::Ruptured {
                        break;
                    }
                }

                let mode = if last.rupture_confirmed {
                    if matches!(c.component, CircuitComponent::AcHose) {
                        "Burst de mangueira A/C"
                    } else if c.state.extrusion_index > c.state.thermal_damage {
                        "Extrusao de vedacao"
                    } else {
                        "Ruptura por fadiga termica"
                    }
                } else if last.alert >= LeakAlertLevel::Warning {
                    "Falha progressiva de vedacao"
                } else {
                    "Sem ruptura no horizonte"
                };

                out.push(ScenarioPrediction {
                    circuit_name: c.name.clone(),
                    scenario_name: name.to_string(),
                    peak_pressure_bar: c.stats.max_bar,
                    final_alert: last.alert,
                    final_rupture_probability_pct: last.rupture_probability_pct,
                    hours_to_rupture: last.predicted_hours_to_rupture,
                    likely_failure_mode: mode.to_string(),
                });
            }
        }

        out.sort_by(|a, b| {
            b.final_rupture_probability_pct
                .total_cmp(&a.final_rupture_probability_pct)
        });
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn high_pressure_eventually_ruptures() {
        let mut rig = LeakPhysicsRig::with_default_machine_presets();
        let mut ruptured = false;
        for _ in 0..4000 {
            let results = rig.step(
                &[(
                    "HYD_MAIN",
                    CircuitInput {
                        pressure_bar: 320.0,
                        delta_p_bar: 300.0,
                        temp_c: 105.0,
                        cycles_per_s: 3.0,
                        duty_01: 0.95,
                        fluid_density_kg_m3: 860.0,
                    },
                )],
                0.02,
            );
            if let Some(r) = results.first() {
                if r.rupture_confirmed {
                    ruptured = true;
                    break;
                }
            }
        }
        assert!(
            ruptured,
            "expected rupture under sustained extreme overpressure"
        );
    }

    #[test]
    fn manual_params_are_applied() {
        let mut rig = LeakPhysicsRig::with_default_machine_presets();
        let ok = rig.apply_manual_params(
            "ENG_OIL",
            ManualCircuitParams {
                oil_type: Some(OilType::Engine10w30),
                piston_pressure_bar: Some(4.8),
                operation_pressure_bar: Some(3.9),
                pressure_min_bar: Some(1.1),
                pressure_mean_bar: Some(3.5),
                pressure_ideal_bar: Some(4.4),
                pressure_max_bar: Some(5.9),
                pressure_rupture_bar: Some(7.2),
                oring_squeeze_pct: Some(22.0),
                compression_set_pct: Some(15.0),
                base_leak_area_mm2: Some(0.0004),
                max_supported_temp_c: None,
            },
        );
        assert!(ok);
        let c = rig
            .circuits
            .iter()
            .find(|c| c.name == "ENG_OIL")
            .expect("circuit exists");
        assert_eq!(c.oil_type, OilType::Engine10w30);
        assert!(c.pressure.max_bar > 5.8);
        assert!(c.spec.squeeze_pct > 21.0);
    }

    #[test]
    fn scenario_prediction_returns_ranked_data() {
        let rig = LeakPhysicsRig::with_default_machine_presets();
        let pred = rig.predict_scenarios(180.0, 0.05);
        assert!(!pred.is_empty());
        assert!(pred.len() >= rig.circuits.len() * 4);
    }

    #[test]
    fn monte_carlo_produces_samples() {
        let rig = LeakPhysicsRig::with_default_machine_presets();
        let pred = rig.run_monte_carlo(20, 120.0, 0.05, 20.0);
        assert!(pred.len() >= rig.circuits.len() * 20);
    }

    #[test]
    fn exports_json_and_csv_files() {
        let mut rig = LeakPhysicsRig::with_default_machine_presets();
        let _ = rig.step(
            &[(
                "AC_HIGH",
                CircuitInput {
                    pressure_bar: 16.0,
                    delta_p_bar: 12.0,
                    temp_c: 55.0,
                    cycles_per_s: 2.0,
                    duty_01: 0.7,
                    fluid_density_kg_m3: 995.0,
                },
            )],
            0.1,
        );
        let pred = rig.predict_scenarios(60.0, 0.05);

        let root = std::env::temp_dir().join("autobreaking_leak_tests");
        let runtime_json = root.join("runtime.json");
        let runtime_csv = root.join("runtime.csv");
        let pred_json = root.join("pred.json");
        let pred_csv = root.join("pred.csv");

        rig.export_last_results_json(&runtime_json)
            .expect("runtime json");
        rig.export_last_results_csv(&runtime_csv)
            .expect("runtime csv");
        rig.export_predictions_json(&pred_json, &pred)
            .expect("pred json");
        rig.export_predictions_csv(&pred_csv, &pred)
            .expect("pred csv");

        assert!(runtime_json.exists());
        assert!(runtime_csv.exists());
        assert!(pred_json.exists());
        assert!(pred_csv.exists());
    }

    #[test]
    fn report_captures_pressure_band_and_safe_window() {
        let mut rig = LeakPhysicsRig::with_default_machine_presets();
        let out = rig.step(
            &[(
                "HYD_MAIN",
                CircuitInput {
                    pressure_bar: 150.0,
                    delta_p_bar: 110.0,
                    temp_c: 60.0,
                    cycles_per_s: 1.8,
                    duty_01: 0.65,
                    fluid_density_kg_m3: 865.0,
                },
            )],
            0.05,
        );

        let r = out.first().expect("result exists");
        assert!(matches!(r.pressure_band, PressureBand::LowBand | PressureBand::MidBand | PressureBand::HighBand));
        assert!(r.recommended_hold_min_bar < r.recommended_hold_target_bar);
        assert!(r.recommended_hold_target_bar < r.recommended_hold_max_bar);
        assert!(r.margin_to_rupture_bar > 0.0);
    }

    #[test]
    fn rupture_event_records_pressure_and_time() {
        let mut rig = LeakPhysicsRig::with_default_machine_presets();
        let mut rupture: Option<CircuitResult> = None;
        for _ in 0..6000 {
            let out = rig.step(
                &[(
                    "HYD_MAIN",
                    CircuitInput {
                        pressure_bar: 330.0,
                        delta_p_bar: 320.0,
                        temp_c: 120.0,
                        cycles_per_s: 3.8,
                        duty_01: 0.98,
                        fluid_density_kg_m3: 860.0,
                    },
                )],
                0.02,
            );
            if let Some(r) = out.first() {
                if r.rupture_confirmed {
                    rupture = Some(r.clone());
                    break;
                }
            }
        }

        let r = rupture.expect("rupture expected in deterministic stress profile");
        assert!(r.rupture_pressure_bar.is_some());
        assert!(r.rupture_elapsed_h.is_some());
        assert!(r.rupture_pressure_bar.expect("has rupture pressure") >= 280.0);
        assert!(r.warning_text.contains("ruptura"));
    }

    #[test]
    fn engineering_catalogs_are_large_and_coherent() {
        let mats = engineering_material_catalog();
        let oils = engineering_oil_catalog();
        assert!(mats.len() >= 30, "material catalog should be large");
        assert!(oils.len() >= 30, "oil catalog should be large");

        let mut mat_ids: Vec<&str> = mats.iter().map(|m| m.id).collect();
        mat_ids.sort_unstable();
        mat_ids.dedup();
        assert_eq!(mat_ids.len(), mats.len(), "material IDs must be unique");

        let mut oil_ids: Vec<&str> = oils.iter().map(|o| o.id).collect();
        oil_ids.sort_unstable();
        oil_ids.dedup();
        assert_eq!(oil_ids.len(), oils.len(), "oil IDs must be unique");
    }

    #[test]
    fn calibration_mode_ingests_csv_and_exports_reports() {
        let mut rig = LeakPhysicsRig::with_default_machine_presets();
        let root = std::env::temp_dir().join("autobreaking_calibration_tests");
        std::fs::create_dir_all(&root).expect("create calibration temp dir");
        let csv_path = root.join("bench_data.csv");

        let mut csv = String::from(
            "timestamp_s,circuit_name,pressure_bar,delta_p_bar,temp_c,cycles_per_s,duty_01,fluid_density_kg_m3,measured_leak_lpm,observed_rupture\n",
        );
        for i in 0..40 {
            let t = i as f64 * 0.05;
            let p = 120.0 + (i as f64 * 0.5);
            let dp = 95.0 + (i as f64 * 0.4);
            let temp = 65.0 + (i as f64 * 0.08);
            let measured_hyd = 0.11 + (i as f64 * 0.0012);
            csv.push_str(&format!(
                "{t:.3},HYD_MAIN,{p:.4},{dp:.4},{temp:.3},1.60,0.62,860.0,{measured_hyd:.6},false\n"
            ));

            let p_ac = 14.0 + (i as f64 * 0.07);
            let dp_ac = 10.0 + (i as f64 * 0.05);
            let measured_ac = 0.018 + (i as f64 * 0.00035);
            csv.push_str(&format!(
                "{t:.3},AC_HIGH,{p_ac:.4},{dp_ac:.4},48.0,1.20,0.55,995.0,{measured_ac:.6},false\n"
            ));
        }
        std::fs::write(&csv_path, csv).expect("write bench csv");

        let report = rig.calibrate_from_csv(&csv_path).expect("run calibration");
        assert!(report.calibrated_circuits >= 2);
        assert!(report.total_samples >= 80);
        assert!(report.by_material.iter().any(|g| g.sample_count > 0));
        assert!(report.by_oil.iter().any(|g| g.sample_count > 0));

        let csv_report = root.join("calibration_report.csv");
        let json_report = root.join("calibration_report.json");
        rig.export_calibration_report_csv(&csv_report, &report)
            .expect("export calibration csv");
        rig.export_calibration_report_json(&json_report, &report)
            .expect("export calibration json");

        assert!(csv_report.exists());
        assert!(json_report.exists());
    }
}
