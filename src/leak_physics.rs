//! O-ring leak / rupture physics for hydraulic, oil and refrigerant circuits.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum LeakAlertLevel {
    Normal,
    Watch,
    Warning,
    Critical,
    Ruptured,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CircuitComponent {
    Oring,
    Seal,
    AcHose,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OilType {
    HydraulicIso46,
    HydraulicIso68,
    Engine15w40,
    Engine10w30,
    Pag46,
    Poe68,
    Custom,
}

impl OilType {
    pub fn density_kg_m3(self) -> f64 {
        match self {
            OilType::HydraulicIso46 => 865.0,
            OilType::HydraulicIso68 => 875.0,
            OilType::Engine15w40 => 880.0,
            OilType::Engine10w30 => 870.0,
            OilType::Pag46 => 995.0,
            OilType::Poe68 => 980.0,
            OilType::Custom => 900.0,
        }
    }

    pub fn viscosity_index(self) -> f64 {
        match self {
            OilType::HydraulicIso46 => 105.0,
            OilType::HydraulicIso68 => 102.0,
            OilType::Engine15w40 => 140.0,
            OilType::Engine10w30 => 150.0,
            OilType::Pag46 => 130.0,
            OilType::Poe68 => 145.0,
            OilType::Custom => 120.0,
        }
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
}

impl OringMaterial {
    pub fn temp_window_c(self) -> (f64, f64) {
        match self {
            OringMaterial::Nbr => (-35.0, 110.0),
            OringMaterial::Hnbr => (-40.0, 150.0),
            OringMaterial::Fkm => (-20.0, 200.0),
            OringMaterial::Epdm => (-45.0, 140.0),
        }
    }

    pub fn gas_permeability_factor(self) -> f64 {
        match self {
            OringMaterial::Nbr => 1.00,
            OringMaterial::Hnbr => 0.92,
            OringMaterial::Fkm => 0.72,
            OringMaterial::Epdm => 1.10,
        }
    }
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
    pub rupture_probability_pct: f64,
    pub predicted_hours_to_rupture: Option<f64>,
    pub leak_area_mm2: f64,
    pub leak_lpm: f64,
    pub pressure_min_bar: f64,
    pub pressure_mean_bar: f64,
    pub pressure_max_bar: f64,
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
    pub reservoir_volume_l: f64,
    pub pressure_support_lpm: f64,
    stats: PressureStats,
    state: SealState,
}

impl LeakCircuit {
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

        let mut damage_rate = 2.2e-6;
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
        self.state.extrusion_index =
            (self.state.extrusion_index + extrusion_drive * dt * 1.8e-4).clamp(0.0, 1.5);

        let thermal_rate = (temp_excess.max(0.0) * (0.9 + 0.4 * duty)) * 2.5e-5;
        self.state.thermal_damage = (self.state.thermal_damage + thermal_rate * dt).clamp(0.0, 1.2);

        if matches!(self.component, CircuitComponent::AcHose) {
            self.state.extrusion_index = (self.state.extrusion_index
                + (pressure_ratio_max - 0.75).max(0.0) * dt * 2.2e-4)
                .clamp(0.0, 1.8);
        }

        if !self.state.rupture_confirmed {
            let immediate_burst = input.pressure_bar >= self.pressure.rupture_bar;
            let fatigue_burst = self.state.damage_index > 1.0 && self.state.extrusion_index > 0.8;
            let thermal_burst = self.state.thermal_damage > 1.0 && pressure_ratio_max > 0.95;
            if immediate_burst || fatigue_burst || thermal_burst {
                self.state.rupture_confirmed = true;
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
                self.spec.cross_section_mm * self.spec.cross_section_mm * 0.14
            } else {
                self.spec.cross_section_mm * self.spec.cross_section_mm * 0.06
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
        let q_m3_s =
            self.spec.discharge_coeff.clamp(0.05, 1.0) * area_m2 * (2.0 * dp_pa / rho).sqrt();
        self.state.leak_lpm = q_m3_s * 60000.0;

        let risk = 34.0 * (pressure_ratio_max - 0.8).max(0.0)
            + 26.0 * (pressure_ratio_rupt - 0.7).max(0.0)
            + 18.0 * self.state.damage_index
            + 12.0 * self.state.extrusion_index
            + 10.0 * self.state.thermal_damage;
        self.state.rupture_probability_pct = risk.clamp(0.0, 100.0);

        let rate_to_failure = damage_rate + extrusion_drive * 1.8e-4 + thermal_rate;
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

        let warning_text = match alert {
            LeakAlertLevel::Ruptured => format!(
                "{}: ruptura detectada; vazamento {:.2} L/min",
                self.application, self.state.leak_lpm
            ),
            LeakAlertLevel::Critical => format!(
                "{}: risco critico de ruptura ({:.0}%)",
                self.application, self.state.rupture_probability_pct
            ),
            LeakAlertLevel::Warning => format!(
                "{}: sobrecarga do O-ring, monitorar de perto",
                self.application
            ),
            LeakAlertLevel::Watch => {
                format!("{}: degradacao progressiva detectada", self.application)
            }
            LeakAlertLevel::Normal => format!("{}: comportamento nominal", self.application),
        };

        CircuitResult {
            name: self.name.clone(),
            alert,
            rupture_probability_pct: self.state.rupture_probability_pct,
            predicted_hours_to_rupture: pred_h,
            leak_area_mm2: self.state.leak_area_mm2,
            leak_lpm: self.state.leak_lpm,
            pressure_min_bar: self.stats.min_bar,
            pressure_mean_bar: self.stats.mean_bar,
            pressure_max_bar: self.stats.max_bar,
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
        let mut out = String::from("name,alert,rupture_probability_pct,predicted_hours_to_rupture,leak_area_mm2,leak_lpm,pressure_min_bar,pressure_mean_bar,pressure_max_bar,rupture_confirmed,warning_text\n");
        for r in &self.last_results {
            let warn = r.warning_text.replace('"', "''").replace(',', ";");
            out.push_str(&format!(
                "{},{},{:.6},{},{:.9},{:.9},{:.6},{:.6},{:.6},{},\"{}\"\n",
                r.name,
                r.alert,
                r.rupture_probability_pct,
                r.predicted_hours_to_rupture
                    .map(|v| format!("{v:.6}"))
                    .unwrap_or_default(),
                r.leak_area_mm2,
                r.leak_lpm,
                r.pressure_min_bar,
                r.pressure_mean_bar,
                r.pressure_max_bar,
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
                    rupture_probability_pct: 0.0,
                    predicted_hours_to_rupture: None,
                    leak_area_mm2: 0.0,
                    leak_lpm: 0.0,
                    pressure_min_bar: c.pressure.min_bar,
                    pressure_mean_bar: c.pressure.mean_bar,
                    pressure_max_bar: c.pressure.max_bar,
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
                    rupture_probability_pct: 0.0,
                    predicted_hours_to_rupture: None,
                    leak_area_mm2: 0.0,
                    leak_lpm: 0.0,
                    pressure_min_bar: c.pressure.min_bar,
                    pressure_mean_bar: c.pressure.mean_bar,
                    pressure_max_bar: c.pressure.max_bar,
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
}
