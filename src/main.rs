//! Heavy Machinery ECU Bench — GUI v3.0
//! Command-queue architecture: all UI→simulation mutations go through Cmd enum,
//! executed AFTER the UI frame, eliminating all borrow-checker closure conflicts.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
#![allow(float_literal_f32_fallback)]

pub mod io;
mod widgets;

use auto_breaking::{
    autonomous::FeatureState,
    boot_sequence::EcuBootStage,
    can_gateway::BusState,
    ecu_abs::EspCondition,
    ecu_ecm::AftertreatmentState,
    ecu_tcm::{AutoShiftMode, ClutchState, Direction},
    implement::PtoMode,
    j1939::DtcSeverity,
    v2x_telematics::ConnectState,
    CircuitComponent, HeavyMachinery, IgnitionState, LeakCircuit, ManualCircuitParams, OilType,
    OringMaterial, OringSpec, PressureEnvelope, ScenarioPrediction, FAULT_TYPES,
};
use chrono::Local;
use eframe::{egui, NativeOptions};
use egui::*;
use egui_plot::{Line, Plot, PlotPoints};
use std::collections::HashMap;
use std::time::Duration;
use widgets::{arc_gauge, bar_gauge, digital_readout, direction_selector, warning_lamp};

const DT: f64 = 1.0 / 60.0;
const PLOT_WINDOW: usize = 600;
const EVENT_MAX: usize = 500;

// ═════════════════════════════════════════════════════════════════════════════
// Command queue — ALL mutations to bench go through here, no closures needed
// ═════════════════════════════════════════════════════════════════════════════
#[derive(Debug, Clone)]
enum Cmd {
    // Ignition
    KeyAdvance,
    KeyOff,
    // Throttle/brake (raw value 0-100)
    SetThrottle(f32),
    SetBrake(f32),
    // Direction
    SetDirection(Direction),
    SetNeutral,
    // Gearbox
    ToggleAutoShift,
    ToggleCreeper,
    ManualUp,
    ManualDn,
    // ECM parameter overrides
    SetFuelLevel(f64),
    SetDefLevel(f64),
    SetCoolantTemp(f64),
    SetOilPressure(f64),
    SetBoostPressure(f64),
    SetEngineHours(f64),
    ClearDtcs,
    // BCM
    ToggleWorkLights,
    ToggleRoadLights,
    ToggleBeacon,
    Honk,
    CycleWiper,
    // Implements
    SetPtoMode(PtoMode),
    SetHitchTarget(f64),
    SetAuxValve(usize, f64),
    SetLoaderLift(f64),
    SetLoaderTilt(f64),
    // Hydraulics
    SetHitchJoystick(f64),
    // Autonomous
    EngageAD,
    DisengageAD,
    ToggleLka,
    AccSpeedSet(f64),
    #[allow(dead_code)]
    AccHeadwaySet(f64),
    AddWaypoint(f64, f64),
    ClearWaypoints,
    // Faults
    SelectFault(usize),
    InjectFault,
    ClearFaults,
    // Telematics
    StartOta,
    // Leak Lab
    LeakSelectCircuit(usize),
    LeakApplyManual,
    LeakPredictScenarios,
    LeakAddCustomCircuit,
    LeakExportReportCsv,
    LeakExportReportJson,
    LeakExportPredCsv,
    LeakExportPredJson,
    LeakRunMonteCarlo,
    // CAN network controls
    CanInjectBitError(usize),
    CanInjectAckError(usize),
    CanInjectBusOff(usize),
    CanInjectBabbling(usize),
    CanClearBusInjections(usize),
    CanClearAllInjections,
    CanExportSnapshotCsv,
    CanExportSnapshotJson,
    // Simulation
    #[allow(dead_code)]
    Pause,
    #[allow(dead_code)]
    Resume,
    Reset,
}

// ═════════════════════════════════════════════════════════════════════════════
// Tab enum
// ═════════════════════════════════════════════════════════════════════════════
#[derive(PartialEq, Clone, Copy, Debug)]
enum Tab {
    Cluster,
    CanBus,
    Events,
    EcuNet,
    Engine,
    Faults,
    Boot,
    Implements,
    Params,
    Sensors,
    Autonomous,
    V2X,
    Uds,
    EcmLiveData,
    LeakLab,
    Plots,
}

#[derive(Clone)]
struct LeakManualUi {
    oil_type_idx: usize,
    piston_pressure_bar: f64,
    operation_pressure_bar: f64,
    pressure_min_bar: f64,
    pressure_mean_bar: f64,
    pressure_ideal_bar: f64,
    pressure_max_bar: f64,
    pressure_rupture_bar: f64,
    squeeze_pct: f64,
    compression_set_pct: f64,
    base_leak_area_mm2: f64,
}

impl Default for LeakManualUi {
    fn default() -> Self {
        Self {
            oil_type_idx: 0,
            piston_pressure_bar: 100.0,
            operation_pressure_bar: 80.0,
            pressure_min_bar: 20.0,
            pressure_mean_bar: 60.0,
            pressure_ideal_bar: 80.0,
            pressure_max_bar: 120.0,
            pressure_rupture_bar: 160.0,
            squeeze_pct: 18.0,
            compression_set_pct: 12.0,
            base_leak_area_mm2: 0.0005,
        }
    }
}

#[derive(Clone)]
struct LeakCustomUi {
    name: String,
    application: String,
    component_idx: usize,
    oil_type_idx: usize,
    seal_count: u32,
    piston_pressure_bar: f64,
    operation_pressure_bar: f64,
    min_bar: f64,
    mean_bar: f64,
    ideal_bar: f64,
    max_bar: f64,
    rupture_bar: f64,
    material_idx: usize,
    shore_a: f64,
    cross_section_mm: f64,
    squeeze_pct: f64,
    extrusion_gap_mm: f64,
    compression_set_pct: f64,
    design_life_hours: f64,
    base_leak_area_mm2: f64,
    discharge_coeff: f64,
    reservoir_volume_l: f64,
    support_lpm: f64,
}

impl Default for LeakCustomUi {
    fn default() -> Self {
        Self {
            name: "CUSTOM_01".into(),
            application: "Circuito custom de producao".into(),
            component_idx: 0,
            oil_type_idx: 0,
            seal_count: 4,
            piston_pressure_bar: 120.0,
            operation_pressure_bar: 95.0,
            min_bar: 20.0,
            mean_bar: 70.0,
            ideal_bar: 95.0,
            max_bar: 130.0,
            rupture_bar: 180.0,
            material_idx: 0,
            shore_a: 80.0,
            cross_section_mm: 3.53,
            squeeze_pct: 18.0,
            extrusion_gap_mm: 0.12,
            compression_set_pct: 10.0,
            design_life_hours: 8000.0,
            base_leak_area_mm2: 0.0005,
            discharge_coeff: 0.58,
            reservoir_volume_l: 20.0,
            support_lpm: 40.0,
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
enum CanMode {
    Signals,
    Trace,
    Network,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum EventLevel {
    Debug,
    Info,
    Ok,
    Warn,
    Critical,
}

#[derive(Clone)]
struct AppEvent {
    ts: f64,
    source: &'static str,
    msg: String,
    lvl: EventLevel,
}

struct Signal {
    pgn_name: &'static str,
    sa_name: String,
    raw_id: u32,
    last_ts: f64,
    count: u64,
    period_ms: f64,
    decoded: Vec<(String, f64, String)>,
    fresh: bool,
}

// ═════════════════════════════════════════════════════════════════════════════
// Application state
// ═════════════════════════════════════════════════════════════════════════════
struct App {
    bench: HeavyMachinery,
    hw_cfg: io::hw::HwConfig,
    tab: Tab,
    cmds: Vec<Cmd>, // deferred command queue

    // ECM live operations state
    ecm_detected_sas: Vec<u8>,
    ecm_selected_idx: usize,
    ecm_connected: bool,
    ecm_live_feed: Option<io::live_runner::LiveFeed>,
    ecm_live_status: String,
    ecm_live_snapshot: io::ecm_params::EcmSnapshot,
    ecm_live_last_update_ms: u64,
    ecm_live_frames_total: u64,

    // Inputs mirrored here (bench mutations happen via Cmd)
    throttle: f32,
    brake: f32,

    // CAN monitor
    can_mode: CanMode,
    can_freeze: bool,
    can_filter: String,
    can_bus_idx: usize,
    can_note: String,
    sig_map: HashMap<(u32, u8), Signal>,
    // (ts, raw_id, sa, dlc, hex_data, pgn[sa], decoded_str)
    trace_snap: Vec<(f64, u32, u8, u8, String, String, String)>,

    // Events
    events: Vec<AppEvent>,
    ev_pause: bool,
    ev_filter: String,
    ev_min_level: EventLevel,

    // Previous state for event detection
    prev_rpm: f64,
    prev_gear: String,
    prev_dtcs: usize,
    prev_abs: bool,
    prev_esp: EspCondition,
    prev_ign: IgnitionState,
    prev_regen: bool,
    prev_mode: String,

    // Parameter editing — user-editable simulation params
    p_fuel: f64,
    p_def: f64,
    p_coolant: f64,
    p_oil_prs: f64,
    p_boost: f64,
    p_engine_h: f64,
    params_dirty: bool,

    // Implements controls
    #[allow(dead_code)]
    pto_target_rpm: f64,
    hitch_target: f64,
    aux_cmds: [f64; 4],

    // AD path planning
    ad_waypoints: Vec<[f64; 2]>, // local ENU metres
    #[allow(dead_code)]
    ad_canvas_pos: f64,

    // Fault panel
    fault_idx: usize,

    // UDS console
    uds_input: String,
    uds_sa: u8,
    uds_log: Vec<(bool, String)>,

    // Leak physics lab
    leak_sel_idx: usize,
    leak_manual: LeakManualUi,
    leak_custom: LeakCustomUi,
    leak_horizon_s: f64,
    leak_scenario_dt: f64,
    leak_predictions: Vec<ScenarioPrediction>,
    leak_note: String,

    // Plots
    pl_rpm: Vec<[f64; 2]>,
    pl_spd: Vec<[f64; 2]>,
    pl_torque: Vec<[f64; 2]>,
    pl_fuel: Vec<[f64; 2]>,
    pl_coolant: Vec<[f64; 2]>,
    pl_dpf: Vec<[f64; 2]>,
    pl_boost: Vec<[f64; 2]>,

    ticks: u64,
}

impl App {
    fn new(cc: &eframe::CreationContext, hw_cfg: io::hw::HwConfig) -> Self {
        let mut vis = Visuals::dark();
        vis.panel_fill = Color32::from_gray(14);
        cc.egui_ctx.set_visuals(vis);

        let bench = HeavyMachinery::new();
        let fuel = bench.ecm.fuel_level_pct;
        let def = bench.ecm.def_level_pct;
        let cool = bench.ecm.coolant_temp_c;
        let oil = bench.ecm.oil_pressure_kpa;
        let boost = bench.ecm.boost_pressure_kpa;
        let hrs = bench.ecm.engine_hours;

        App {
            bench,
            hw_cfg,
            tab: Tab::Cluster,
            cmds: Vec::new(),
            ecm_detected_sas: Vec::new(),
            ecm_selected_idx: 0,
            ecm_connected: false,
            ecm_live_feed: None,
            ecm_live_status: "Idle. Press Detect to scan ECMs on the network.".into(),
            ecm_live_snapshot: io::ecm_params::EcmSnapshot::default(),
            ecm_live_last_update_ms: 0,
            ecm_live_frames_total: 0,
            throttle: 0.0,
            brake: 0.0,
            can_mode: CanMode::Signals,
            can_freeze: false,
            can_filter: String::new(),
            sig_map: HashMap::new(),
            trace_snap: Vec::new(),
            can_bus_idx: 0,
            can_note: String::new(),
            events: Vec::new(),
            ev_pause: false,
            ev_filter: String::new(),
            ev_min_level: EventLevel::Info,
            prev_rpm: 0.0,
            prev_gear: "N".into(),
            prev_dtcs: 0,
            prev_abs: false,
            prev_esp: EspCondition::Neutral,
            prev_ign: IgnitionState::Off,
            prev_regen: false,
            prev_mode: String::new(),
            p_fuel: fuel,
            p_def: def,
            p_coolant: cool,
            p_oil_prs: oil,
            p_boost: boost,
            p_engine_h: hrs,
            params_dirty: false,
            pto_target_rpm: 540.0,
            hitch_target: 100.0,
            aux_cmds: [0.0; 4],
            ad_waypoints: Vec::new(),
            ad_canvas_pos: 0.0,
            fault_idx: 0,
            uds_input: "10 02".into(),
            uds_sa: 0x00,
            uds_log: Vec::new(),
            leak_sel_idx: 0,
            leak_manual: LeakManualUi::default(),
            leak_custom: LeakCustomUi::default(),
            leak_horizon_s: 600.0,
            leak_scenario_dt: 0.05,
            leak_predictions: Vec::new(),
            leak_note: String::new(),
            pl_rpm: vec![],
            pl_spd: vec![],
            pl_torque: vec![],
            pl_fuel: vec![],
            pl_coolant: vec![],
            pl_dpf: vec![],
            pl_boost: vec![],
            ticks: 0,
        }
    }

    fn oil_type_from_idx(idx: usize) -> OilType {
        match idx {
            0 => OilType::HydraulicIso46,
            1 => OilType::HydraulicIso68,
            2 => OilType::Engine15w40,
            3 => OilType::Engine10w30,
            4 => OilType::Pag46,
            5 => OilType::Poe68,
            _ => OilType::Custom,
        }
    }

    fn component_from_idx(idx: usize) -> CircuitComponent {
        match idx {
            0 => CircuitComponent::Oring,
            1 => CircuitComponent::Seal,
            _ => CircuitComponent::AcHose,
        }
    }

    fn material_from_idx(idx: usize) -> OringMaterial {
        match idx {
            0 => OringMaterial::Nbr,
            1 => OringMaterial::Hnbr,
            2 => OringMaterial::Fkm,
            _ => OringMaterial::Epdm,
        }
    }

    fn idx_from_oil_type(o: OilType) -> usize {
        match o {
            OilType::HydraulicIso46 => 0,
            OilType::HydraulicIso68 => 1,
            OilType::Engine15w40 => 2,
            OilType::Engine10w30 => 3,
            OilType::Pag46 => 4,
            OilType::Poe68 => 5,
            OilType::Custom => 6,
        }
    }

    fn sync_leak_manual_from_selected(&mut self) {
        if let Some(c) = self.bench.leak_rig.circuits.get(self.leak_sel_idx) {
            self.leak_manual.oil_type_idx = Self::idx_from_oil_type(c.oil_type);
            self.leak_manual.piston_pressure_bar = c.piston_pressure_bar;
            self.leak_manual.operation_pressure_bar = c.operation_pressure_bar;
            self.leak_manual.pressure_min_bar = c.pressure.min_bar;
            self.leak_manual.pressure_mean_bar = c.pressure.mean_bar;
            self.leak_manual.pressure_ideal_bar = c.pressure.ideal_bar;
            self.leak_manual.pressure_max_bar = c.pressure.max_bar;
            self.leak_manual.pressure_rupture_bar = c.pressure.rupture_bar;
            self.leak_manual.squeeze_pct = c.spec.squeeze_pct;
            self.leak_manual.compression_set_pct = c.spec.compression_set_pct;
            self.leak_manual.base_leak_area_mm2 = c.spec.base_leak_area_mm2;
        }
    }

    fn can_bus_from_idx(idx: usize) -> auto_breaking::BusId {
        match idx {
            0 => auto_breaking::BusId::PowertrainHs,
            1 => auto_breaking::BusId::ChassisHs,
            2 => auto_breaking::BusId::BodyMs,
            3 => auto_breaking::BusId::IsoBus,
            _ => auto_breaking::BusId::Diagnostic,
        }
    }

    // ─ Execute all queued commands ─────────────────────────────────────────
    fn flush_cmds(&mut self) {
        let cmds: Vec<Cmd> = self.cmds.drain(..).collect();
        for cmd in cmds {
            match cmd {
                Cmd::KeyAdvance => self.bench.key_advance(),
                Cmd::KeyOff => {
                    self.bench.key_off();
                    self.throttle = 0.0;
                    self.brake = 0.0;
                }
                Cmd::SetThrottle(v) => {
                    self.throttle = v;
                    self.bench.throttle_pct = v as f64 * 100.0;
                }
                Cmd::SetBrake(v) => {
                    self.brake = v;
                    self.bench.brake_pct = v as f64 * 100.0;
                }
                Cmd::SetDirection(d) => self.bench.tcm.set_direction(d),
                Cmd::SetNeutral => self.bench.tcm.set_neutral(),
                Cmd::ToggleAutoShift => self.bench.tcm.toggle_auto(),
                Cmd::ToggleCreeper => self.bench.tcm.toggle_creeper(),
                Cmd::ManualUp => self.bench.tcm.manual_upshift(),
                Cmd::ManualDn => self.bench.tcm.manual_downshift(),
                // ECM overrides
                Cmd::SetFuelLevel(v) => {
                    self.bench.ecm.fuel_level_pct = v.clamp(0.0, 100.0);
                    self.p_fuel = v;
                }
                Cmd::SetDefLevel(v) => {
                    self.bench.ecm.def_level_pct = v.clamp(0.0, 100.0);
                    self.p_def = v;
                }
                Cmd::SetCoolantTemp(v) => {
                    self.bench.ecm.coolant_temp_c = v.clamp(-40.0, 130.0);
                    self.p_coolant = v;
                }
                Cmd::SetOilPressure(v) => {
                    self.bench.ecm.oil_pressure_kpa = v.clamp(0.0, 800.0);
                    self.p_oil_prs = v;
                }
                Cmd::SetBoostPressure(v) => {
                    self.bench.ecm.boost_pressure_kpa = v.clamp(0.0, 350.0);
                    self.p_boost = v;
                }
                Cmd::SetEngineHours(v) => {
                    self.bench.ecm.engine_hours = v.max(0.0);
                    self.p_engine_h = v;
                }
                Cmd::ClearDtcs => self.bench.clear_faults(),
                // BCM
                Cmd::ToggleWorkLights => self.bench.bcm.toggle_work_lights(),
                Cmd::ToggleRoadLights => self.bench.bcm.toggle_road_lights(),
                Cmd::ToggleBeacon => self.bench.bcm.toggle_beacon(),
                Cmd::Honk => self.bench.bcm.honk(),
                Cmd::CycleWiper => self.bench.bcm.cycle_wiper(),
                // Implements
                Cmd::SetPtoMode(m) => {
                    self.bench.implement.pto_rear_enabled = m != PtoMode::Off;
                    self.bench.implement.pto_mode = m;
                }
                Cmd::SetHitchTarget(v) => {
                    self.bench.implement.hitch_target_pct = v.clamp(0.0, 100.0);
                    self.hitch_target = v;
                }
                Cmd::SetAuxValve(i, v) => {
                    if i < 4 {
                        self.bench.hcm.set_aux_cmd(i, v);
                        self.bench.implement.aux_banks[i].direction = if v > 0.1 {
                            auto_breaking::implement::ValveDirection::Extend
                        } else if v < -0.1 {
                            auto_breaking::implement::ValveDirection::Retract
                        } else {
                            auto_breaking::implement::ValveDirection::Neutral
                        };
                        self.bench.implement.aux_banks[i].engaged = v.abs() > 0.05;
                        self.aux_cmds[i] = v;
                    }
                }
                Cmd::SetLoaderLift(v) => {
                    self.bench.loader_lift_cmd = v;
                }
                Cmd::SetLoaderTilt(v) => {
                    self.bench.loader_tilt_cmd = v;
                }
                Cmd::SetHitchJoystick(v) => {
                    self.bench.hitch_joystick = v;
                }
                // AD
                Cmd::EngageAD => self.bench.ad.engage(self.bench.tcm.ground_speed_kmh),
                Cmd::DisengageAD => self.bench.ad.disengage(),
                Cmd::ToggleLka => self.bench.ad.toggle_lka(),
                Cmd::AccSpeedSet(v) => self.bench.ad.set_acc_speed(v),
                Cmd::AccHeadwaySet(v) => self.bench.ad.set_headway(v),
                Cmd::AddWaypoint(x, y) => self.ad_waypoints.push([x, y]),
                Cmd::ClearWaypoints => self.ad_waypoints.clear(),
                // Faults
                Cmd::SelectFault(i) => {
                    self.fault_idx = i.min(FAULT_TYPES.len().saturating_sub(1));
                    self.bench.selected_fault = FAULT_TYPES[self.fault_idx];
                }
                Cmd::InjectFault => self.bench.inject_fault(),
                Cmd::ClearFaults => self.bench.clear_faults(),
                // Telematics
                Cmd::StartOta => self.bench.telematics.start_ota(),
                // Leak Lab
                Cmd::LeakSelectCircuit(i) => {
                    self.leak_sel_idx = i.min(self.bench.leak_rig.circuits.len().saturating_sub(1));
                    self.sync_leak_manual_from_selected();
                }
                Cmd::LeakApplyManual => {
                    if let Some(c) = self.bench.leak_rig.circuits.get(self.leak_sel_idx) {
                        let name = c.name.clone();
                        let params = ManualCircuitParams {
                            oil_type: Some(Self::oil_type_from_idx(self.leak_manual.oil_type_idx)),
                            piston_pressure_bar: Some(self.leak_manual.piston_pressure_bar),
                            operation_pressure_bar: Some(self.leak_manual.operation_pressure_bar),
                            pressure_min_bar: Some(self.leak_manual.pressure_min_bar),
                            pressure_mean_bar: Some(self.leak_manual.pressure_mean_bar),
                            pressure_ideal_bar: Some(self.leak_manual.pressure_ideal_bar),
                            pressure_max_bar: Some(self.leak_manual.pressure_max_bar),
                            pressure_rupture_bar: Some(self.leak_manual.pressure_rupture_bar),
                            oring_squeeze_pct: Some(self.leak_manual.squeeze_pct),
                            compression_set_pct: Some(self.leak_manual.compression_set_pct),
                            base_leak_area_mm2: Some(self.leak_manual.base_leak_area_mm2),
                            max_supported_temp_c: None,
                        };
                        let ok = self.bench.apply_leak_manual_params(&name, params);
                        self.leak_note = if ok {
                            "Parametros aplicados com sucesso".into()
                        } else {
                            "Falha ao aplicar parametros".into()
                        };
                    }
                }
                Cmd::LeakPredictScenarios => {
                    self.leak_predictions = self.bench.predict_leak_scenarios(
                        self.leak_horizon_s,
                        self.leak_scenario_dt.max(0.01),
                    );
                    self.leak_note = format!("{} cenarios avaliados", self.leak_predictions.len());
                }
                Cmd::LeakAddCustomCircuit => {
                    let c = LeakCircuit::new(
                        &self.leak_custom.name,
                        &self.leak_custom.application,
                        Self::component_from_idx(self.leak_custom.component_idx),
                        Self::oil_type_from_idx(self.leak_custom.oil_type_idx),
                        self.leak_custom.seal_count.max(1),
                        self.leak_custom.piston_pressure_bar,
                        self.leak_custom.operation_pressure_bar,
                        PressureEnvelope {
                            min_bar: self.leak_custom.min_bar,
                            mean_bar: self.leak_custom.mean_bar,
                            ideal_bar: self.leak_custom.ideal_bar,
                            max_bar: self.leak_custom.max_bar,
                            rupture_bar: self
                                .leak_custom
                                .rupture_bar
                                .max(self.leak_custom.max_bar + 0.1),
                        },
                        OringSpec {
                            id_tag: format!("{}-custom", self.leak_custom.name),
                            material: Self::material_from_idx(self.leak_custom.material_idx),
                            shore_a: self.leak_custom.shore_a,
                            cross_section_mm: self.leak_custom.cross_section_mm,
                            squeeze_pct: self.leak_custom.squeeze_pct,
                            extrusion_gap_mm: self.leak_custom.extrusion_gap_mm,
                            compression_set_pct: self.leak_custom.compression_set_pct,
                            design_life_hours: self.leak_custom.design_life_hours,
                            base_leak_area_mm2: self.leak_custom.base_leak_area_mm2,
                            discharge_coeff: self.leak_custom.discharge_coeff,
                        },
                        self.leak_custom.reservoir_volume_l,
                        self.leak_custom.support_lpm,
                    );
                    self.bench.add_custom_leak_circuit(c);
                    self.leak_sel_idx = self.bench.leak_rig.circuits.len().saturating_sub(1);
                    self.sync_leak_manual_from_selected();
                    self.leak_note = "Circuito custom adicionado".into();
                }
                Cmd::LeakExportReportCsv => {
                    let ts = Local::now().format("%Y%m%d_%H%M%S");
                    let path = format!("reports/leak_report_{}.csv", ts);
                    self.leak_note = match self.bench.export_leak_report_csv(&path) {
                        Ok(_) => format!("CSV salvo em {}", path),
                        Err(e) => format!("Falha export CSV: {}", e),
                    };
                }
                Cmd::LeakExportReportJson => {
                    let ts = Local::now().format("%Y%m%d_%H%M%S");
                    let path = format!("reports/leak_report_{}.json", ts);
                    self.leak_note = match self.bench.export_leak_report_json(&path) {
                        Ok(_) => format!("JSON salvo em {}", path),
                        Err(e) => format!("Falha export JSON: {}", e),
                    };
                }
                Cmd::LeakExportPredCsv => {
                    let ts = Local::now().format("%Y%m%d_%H%M%S");
                    let path = format!("reports/leak_predictions_{}.csv", ts);
                    self.leak_note = match self
                        .bench
                        .export_leak_predictions_csv(&path, &self.leak_predictions)
                    {
                        Ok(_) => format!("Pred CSV salvo em {}", path),
                        Err(e) => format!("Falha export pred CSV: {}", e),
                    };
                }
                Cmd::LeakExportPredJson => {
                    let ts = Local::now().format("%Y%m%d_%H%M%S");
                    let path = format!("reports/leak_predictions_{}.json", ts);
                    self.leak_note = match self
                        .bench
                        .export_leak_predictions_json(&path, &self.leak_predictions)
                    {
                        Ok(_) => format!("Pred JSON salvo em {}", path),
                        Err(e) => format!("Falha export pred JSON: {}", e),
                    };
                }
                Cmd::LeakRunMonteCarlo => {
                    self.leak_predictions = self.bench.monte_carlo_leak_predictions(
                        120,
                        self.leak_horizon_s,
                        self.leak_scenario_dt.max(0.01),
                        25.0,
                    );
                    self.leak_note = format!(
                        "Monte Carlo concluido: {} amostras",
                        self.leak_predictions.len()
                    );
                }
                Cmd::CanInjectBitError(idx) => {
                    let bus = Self::can_bus_from_idx(idx);
                    self.bench
                        .can_net
                        .inject_error_once(bus, auto_breaking::CanErrorKind::Bit);
                    self.can_note = format!("Bit error armado em {}", bus);
                }
                Cmd::CanInjectAckError(idx) => {
                    let bus = Self::can_bus_from_idx(idx);
                    self.bench
                        .can_net
                        .inject_error_once(bus, auto_breaking::CanErrorKind::Ack);
                    self.can_note = format!("ACK error armado em {}", bus);
                }
                Cmd::CanInjectBusOff(idx) => {
                    let bus = Self::can_bus_from_idx(idx);
                    self.bench.can_net.inject_bus_off_once(bus);
                    self.can_note = format!("Bus-Off armado em {}", bus);
                }
                Cmd::CanInjectBabbling(idx) => {
                    let bus = Self::can_bus_from_idx(idx);
                    self.bench
                        .can_net
                        .inject_error_once(bus, auto_breaking::CanErrorKind::BablingIdiot);
                    self.can_note = format!("Babbling armado em {}", bus);
                }
                Cmd::CanClearBusInjections(idx) => {
                    let bus = Self::can_bus_from_idx(idx);
                    self.bench.can_net.clear_injections(Some(bus));
                    self.can_note = format!("Injecoes limpas em {}", bus);
                }
                Cmd::CanClearAllInjections => {
                    self.bench.can_net.clear_injections(None);
                    self.can_note = "Injecoes limpas em todos os barramentos".into();
                }
                Cmd::CanExportSnapshotCsv => {
                    let ts = Local::now().format("%Y%m%d_%H%M%S");
                    let path = format!("reports/can_network_snapshot_{}.csv", ts);
                    self.can_note = match self.bench.export_can_snapshot_csv(&path) {
                        Ok(_) => format!("Snapshot CSV salvo em {}", path),
                        Err(e) => format!("Falha snapshot CSV: {}", e),
                    };
                }
                Cmd::CanExportSnapshotJson => {
                    let ts = Local::now().format("%Y%m%d_%H%M%S");
                    let path = format!("reports/can_network_snapshot_{}.json", ts);
                    self.can_note = match self.bench.export_can_snapshot_json(&path) {
                        Ok(_) => format!("Snapshot JSON salvo em {}", path),
                        Err(e) => format!("Falha snapshot JSON: {}", e),
                    };
                }
                // Simulation
                Cmd::Reset => {
                    self.bench.reset();
                    self.throttle = 0.0;
                    self.brake = 0.0;
                    self.sig_map.clear();
                    self.events.clear();
                    self.ticks = 0;
                    for v in [
                        &mut self.pl_rpm,
                        &mut self.pl_spd,
                        &mut self.pl_torque,
                        &mut self.pl_fuel,
                        &mut self.pl_coolant,
                        &mut self.pl_dpf,
                        &mut self.pl_boost,
                    ] {
                        v.clear();
                    }
                    self.uds_log.clear();
                    self.ad_waypoints.clear();
                    self.leak_predictions.clear();
                    self.leak_note.clear();
                    self.can_note.clear();
                }
                Cmd::Pause | Cmd::Resume => {} // UI-only, no bench mutation needed
            }
        }
    }

    // ─ Sync parameter panel from simulation (when not dirty) ──────────────
    fn sync_params_from_bench(&mut self) {
        if !self.params_dirty {
            self.p_fuel = self.bench.ecm.fuel_level_pct;
            self.p_def = self.bench.ecm.def_level_pct;
            self.p_coolant = self.bench.ecm.coolant_temp_c;
            self.p_oil_prs = self.bench.ecm.oil_pressure_kpa;
            self.p_boost = self.bench.ecm.boost_pressure_kpa;
            self.p_engine_h = self.bench.ecm.engine_hours;
        }
    }

    // ─ Update CAN signal map ─────────────────────────────────────────────
    fn update_signals(&mut self) {
        let t = self.bench.elapsed;
        if !self.can_freeze {
            for frame in self.bench.gateway.dispatched.clone() {
                let e = self.sig_map.entry((frame.pgn, frame.sa)).or_insert(Signal {
                    pgn_name: frame.pgn_name(),
                    sa_name: frame.sa_name().into(),
                    raw_id: frame.raw_id,
                    last_ts: t,
                    count: 0,
                    period_ms: 0.0,
                    decoded: Vec::new(),
                    fresh: true,
                });
                let dt_ms = (t - e.last_ts) * 1000.0;
                if e.count > 0 {
                    e.period_ms = e.period_ms * 0.85 + dt_ms * 0.15;
                }
                e.last_ts = t;
                e.count += 1;
                e.fresh = true;
                e.decoded = frame
                    .decoded
                    .iter()
                    .map(|v| (v.name.to_string(), v.physical, v.unit.to_string()))
                    .collect();
            }
        }
        for sig in self.sig_map.values_mut() {
            if t - sig.last_ts > 0.25 {
                sig.fresh = false;
            }
        }
        if !self.can_freeze {
            self.trace_snap = self
                .bench
                .gateway
                .bus
                .frames
                .iter()
                .filter(|f| {
                    let filt = &self.can_filter;
                    filt.is_empty()
                        || f.pgn_name().to_lowercase().contains(filt.as_str())
                        || f.sa_name().to_lowercase().contains(filt.as_str())
                })
                .map(|f| {
                    (
                        f.timestamp,
                        f.raw_id,
                        f.sa,
                        f.dlc,
                        f.data_hex(),
                        format!("{}[{}]", f.pgn_name(), f.sa_name()),
                        f.decoded
                            .iter()
                            .take(3)
                            .map(|v| format!("{:.1}{}", v.physical, v.unit))
                            .collect::<Vec<_>>()
                            .join(" "),
                    )
                })
                .collect();
        }
    }

    // ─ Generate meaningful events ─────────────────────────────────────────
    fn detect_events(&mut self) {
        if self.ev_pause {
            return;
        }
        let t = self.bench.elapsed;
        let ecm = &self.bench.ecm;
        let abs = &self.bench.abs;
        let ign = self.bench.ignition();

        macro_rules! ev {
            ($src:expr,$msg:expr,$lvl:expr) => {
                if self.events.len() < EVENT_MAX {
                    self.events.push(AppEvent {
                        ts: t,
                        source: $src,
                        msg: $msg.into(),
                        lvl: $lvl,
                    });
                }
            };
        }

        // Ignition state
        if ign != self.prev_ign {
            ev!(
                "IGN",
                format!(
                    "Key → {:?}  [{}]",
                    ign,
                    self.bench
                        .boot
                        .crank_inhibited
                        .then_some("INHIBITED")
                        .unwrap_or("OK")
                ),
                EventLevel::Info
            );
            self.prev_ign = ign;
        }
        // Engine start/stop
        if ecm.rpm > 400.0 && self.prev_rpm <= 400.0 {
            ev!(
                "ECM",
                format!("ENGINE STARTED  RPM={:.0}  T={:.1}s", ecm.rpm, t),
                EventLevel::Ok
            );
        }
        if ecm.rpm <= 400.0 && self.prev_rpm > 400.0 {
            ev!("ECM", "ENGINE STOPPED", EventLevel::Warn);
        }
        self.prev_rpm = ecm.rpm;
        // Gear change
        let gear = self.bench.tcm.gear_label.clone();
        if gear != self.prev_gear {
            ev!(
                "TCM",
                format!(
                    "GEAR  {} → {}  (RPM {:.0}  Spd {:.1}km/h)",
                    self.prev_gear, gear, ecm.rpm, self.bench.tcm.ground_speed_kmh
                ),
                EventLevel::Info
            );
            self.prev_gear = gear;
        }
        // DTC set/clear
        let dtcs = ecm.active_dtcs.len();
        if dtcs > self.prev_dtcs {
            if let Some(d) = ecm.active_dtcs.last() {
                let severity = format!("{}", d.severity);
                ev!(
                    "ECM",
                    format!(
                        "DTC SET  SPN:{:5} FMI:{:2}  [{:4}]  {}",
                        d.spn, d.fmi, severity, d.desc
                    ),
                    EventLevel::Critical
                );
            }
        } else if dtcs < self.prev_dtcs {
            ev!(
                "ECM",
                format!("DTCs CLEARED ({} were active)", self.prev_dtcs),
                EventLevel::Ok
            );
        }
        self.prev_dtcs = dtcs;
        // Warning lamps
        if ecm.red_lamp && self.prev_dtcs == 0 {
            ev!("ECM", "🔴 RED STOP LAMP ON", EventLevel::Critical);
        }
        if ecm.amber_lamp && t > 1.0 { /* only on transition */ }
        // ABS
        if abs.abs_system_active && !self.prev_abs {
            ev!(
                "ABS",
                format!(
                    "ABS ACTIVATED  Speed={:.1}km/h  Brake={:.0}%",
                    self.bench.tcm.ground_speed_kmh, self.bench.brake_pct
                ),
                EventLevel::Warn
            );
        } else if !abs.abs_system_active && self.prev_abs {
            ev!("ABS", "ABS DEACTIVATED — slip recovered", EventLevel::Ok);
        }
        self.prev_abs = abs.abs_system_active;
        // ESP
        if abs.esp_condition != self.prev_esp {
            if abs.esp_condition != EspCondition::Neutral {
                ev!(
                    "ESP",
                    format!(
                        "ESP INTERVENTION: {:?}  Yaw-err={:+.1}°/s",
                        abs.esp_condition, abs.esp_yaw_error
                    ),
                    EventLevel::Warn
                );
            } else {
                ev!("ESP", "ESP STABLE", EventLevel::Ok);
            }
            self.prev_esp = abs.esp_condition;
        }
        // DPF regen
        let regen = self.bench.ecm.regen_requested;
        if regen && !self.prev_regen {
            ev!(
                "DPF",
                format!(
                    "DPF REGEN STARTED  Soot={:.0}%  Exhaust={:.0}°C",
                    ecm.dpf_soot_pct, ecm.exhaust_temp_c
                ),
                EventLevel::Warn
            );
        } else if !regen && self.prev_regen {
            ev!(
                "DPF",
                format!("DPF REGEN COMPLETE  Soot={:.0}%", ecm.dpf_soot_pct),
                EventLevel::Ok
            );
        }
        self.prev_regen = regen;
        // Coolant overheat
        if ecm.coolant_temp_c > 102.0 && self.prev_rpm > 400.0 {
            if t as u32 % 5 == 0 {
                // throttle event rate
                ev!(
                    "ECM",
                    format!("HIGH COOLANT TEMP {:.1}°C", ecm.coolant_temp_c),
                    EventLevel::Critical
                );
            }
        }
        // DEF warning
        if ecm.def_level_pct < 10.0 && ecm.def_level_pct > 0.0 {
            if t as u32 % 30 == 0 {
                ev!(
                    "SCR",
                    format!(
                        "DEF LOW {:.1}%  NOx={:.0}ppm",
                        ecm.def_level_pct, ecm.nox_tailpipe_ppm
                    ),
                    EventLevel::Warn
                );
            }
        }
        // Boot events
        let mode = format!("{}", self.bench.vcm.mode);
        if mode != self.prev_mode {
            ev!("VCM", format!("Vehicle mode → {}", mode), EventLevel::Info);
            self.prev_mode = mode;
        }
        // NM state
        let nm_state = format!("{}", self.bench.net_mgmt.bus_state);
        let _ = nm_state; // could add NM events
                          // TCS
        if abs.tcs_system_active {
            if t as u32 % 2 == 0 {
                ev!(
                    "TCS",
                    format!(
                        "TCS cut={:.0}%  Speed={:.1}km/h",
                        abs.tcs_throttle_cut * 100.0,
                        self.bench.tcm.ground_speed_kmh
                    ),
                    EventLevel::Warn
                );
            }
        }
    }

    fn push_plots(&mut self) {
        let t = self.bench.elapsed;
        macro_rules! p {
            ($v:expr,$x:expr) => {
                $v.push([t, $x]);
                if $v.len() > PLOT_WINDOW {
                    $v.remove(0);
                }
            };
        }
        p!(self.pl_rpm, self.bench.ecm.rpm);
        p!(self.pl_spd, self.bench.tcm.ground_speed_kmh);
        p!(self.pl_torque, self.bench.ecm.actual_torque_nm);
        p!(self.pl_fuel, self.bench.ecm.fuel_rate_lph);
        p!(self.pl_coolant, self.bench.ecm.coolant_temp_c);
        p!(self.pl_dpf, self.bench.ecm.dpf_soot_pct);
        p!(self.pl_boost, self.bench.ecm.boost_pressure_kpa);
    }

    fn auto_sequence(&mut self) {
        match self.ticks {
            60 => self.cmds.push(Cmd::KeyAdvance),
            120 => self.cmds.push(Cmd::KeyAdvance),
            180 => self.cmds.push(Cmd::KeyAdvance),
            _ => {}
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Entry point
// ═════════════════════════════════════════════════════════════════════════════
fn main() -> eframe::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let hw_cfg = match io::hw::HwConfig::from_cli_args(&args) {
        Ok(cfg) => cfg,
        Err(e) => {
            eprintln!("[hw-config] invalid arguments, using defaults: {}", e);
            io::hw::HwConfig::default()
        }
    };

    match io::hw::write_listen_only_startup_log(&hw_cfg) {
        Ok(path) => {
            println!(
                "[hw-audit] mode={:?} dry_run={} allowlist_present={} write_enabled={} log={}",
                hw_cfg.mode,
                hw_cfg.dry_run,
                hw_cfg.allowlist_path.is_some(),
                hw_cfg.write_effectively_enabled(),
                path.display()
            );
        }
        Err(e) => {
            eprintln!("[hw-audit] failed to write startup audit: {}", e);
        }
    }

    #[cfg(feature = "advanced_observability")]
    {
        let _ = tracing_subscriber::fmt()
            .json()
            .with_current_span(false)
            .with_target(false)
            .try_init();
    }

    eframe::run_native(
        "🚜 Heavy Machinery ECU Bench v3.0",
        NativeOptions {
            viewport: ViewportBuilder::default()
                .with_inner_size([1700.0, 1020.0])
                .with_min_inner_size([1200.0, 700.0]),
            ..Default::default()
        },
        Box::new(move |cc| Ok(Box::new(App::new(cc, hw_cfg.clone())))),
    )
}

// ═════════════════════════════════════════════════════════════════════════════
// Main loop
// ═════════════════════════════════════════════════════════════════════════════
impl eframe::App for App {
    fn update(&mut self, ctx: &Context, _: &mut eframe::Frame) {
        // ── Execute deferred commands ──────────────────────────────────────
        self.flush_cmds();

        // ── Simulation tick ────────────────────────────────────────────────
        self.bench.throttle_pct = self.throttle as f64 * 100.0;
        self.bench.brake_pct = self.brake as f64 * 100.0;
        self.auto_sequence();
        self.bench.tick(DT);
        self.ticks += 1;
        self.update_signals();
        self.detect_events();
        self.push_plots();
        self.sync_params_from_bench();
        if let Some(feed) = &self.ecm_live_feed {
            self.ecm_live_snapshot = feed.latest_snapshot();
            self.ecm_live_last_update_ms = feed.last_update_ms();
            self.ecm_live_frames_total = feed.frames_total();
        }
        ctx.request_repaint();

        // ── Panels ──────────────────────────────────────────────────────────
        TopBottomPanel::top("tb")
            .min_height(56.0)
            .show(ctx, |ui| self.toolbar(ui));
        TopBottomPanel::top("tabs")
            .exact_height(30.0)
            .show(ctx, |ui| self.tab_bar(ui));
        TopBottomPanel::bottom("sb")
            .exact_height(22.0)
            .show(ctx, |ui| self.statusbar(ui));
        CentralPanel::default().show(ctx, |ui| match self.tab {
            Tab::Cluster => self.tab_cluster(ui),
            Tab::CanBus => self.tab_can(ui),
            Tab::Events => self.tab_events(ui),
            Tab::EcuNet => self.tab_ecu_net(ui),
            Tab::Engine => self.tab_engine(ui),
            Tab::Faults => self.tab_faults(ui),
            Tab::Boot => self.tab_boot(ui),
            Tab::Implements => self.tab_implements(ui),
            Tab::Params => self.tab_params(ui),
            Tab::Sensors => self.tab_sensors(ui),
            Tab::Autonomous => self.tab_autonomous(ui),
            Tab::V2X => self.tab_v2x(ui),
            Tab::Uds => self.tab_uds(ui),
            Tab::EcmLiveData => self.tab_ecm_live_data(ui),
            Tab::LeakLab => self.tab_leak_lab(ui),
            Tab::Plots => self.tab_plots(ui),
        });
    }
}

// ── Tab bar ──────────────────────────────────────────────────────────────────
impl App {
    fn tab_bar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            let dtcs = self.bench.ecm.active_dtcs.len();
            let tabs = [
                (Tab::Cluster, "🎛 CLUSTER"),
                (Tab::CanBus, "📡 CAN BUS"),
                (Tab::Events, "📋 EVENTS"),
                (Tab::EcuNet, "🔌 ECU NET"),
                (Tab::Engine, "⚙ ENGINE"),
                (Tab::Faults, "⚠ FAULTS"),
                (Tab::Boot, "🔑 BOOT"),
                (Tab::Implements, "🌾 IMPL"),
                (Tab::Params, "🎚 PARAMS"),
                (Tab::Sensors, "🛰 SENSORS"),
                (Tab::Autonomous, "🤖 AD"),
                (Tab::V2X, "📶 V2X"),
                (Tab::Uds, "🔧 UDS"),
                (Tab::EcmLiveData, "🧲 ECM LIVE"),
                (Tab::LeakLab, "🧪 LEAK LAB"),
                (Tab::Plots, "📈 PLOTS"),
            ];
            for (t, lbl) in tabs {
                let sel = self.tab == t;
                let fill = if sel {
                    Color32::from_rgb(45, 105, 210)
                } else {
                    Color32::from_gray(30)
                };
                let label = if t == Tab::Faults && dtcs > 0 {
                    format!("⚠ FAULTS({})", dtcs)
                } else {
                    lbl.into()
                };
                let col = if t == Tab::Faults && dtcs > 0 {
                    Color32::RED
                } else if sel {
                    Color32::WHITE
                } else {
                    Color32::from_gray(160)
                };
                if ui
                    .add(Button::new(RichText::new(label).size(11.0).color(col)).fill(fill))
                    .clicked()
                {
                    self.tab = t;
                }
            }
        });
    }

    fn statusbar(&self, ui: &mut Ui) {
        let t = self.bench.elapsed;
        let rpm = self.bench.ecm.rpm;
        let spd = self.bench.tcm.ground_speed_kmh;
        let gear = &self.bench.tcm.gear_label;
        let can = self.bench.gateway.bus_load_pct;
        let dtcs = self.bench.ecm.active_dtcs.len();
        let nm = format!("{}", self.bench.net_mgmt.bus_state);
        let red = self.bench.ecm.red_lamp;
        let amber = self.bench.ecm.amber_lamp;
        let abs_on = self.bench.abs.abs_system_active;
        let esp_on = self.bench.abs.esp_system_active;
        ui.horizontal(|ui| {
            macro_rules! s {
                ($v:expr,$c:expr) => {
                    ui.label(RichText::new($v).size(11.0).color($c));
                };
            }
            s!(format!("t={:.1}s", t), Color32::from_gray(120));
            ui.separator();
            s!(format!("RPM {:.0}", rpm), Color32::GREEN);
            ui.separator();
            s!(format!("{:.1}km/h", spd), Color32::LIGHT_BLUE);
            ui.separator();
            s!(format!("Gear {}", gear), Color32::YELLOW);
            ui.separator();
            s!(format!("CAN {:.0}%", can), Color32::GOLD);
            ui.separator();
            s!(
                format!("{} DTC", dtcs),
                if dtcs == 0 {
                    Color32::from_gray(100)
                } else {
                    Color32::RED
                }
            );
            ui.separator();
            s!(format!("NM:{}", nm), Color32::from_gray(120));
            if red {
                ui.separator();
                s!("🔴 RED STOP", Color32::RED);
            }
            if amber {
                ui.separator();
                s!("🟡 AMBER", Color32::GOLD);
            }
            if abs_on {
                ui.separator();
                s!("■ ABS", Color32::WHITE);
            }
            if esp_on {
                ui.separator();
                s!("■ ESP", Color32::LIGHT_BLUE);
            }
            if self.bench.ad.engaged {
                ui.separator();
                s!("🤖 AD ON", Color32::GREEN);
            }
        });
    }

    fn tab_ecm_live_data(&mut self, ui: &mut Ui) {
        ui.heading("ECM-Live Data");
        ui.add_space(6.0);

        if !matches!(self.hw_cfg.mode, io::hw::HwMode::Live) {
            ui.colored_label(
                Color32::YELLOW,
                "Live mode is disabled. Start with --hw-mode=live and either --vendor-name=cat_comm (Windows), --serial-port, or --can-if.",
            );
            return;
        }

        ui.horizontal(|ui| {
            if ui.button("Detect").clicked() {
                match io::live_runner::detect_ecms(&self.hw_cfg, Duration::from_secs(3)) {
                    Ok(res) => {
                        self.ecm_detected_sas = res.source_addresses;
                        self.ecm_selected_idx = 0;
                        self.ecm_connected = false;
                        if self.ecm_detected_sas.is_empty() {
                            self.ecm_live_status =
                                "Detect completed: no ECM found in timeout window.".into();
                        } else {
                            self.ecm_live_status = format!(
                                "Detect completed: {} ECM source address(es) found.",
                                self.ecm_detected_sas.len()
                            );
                        }
                    }
                    Err(e) => {
                        self.ecm_live_status = format!("Detect failed: {e}");
                    }
                }
            }

            let can_connect = !self.ecm_detected_sas.is_empty();
            if ui
                .add_enabled(can_connect, Button::new("Connect"))
                .clicked()
            {
                let selected_sa = self.ecm_detected_sas.get(self.ecm_selected_idx).copied();
                let target_sa = if self.hw_cfg.can_interface.is_some() {
                    selected_sa
                } else {
                    None
                };

                match io::live_runner::connect_ecm(&self.hw_cfg, target_sa) {
                    Ok(()) => {
                        self.ecm_connected = true;
                        self.ecm_live_status = "Connect successful. Ready to Retrieve Data.".into();
                    }
                    Err(e) => {
                        self.ecm_connected = false;
                        self.ecm_live_status = format!("Connect failed: {e}");
                    }
                }
            }

            let can_retrieve = self.ecm_connected && self.ecm_live_feed.is_none();
            if ui
                .add_enabled(can_retrieve, Button::new("Retrieve Data"))
                .clicked()
            {
                let selected_sa = self.ecm_detected_sas.get(self.ecm_selected_idx).copied();
                let target_sa = if self.hw_cfg.can_interface.is_some() {
                    selected_sa
                } else {
                    None
                };

                match io::live_runner::start_retrieve_data(self.hw_cfg.clone(), target_sa) {
                    Ok(feed) => {
                        self.ecm_live_feed = Some(feed);
                        self.ecm_live_status =
                            "Retrieve started. Real-time parameter updates are active.".into();
                    }
                    Err(e) => {
                        self.ecm_live_status = format!("Retrieve failed: {e}");
                    }
                }
            }

            if ui
                .add_enabled(self.ecm_live_feed.is_some(), Button::new("Stop"))
                .clicked()
            {
                if let Some(feed) = &self.ecm_live_feed {
                    feed.stop();
                }
                self.ecm_live_feed = None;
                self.ecm_live_status = "Retrieve stopped by operator.".into();
            }

            if ui
                .add_enabled(self.ecm_live_feed.is_some(), Button::new("Export CSV"))
                .clicked()
            {
                if let Some(feed) = &self.ecm_live_feed {
                    let ts = chrono::Local::now().format("%Y%m%d_%H%M%S").to_string();
                    let path = format!("reports/ecm_live_{}.csv", ts);
                    match feed.export_csv(&path) {
                        Ok(()) => {
                            self.ecm_live_status = format!("CSV exported to {}", path);
                        }
                        Err(e) => {
                            self.ecm_live_status = format!("CSV export failed: {e}");
                        }
                    }
                }
            }
        });

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.label("Detected ECM SA:");
            if self.ecm_detected_sas.is_empty() {
                ui.label("(none)");
            } else {
                let mut selected = self.ecm_selected_idx.min(self.ecm_detected_sas.len() - 1);
                ComboBox::from_id_salt("ecm_live_sa_combo")
                    .selected_text(format!("0x{:02X}", self.ecm_detected_sas[selected]))
                    .show_ui(ui, |ui| {
                        for (i, sa) in self.ecm_detected_sas.iter().enumerate() {
                            ui.selectable_value(&mut selected, i, format!("0x{:02X}", sa));
                        }
                    });
                self.ecm_selected_idx = selected;
            }
        });

        ui.add_space(8.0);
        ui.label(format!("Status: {}", self.ecm_live_status));
        ui.label(format!(
            "Connected: {} | Streaming: {} | Frames: {} | Last update(ms): {}",
            self.ecm_connected,
            self.ecm_live_feed.is_some(),
            self.ecm_live_frames_total,
            self.ecm_live_last_update_ms
        ));

        ui.separator();
        if let Some(feed) = &self.ecm_live_feed {
            let points = feed.recent_points(300);
            if !points.is_empty() {
                let mut rpm_min = f64::INFINITY;
                let mut rpm_max = f64::NEG_INFINITY;
                let mut rpm_sum = 0.0;
                let mut rpm_count = 0usize;
                let mut cool_max = f64::NEG_INFINITY;
                let mut oil_min = f64::INFINITY;

                for p in &points {
                    if let Some(r) = p.engine_speed_rpm {
                        rpm_min = rpm_min.min(r);
                        rpm_max = rpm_max.max(r);
                        rpm_sum += r;
                        rpm_count += 1;
                    }
                    if let Some(c) = p.coolant_temp_c {
                        cool_max = cool_max.max(c);
                    }
                    if let Some(o) = p.oil_pressure_kpa {
                        oil_min = oil_min.min(o);
                    }
                }

                ui.label("Post-analysis dashboard (rolling window)");
                Grid::new("ecm_live_dashboard_grid").show(ui, |ui| {
                    ui.label("Samples");
                    ui.label(format!("{}", points.len()));
                    ui.end_row();

                    ui.label("RPM Min");
                    ui.label(if rpm_count > 0 {
                        format!("{:.1}", rpm_min)
                    } else {
                        "-".to_string()
                    });
                    ui.end_row();

                    ui.label("RPM Avg");
                    ui.label(if rpm_count > 0 {
                        format!("{:.1}", rpm_sum / rpm_count as f64)
                    } else {
                        "-".to_string()
                    });
                    ui.end_row();

                    ui.label("RPM Max");
                    ui.label(if rpm_count > 0 {
                        format!("{:.1}", rpm_max)
                    } else {
                        "-".to_string()
                    });
                    ui.end_row();

                    ui.label("Coolant Peak C");
                    ui.label(if cool_max.is_finite() {
                        format!("{:.1}", cool_max)
                    } else {
                        "-".to_string()
                    });
                    ui.end_row();

                    ui.label("Oil Pressure Min kPa");
                    ui.label(if oil_min.is_finite() {
                        format!("{:.1}", oil_min)
                    } else {
                        "-".to_string()
                    });
                    ui.end_row();
                });
                ui.separator();
            }
        }

        ui.label("Real-time ECM Snapshot:");
        Grid::new("ecm_live_snapshot_grid").show(ui, |ui| {
            ui.label("Engine Speed");
            ui.label(
                self.ecm_live_snapshot
                    .engine_speed_rpm
                    .map_or("-".into(), |v| format!("{v:.1} rpm")),
            );
            ui.end_row();

            ui.label("Accel Pedal");
            ui.label(
                self.ecm_live_snapshot
                    .accel_pedal_pct
                    .map_or("-".into(), |v| format!("{v:.1} %")),
            );
            ui.end_row();

            ui.label("Coolant Temp");
            ui.label(
                self.ecm_live_snapshot
                    .coolant_temp_c
                    .map_or("-".into(), |v| format!("{v:.1} C")),
            );
            ui.end_row();

            ui.label("Fuel Temp");
            ui.label(
                self.ecm_live_snapshot
                    .fuel_temp_c
                    .map_or("-".into(), |v| format!("{v:.1} C")),
            );
            ui.end_row();

            ui.label("Oil Pressure");
            ui.label(
                self.ecm_live_snapshot
                    .oil_pressure_kpa
                    .map_or("-".into(), |v| format!("{v:.1} kPa")),
            );
            ui.end_row();

            ui.label("Last PGN");
            ui.label(
                self.ecm_live_snapshot
                    .last_seen_pgn
                    .map_or("-".into(), |v| format!("{v}")),
            );
            ui.end_row();

            ui.label("Source Address");
            ui.label(
                self.ecm_live_snapshot
                    .source_address
                    .map_or("-".into(), |v| format!("0x{v:02X}")),
            );
            ui.end_row();
        });
    }

    fn toolbar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.add_space(6.0);
            ui.label(
                RichText::new("🚜 ECU BENCH v3.0")
                    .size(17.0)
                    .color(Color32::from_rgb(80, 155, 255))
                    .strong(),
            );
            ui.separator();
            // IGN key
            ui.label(
                RichText::new("IGN")
                    .size(11.0)
                    .color(Color32::from_gray(150)),
            );
            let ign = self.bench.ignition();
            for (lbl, target) in [
                ("OFF", IgnitionState::Off),
                ("ACC", IgnitionState::Accessory),
                ("ON", IgnitionState::On),
                ("START", IgnitionState::Cranking),
            ] {
                let cur = ign == target;
                let col = if cur {
                    match target {
                        IgnitionState::Off => Color32::from_gray(110),
                        IgnitionState::Accessory => Color32::YELLOW,
                        IgnitionState::On => Color32::LIGHT_BLUE,
                        IgnitionState::Cranking => Color32::from_rgb(255, 130, 40),
                        IgnitionState::Running => Color32::GREEN,
                    }
                } else {
                    Color32::from_gray(40)
                };
                let btn = Button::new(RichText::new(lbl).size(11.0).color(if cur {
                    Color32::BLACK
                } else {
                    Color32::from_gray(155)
                }))
                .fill(col)
                .min_size(Vec2::new(48.0, 26.0));
                if ui.add(btn).clicked() {
                    match target {
                        IgnitionState::Off => self.cmds.push(Cmd::KeyOff),
                        _ => {
                            for _ in 0..5 {
                                if self.bench.ignition() == target {
                                    break;
                                }
                                self.cmds.push(Cmd::KeyAdvance);
                            }
                        }
                    }
                }
            }
            let run = self.bench.engine_running();
            ui.label(
                RichText::new(if run { "● RUN" } else { "○ OFF" })
                    .size(11.0)
                    .color(if run {
                        Color32::GREEN
                    } else {
                        Color32::from_gray(80)
                    }),
            );
            ui.separator();
            // Throttle
            ui.label(
                RichText::new("THROT")
                    .size(11.0)
                    .color(Color32::from_gray(145)),
            );
            let mut thr = self.throttle;
            ui.add_sized(
                [110.0, 22.0],
                Slider::new(&mut thr, 0.0..=1.0)
                    .show_value(false)
                    .trailing_fill(true),
            );
            if thr != self.throttle {
                self.cmds.push(Cmd::SetThrottle(thr));
            }
            ui.label(
                RichText::new(format!("{:3.0}%", self.throttle * 100.0))
                    .size(11.0)
                    .monospace()
                    .color(Color32::YELLOW),
            );
            ui.separator();
            // Brake
            ui.label(
                RichText::new("BRAKE")
                    .size(11.0)
                    .color(Color32::from_gray(145)),
            );
            let mut brk = self.brake;
            ui.add_sized(
                [80.0, 22.0],
                Slider::new(&mut brk, 0.0..=1.0)
                    .show_value(false)
                    .trailing_fill(true),
            );
            if brk != self.brake {
                self.cmds.push(Cmd::SetBrake(brk));
            }
            ui.label(
                RichText::new(format!("{:3.0}%", self.brake * 100.0))
                    .size(11.0)
                    .monospace()
                    .color(Color32::RED),
            );
            ui.separator();
            // Direction
            let dir_s = match self.bench.tcm.direction {
                Direction::Forward => "F",
                Direction::Reverse => "R",
                Direction::Neutral => "N",
                Direction::Park => "P",
            };
            if let Some(k) = direction_selector(ui, dir_s) {
                match k {
                    'F' => self.cmds.push(Cmd::SetDirection(Direction::Forward)),
                    'R' => self.cmds.push(Cmd::SetDirection(Direction::Reverse)),
                    'N' => self.cmds.push(Cmd::SetNeutral),
                    _ => {}
                }
            }
            ui.separator();
            // Auto-shift
            let auto = self.bench.tcm.auto_mode == AutoShiftMode::Auto;
            if ui
                .add(
                    Button::new(RichText::new(if auto { "AUTO✓" } else { "MANUAL" }).size(11.0))
                        .fill(if auto {
                            Color32::from_rgb(20, 85, 35)
                        } else {
                            Color32::from_gray(35)
                        }),
                )
                .clicked()
            {
                self.cmds.push(Cmd::ToggleAutoShift);
            }
            // PTO
            let pto = self.bench.implement.pto_rear_enabled;
            if ui
                .add(
                    Button::new(RichText::new(if pto { "PTO ON" } else { "PTO OFF" }).size(11.0))
                        .fill(if pto {
                            Color32::from_rgb(10, 95, 10)
                        } else {
                            Color32::from_gray(35)
                        }),
                )
                .clicked()
            {
                self.cmds.push(if pto {
                    Cmd::SetPtoMode(PtoMode::Off)
                } else {
                    Cmd::SetPtoMode(PtoMode::Std540)
                });
            }
            ui.separator();
            // Lights
            let wl = self.bench.bcm.work_lights_front;
            if ui
                .add(
                    Button::new(RichText::new(if wl { "💡 WRK" } else { "💡 off" }).size(11.0))
                        .fill(if wl {
                            Color32::from_rgb(85, 65, 8)
                        } else {
                            Color32::from_gray(32)
                        }),
                )
                .clicked()
            {
                self.cmds.push(Cmd::ToggleWorkLights);
            }
            ui.separator();
            // Pause / Reset
            if ui
                .add(Button::new(RichText::new("⏸ PAUSE").size(12.0)).fill(Color32::from_gray(42)))
                .clicked()
            { /* toggle handled via bench.paused would need lib support; just use ticks */
            }
            if ui
                .add(
                    Button::new(RichText::new("⟳ RESET").size(12.0))
                        .fill(Color32::from_rgb(70, 22, 22)),
                )
                .clicked()
            {
                self.cmds.push(Cmd::Reset);
            }
        });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB CLUSTER
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_cluster(&self, ui: &mut Ui) {
        let ecm = &self.bench.ecm;
        let tcm = &self.bench.tcm;
        let abs = &self.bench.abs;
        ui.horizontal(|ui| {
            // Left: big gauges + warning lamps
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                        arc_gauge(
                            ui, ecm.rpm, 0.0, 3000.0, "ENGINE", "RPM", 155.0, 2000.0, 2400.0, None,
                        )
                    });
                    egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                        arc_gauge(
                            ui,
                            tcm.ground_speed_kmh,
                            0.0,
                            50.0,
                            "SPEED",
                            "km/h",
                            155.0,
                            40.0,
                            48.0,
                            None,
                        )
                    });
                    egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                        arc_gauge(
                            ui,
                            ecm.coolant_temp_c,
                            0.0,
                            130.0,
                            "COOLANT",
                            "°C",
                            125.0,
                            100.0,
                            108.0,
                            None,
                        )
                    });
                    egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                        arc_gauge(
                            ui,
                            ecm.oil_pressure_kpa,
                            0.0,
                            700.0,
                            "OIL PRESS",
                            "kPa",
                            125.0,
                            0.0,
                            0.0,
                            Some(80.0),
                        )
                    });
                    egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                        arc_gauge(
                            ui,
                            ecm.boost_pressure_kpa,
                            0.0,
                            300.0,
                            "BOOST",
                            "kPa",
                            110.0,
                            250.0,
                            280.0,
                            None,
                        )
                    });
                });
                egui::Frame::group(ui.style())
                    .fill(Color32::from_gray(15))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new(" ⚠ WARNING CLUSTER")
                                .size(11.0)
                                .color(Color32::from_gray(130)),
                        );
                        ui.horizontal_wrapped(|ui| {
                            warning_lamp(ui, ecm.red_lamp, "RED STOP", Color32::RED);
                            warning_lamp(ui, ecm.amber_lamp, "AMBER", Color32::GOLD);
                            warning_lamp(ui, ecm.mil_active, "MIL", Color32::LIGHT_BLUE);
                            warning_lamp(
                                ui,
                                ecm.protect_lamp,
                                "PROTECT",
                                Color32::from_rgb(0, 210, 210),
                            );
                            warning_lamp(
                                ui,
                                ecm.oil_pressure_kpa < 100.0 && ecm.rpm > 500.0,
                                "OIL PRESS",
                                Color32::RED,
                            );
                            warning_lamp(
                                ui,
                                ecm.coolant_temp_c > 105.0,
                                "COOLANT HI",
                                Color32::RED,
                            );
                            warning_lamp(ui, ecm.def_level_pct < 10.0, "DEF LOW", Color32::YELLOW);
                            warning_lamp(
                                ui,
                                ecm.dpf_soot_pct > 75.0,
                                "DPF REGEN",
                                Color32::from_rgb(200, 80, 200),
                            );
                            warning_lamp(
                                ui,
                                ecm.fuel_level_pct < 10.0,
                                "FUEL LOW",
                                Color32::YELLOW,
                            );
                            warning_lamp(ui, abs.abs_system_active, "ABS", Color32::WHITE);
                            warning_lamp(ui, abs.esp_system_active, "ESP", Color32::LIGHT_BLUE);
                            warning_lamp(
                                ui,
                                abs.tcs_system_active,
                                "TCS",
                                Color32::from_rgb(80, 200, 255),
                            );
                            warning_lamp(
                                ui,
                                self.bench.bcm.work_lights_front,
                                "WORK LT",
                                Color32::from_rgb(255, 215, 0),
                            );
                            warning_lamp(
                                ui,
                                self.bench.implement.pto_rear_enabled,
                                "PTO",
                                Color32::GREEN,
                            );
                            warning_lamp(ui, self.bench.ad.engaged, "AD ON", Color32::GREEN);
                            warning_lamp(
                                ui,
                                self.bench.hcm.alarm_high_temp,
                                "HYD TEMP",
                                Color32::RED,
                            );
                            warning_lamp(
                                ui,
                                self.bench.hcm.filter_bypass_open,
                                "HYD FILT",
                                Color32::YELLOW,
                            );
                        });
                    });
                egui::Frame::group(ui.style())
                    .fill(Color32::from_gray(15))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("ENGINE")
                                .size(11.0)
                                .color(Color32::from_rgb(80, 155, 255)),
                        );
                        let w = 130.0;
                        bar_gauge(ui, "Load", ecm.percent_load, 150.0, "%", w, Color32::YELLOW);
                        bar_gauge(
                            ui,
                            "Torque",
                            ecm.actual_torque_nm,
                            1200.0,
                            "Nm",
                            w,
                            Color32::from_rgb(80, 180, 255),
                        );
                        bar_gauge(
                            ui,
                            "Power",
                            ecm.power_kw(),
                            260.0,
                            "kW",
                            w,
                            Color32::from_rgb(100, 220, 100),
                        );
                        bar_gauge(
                            ui,
                            "Rail P",
                            ecm.fuel_rail_pressure_mpa * 10.0,
                            2000.0,
                            "bar",
                            w,
                            Color32::from_rgb(0, 210, 210),
                        );
                        bar_gauge(
                            ui,
                            "Exhaust",
                            ecm.exhaust_temp_c,
                            900.0,
                            "°C",
                            w,
                            Color32::from_rgb(255, 120, 30),
                        );
                        bar_gauge(
                            ui,
                            "Fuel/h",
                            ecm.fuel_rate_lph,
                            30.0,
                            "L/h",
                            w,
                            Color32::YELLOW,
                        );
                        bar_gauge(
                            ui,
                            "DPF",
                            ecm.dpf_soot_pct,
                            100.0,
                            "%",
                            w,
                            if ecm.dpf_soot_pct > 75.0 {
                                Color32::RED
                            } else {
                                Color32::from_rgb(180, 100, 220)
                            },
                        );
                        bar_gauge(
                            ui,
                            "DEF",
                            ecm.def_level_pct,
                            100.0,
                            "%",
                            w,
                            if ecm.def_level_pct < 10.0 {
                                Color32::RED
                            } else {
                                Color32::from_rgb(0, 210, 210)
                            },
                        );
                    });
            });
            ui.separator();
            // Centre: transmission
            ui.vertical(|ui| {
                egui::Frame::group(ui.style())
                    .fill(Color32::from_gray(15))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("TRANSMISSION")
                                .size(11.0)
                                .color(Color32::from_rgb(80, 155, 255)),
                        );
                        let dc = match tcm.direction {
                            Direction::Forward => Color32::GREEN,
                            Direction::Reverse => Color32::RED,
                            Direction::Neutral => Color32::YELLOW,
                            Direction::Park => Color32::from_gray(180),
                        };
                        let dl = match tcm.direction {
                            Direction::Forward => "▶ FWD",
                            Direction::Reverse => "◀ REV",
                            Direction::Neutral => "■ NEU",
                            Direction::Park => "P",
                        };
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(&tcm.gear_label)
                                    .size(38.0)
                                    .color(Color32::from_rgb(80, 220, 80))
                                    .strong(),
                            );
                            ui.add_space(8.0);
                            ui.label(RichText::new(dl).size(22.0).color(dc).strong());
                        });
                        let am = match tcm.auto_mode {
                            AutoShiftMode::Auto => "AUTO",
                            AutoShiftMode::Manual => "MANUAL",
                            AutoShiftMode::Hold => "HOLD",
                        };
                        ui.horizontal(|ui| {
                            ui.label(RichText::new(am).size(12.0).color(Color32::YELLOW));
                            ui.separator();
                            ui.label(
                                RichText::new(format!("Rng {}", tcm.range))
                                    .size(12.0)
                                    .color(Color32::LIGHT_BLUE),
                            );
                            if tcm.creeper_engaged {
                                ui.separator();
                                ui.label(RichText::new("CREEP").size(11.0).color(Color32::YELLOW));
                            }
                        });
                        let cc = match tcm.clutch_state {
                            ClutchState::Locked => Color32::GREEN,
                            ClutchState::Modulating => Color32::YELLOW,
                            ClutchState::Filling => Color32::from_rgb(0, 220, 220),
                            ClutchState::Open => Color32::from_gray(70),
                        };
                        digital_readout(ui, "Clutch", &format!("{}", tcm.clutch_state), cc);
                        bar_gauge(
                            ui,
                            "Slip",
                            tcm.clutch_slip_pct,
                            100.0,
                            "%",
                            120.0,
                            if tcm.clutch_slip_pct > 20.0 {
                                Color32::YELLOW
                            } else {
                                Color32::GREEN
                            },
                        );
                        if tcm.is_shifting {
                            ui.label(
                                RichText::new("▶▶ SHIFTING…")
                                    .size(11.0)
                                    .color(Color32::YELLOW),
                            );
                        }
                        digital_readout(
                            ui,
                            "Speed",
                            &format!("{:.2} km/h", tcm.ground_speed_kmh),
                            Color32::LIGHT_BLUE,
                        );
                        digital_readout(
                            ui,
                            "Out RPM",
                            &format!("{:.0}", tcm.output_shaft_rpm),
                            Color32::from_gray(200),
                        );
                        digital_readout(
                            ui,
                            "Shifts",
                            &format!("{}", tcm.total_shifts),
                            Color32::from_gray(160),
                        );
                    });
                egui::Frame::group(ui.style())
                    .fill(Color32::from_gray(15))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("ABS / WHEEL SPEEDS")
                                .size(11.0)
                                .color(Color32::from_rgb(80, 155, 255)),
                        );
                        for (i, w) in abs.wheels.iter().enumerate() {
                            let n = ["FL", "FR", "RL", "RR"][i];
                            let c = if w.abs_active {
                                Color32::RED
                            } else if w.tcs_active {
                                Color32::from_rgb(80, 200, 255)
                            } else {
                                Color32::GREEN
                            };
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new(format!(
                                        "{}: {:5.1}km/h {}",
                                        n, w.speed, w.valve_state
                                    ))
                                    .size(10.5)
                                    .monospace()
                                    .color(c),
                                );
                                if w.slip_ratio > 0.1 {
                                    ui.label(
                                        RichText::new(format!("slip:{:.0}%", w.slip_ratio * 100.0))
                                            .size(10.0)
                                            .color(Color32::YELLOW),
                                    );
                                }
                            });
                        }
                        let ec = match abs.esp_condition {
                            EspCondition::Neutral => Color32::GREEN,
                            EspCondition::Understeer => Color32::YELLOW,
                            _ => Color32::RED,
                        };
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new("ESP:")
                                    .size(11.0)
                                    .color(Color32::from_gray(140)),
                            );
                            ui.label(
                                RichText::new(format!("{}", abs.esp_condition))
                                    .size(11.0)
                                    .color(ec)
                                    .strong(),
                            );
                        });
                    });
            });
            ui.separator();
            // Right: fuel/def/battery + hitch
            ui.vertical(|ui| {
                egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                    arc_gauge(
                        ui,
                        ecm.fuel_level_pct,
                        0.0,
                        100.0,
                        "FUEL",
                        "%",
                        125.0,
                        0.0,
                        0.0,
                        Some(12.0),
                    )
                });
                egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                    arc_gauge(
                        ui,
                        ecm.def_level_pct,
                        0.0,
                        100.0,
                        "DEF/AdBlue",
                        "%",
                        125.0,
                        0.0,
                        0.0,
                        Some(10.0),
                    )
                });
                egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                    arc_gauge(
                        ui,
                        self.bench.bcm.battery_voltage,
                        9.0,
                        16.0,
                        "BATTERY",
                        "V",
                        105.0,
                        0.0,
                        0.0,
                        Some(11.5),
                    )
                });
                egui::Frame::group(ui.style())
                    .fill(Color32::from_gray(15))
                    .show(ui, |ui| {
                        ui.label(
                            RichText::new("SERVICE")
                                .size(11.0)
                                .color(Color32::from_rgb(80, 155, 255)),
                        );
                        digital_readout(
                            ui,
                            "Hours",
                            &format!("{:.1} h", ecm.engine_hours),
                            Color32::from_gray(200),
                        );
                        digital_readout(
                            ui,
                            "Svc due",
                            &format!("{:.1} h", ecm.service_hours_left),
                            if ecm.service_hours_left < 50.0 {
                                Color32::RED
                            } else if ecm.service_hours_left < 100.0 {
                                Color32::YELLOW
                            } else {
                                Color32::GREEN
                            },
                        );
                        digital_readout(
                            ui,
                            "NOx tail",
                            &format!("{:.0} ppm", ecm.nox_tailpipe_ppm),
                            if ecm.nox_tailpipe_ppm > 500.0 {
                                Color32::RED
                            } else {
                                Color32::from_gray(180)
                            },
                        );
                    });
            });
        });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB CAN BUS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_can(&mut self, ui: &mut Ui) {
        // Controls bar
        ui.horizontal(|ui| {
            let mc = if self.can_mode == CanMode::Signals {
                Color32::from_rgb(45, 105, 210)
            } else {
                Color32::from_gray(35)
            };
            let tc = if self.can_mode == CanMode::Trace {
                Color32::from_rgb(45, 105, 210)
            } else {
                Color32::from_gray(35)
            };
            let nc = if self.can_mode == CanMode::Network {
                Color32::from_rgb(45, 105, 210)
            } else {
                Color32::from_gray(35)
            };
            if ui
                .add(Button::new(RichText::new("📊 Signals").size(12.0)).fill(mc))
                .clicked()
            {
                self.can_mode = CanMode::Signals;
            }
            if ui
                .add(Button::new(RichText::new("📋 Trace").size(12.0)).fill(tc))
                .clicked()
            {
                self.can_mode = CanMode::Trace;
            }
            if ui
                .add(Button::new(RichText::new("🌐 Network").size(12.0)).fill(nc))
                .clicked()
            {
                self.can_mode = CanMode::Network;
            }
            ui.separator();
            let fc = if self.can_freeze {
                Color32::RED
            } else {
                Color32::from_gray(38)
            };
            if ui
                .add(
                    Button::new(
                        RichText::new(if self.can_freeze {
                            "⏸ FROZEN"
                        } else {
                            "⏸ Freeze"
                        })
                        .size(12.0),
                    )
                    .fill(fc),
                )
                .clicked()
            {
                self.can_freeze = !self.can_freeze;
            }
            if ui.button("🗑 Clear").clicked() {
                self.sig_map.clear();
            }
            ui.separator();
            ui.label(RichText::new("Filter:").size(11.0).color(Color32::GRAY));
            ui.add_sized(
                [160.0, 20.0],
                TextEdit::singleline(&mut self.can_filter).hint_text("PGN name / SA hex…"),
            );
            ui.separator();
            let gw = &self.bench.gateway;
            let state_col = match gw.bus_state {
                BusState::ErrorActive => Color32::GREEN,
                BusState::ErrorPassive => Color32::YELLOW,
                BusState::BusOff => Color32::RED,
            };
            ui.label(
                RichText::new(format!(
                    "Bus:{} Load:{:.0}% Frames:{} {}/s Err:{}",
                    gw.bus_state, gw.bus_load_pct, gw.total_tx, gw.bus.fps, gw.total_errors
                ))
                .size(11.0)
                .color(state_col),
            );
        });
        ui.separator();
        match self.can_mode {
            CanMode::Signals => self.can_signals(ui),
            CanMode::Trace => self.can_trace(ui),
            CanMode::Network => self.can_network_overview(ui),
        }
    }

    fn can_network_overview(&mut self, ui: &mut Ui) {
        let net = &self.bench.can_net;
        let health = (net.network_health_score_01() * 100.0).round();
        let hc = if health >= 85.0 {
            Color32::GREEN
        } else if health >= 65.0 {
            Color32::YELLOW
        } else {
            Color32::RED
        };
        ui.label(RichText::new(format!(
            "Multi-bus overview | Health: {:.0}% | Frames(all buses): {} | Nodes online: {} | Errors(total): {}",
            health,
            net.total_frames_all_buses,
            net.online_count(),
            net.total_errors()
        )).size(11.0).color(hc));
        if !self.can_note.is_empty() {
            ui.label(
                RichText::new(&self.can_note)
                    .size(10.4)
                    .color(Color32::LIGHT_BLUE),
            );
        }
        ui.horizontal(|ui| {
            const BUS_NAMES: [&str; 5] = ["Powertrain", "Chassis", "Body", "ISOBUS", "Diagnostic"];
            ComboBox::from_label("Target bus")
                .selected_text(BUS_NAMES[self.can_bus_idx.min(4)])
                .show_ui(ui, |ui| {
                    for (i, n) in BUS_NAMES.iter().enumerate() {
                        ui.selectable_value(&mut self.can_bus_idx, i, *n);
                    }
                });

            if ui.button("Inject Bit").clicked() {
                self.cmds.push(Cmd::CanInjectBitError(self.can_bus_idx));
            }
            if ui.button("Inject Ack").clicked() {
                self.cmds.push(Cmd::CanInjectAckError(self.can_bus_idx));
            }
            if ui.button("Inject BusOff").clicked() {
                self.cmds.push(Cmd::CanInjectBusOff(self.can_bus_idx));
            }
            if ui.button("Inject Babbling").clicked() {
                self.cmds.push(Cmd::CanInjectBabbling(self.can_bus_idx));
            }
            if ui.button("Clear Bus").clicked() {
                self.cmds.push(Cmd::CanClearBusInjections(self.can_bus_idx));
            }
            if ui.button("Clear All").clicked() {
                self.cmds.push(Cmd::CanClearAllInjections);
            }
            ui.separator();
            if ui.button("Export CSV").clicked() {
                self.cmds.push(Cmd::CanExportSnapshotCsv);
            }
            if ui.button("Export JSON").clicked() {
                self.cmds.push(Cmd::CanExportSnapshotJson);
            }
        });
        ui.separator();

        let bus_cards = [
            ("HS-CAN Powertrain", &net.powertrain),
            ("HS-CAN Chassis", &net.chassis),
            ("MS-CAN Body", &net.body),
            ("ISOBUS", &net.isobus),
            ("Diag", &net.diagnostic),
        ];

        egui::Grid::new("can_net_buses")
            .num_columns(6)
            .striped(true)
            .show(ui, |ui| {
                for h in ["BUS", "STATE", "LOAD", "TX", "LOG", "ERRORS"] {
                    ui.label(
                        RichText::new(h)
                            .size(10.8)
                            .color(Color32::from_gray(160))
                            .strong(),
                    );
                }
                ui.end_row();

                for (name, bus) in bus_cards {
                    let state_col = match bus.state {
                        auto_breaking::can_network::BusState::ErrorActive => Color32::GREEN,
                        auto_breaking::can_network::BusState::ErrorPassive => Color32::YELLOW,
                        auto_breaking::can_network::BusState::BusOff => Color32::RED,
                    };
                    ui.label(
                        RichText::new(name)
                            .size(10.8)
                            .color(Color32::from_gray(220)),
                    );
                    ui.label(
                        RichText::new(format!("{}", bus.state))
                            .size(10.8)
                            .monospace()
                            .color(state_col),
                    );
                    ui.label(
                        RichText::new(format!("{:.1}%", bus.bus_load_pct))
                            .size(10.8)
                            .monospace()
                            .color(Color32::from_rgb(100, 205, 255)),
                    );
                    ui.label(
                        RichText::new(format!("{}", bus.total_tx))
                            .size(10.8)
                            .monospace()
                            .color(Color32::from_gray(190)),
                    );
                    ui.label(
                        RichText::new(format!("{:.0}", bus.frame_log.len()))
                            .size(10.8)
                            .monospace()
                            .color(Color32::from_gray(160)),
                    );
                    ui.label(
                        RichText::new(format!("{}", bus.total_errors))
                            .size(10.8)
                            .monospace()
                            .color(if bus.total_errors > 0 {
                                Color32::YELLOW
                            } else {
                                Color32::GREEN
                            }),
                    );
                    ui.end_row();
                }
            });

        ui.separator();
        ui.columns(2, |cols| {
            egui::Frame::group(&cols[0].style())
                .fill(Color32::from_gray(15))
                .show(&mut cols[0], |ui| {
                    ui.label(
                        RichText::new("VCM ROUTING TABLE")
                            .size(11.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    ui.separator();
                    for (pgn_id, src, dst) in &net.routing {
                        let pgn_name = auto_breaking::find_pgn(*pgn_id)
                            .map(|p| p.name)
                            .unwrap_or("???");
                        ui.label(
                            RichText::new(format!(
                                "PGN {:5} {:8}  {} -> {}",
                                pgn_id, pgn_name, src, dst
                            ))
                            .size(10.2)
                            .monospace()
                            .color(Color32::from_gray(200)),
                        );
                    }
                });

            egui::Frame::group(&cols[1].style())
                .fill(Color32::from_gray(15))
                .show(&mut cols[1], |ui| {
                    ui.label(
                        RichText::new("LATEST NETWORK ERRORS")
                            .size(11.0)
                            .color(Color32::from_rgb(255, 150, 80)),
                    );
                    ui.separator();
                    for err in net.all_errors().take(14) {
                        let sa = err.source_sa.map_or("--".into(), |v| format!("{:02X}", v));
                        let id = err
                            .raw_id
                            .map_or("--------".into(), |v| format!("{:08X}", v));
                        ui.label(
                            RichText::new(format!(
                                "{:.2}s [{}] SA {} ID {} {:?}",
                                err.timestamp, err.bus, sa, id, err.kind
                            ))
                            .size(9.9)
                            .monospace()
                            .color(Color32::from_rgb(255, 170, 110)),
                        );
                        ui.label(
                            RichText::new(err.description)
                                .size(9.6)
                                .color(Color32::from_gray(150)),
                        );
                    }
                });
        });
    }

    fn can_signals(&self, ui: &mut Ui) {
        let filt = self.can_filter.to_lowercase();
        // Table headers
        egui::Grid::new("sig_hdr")
            .num_columns(6)
            .min_col_width(60.0)
            .show(ui, |ui| {
                for h in ["AGE(s)", "PGN", "SA", "PERIOD", "COUNT", "DECODED VALUES"] {
                    ui.label(
                        RichText::new(h)
                            .size(10.5)
                            .color(Color32::from_gray(155))
                            .strong(),
                    );
                }
                ui.end_row();
            });
        ui.separator();
        let t = self.bench.elapsed;
        let mut sigs: Vec<(&(u32, u8), &Signal)> = self
            .sig_map
            .iter()
            .filter(|(_, s)| {
                filt.is_empty()
                    || s.pgn_name.to_lowercase().contains(&filt)
                    || s.sa_name.to_lowercase().contains(&filt)
            })
            .collect();
        sigs.sort_by_key(|((pgn, sa), _)| (*pgn, *sa));

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .max_height(ui.available_height())
            .show(ui, |ui| {
                for (_, sig) in &sigs {
                    let age = t - sig.last_ts;
                    let ac = if age < 0.05 {
                        Color32::GREEN
                    } else if age < 0.5 {
                        Color32::from_gray(220)
                    } else if age < 2.0 {
                        Color32::from_gray(150)
                    } else {
                        Color32::from_gray(80)
                    };
                    let vals = sig
                        .decoded
                        .iter()
                        .take(4)
                        .map(|(n, v, u)| {
                            format!(
                                "{}:{:.1}{}",
                                n.split_whitespace().next().unwrap_or("?"),
                                v,
                                u
                            )
                        })
                        .collect::<Vec<_>>()
                        .join("  ");
                    egui::Grid::new(format!("s{:08X}", sig.raw_id))
                        .num_columns(6)
                        .min_col_width(60.0)
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!("{:.2}", age))
                                    .size(10.5)
                                    .monospace()
                                    .color(ac),
                            );
                            ui.label(
                                RichText::new(sig.pgn_name)
                                    .size(10.5)
                                    .monospace()
                                    .color(Color32::GOLD),
                            );
                            ui.label(
                                RichText::new(&sig.sa_name)
                                    .size(10.5)
                                    .monospace()
                                    .color(Color32::from_rgb(120, 200, 255)),
                            );
                            ui.label(
                                RichText::new(format!("{:.0}ms", sig.period_ms))
                                    .size(10.5)
                                    .monospace()
                                    .color(Color32::from_gray(150)),
                            );
                            ui.label(
                                RichText::new(format!("{}", sig.count))
                                    .size(10.5)
                                    .monospace()
                                    .color(Color32::from_gray(130)),
                            );
                            ui.label(RichText::new(&vals).size(10.5).color(ac));
                            ui.end_row();
                        });
                }
            });
    }

    fn can_trace(&self, ui: &mut Ui) {
        egui::Grid::new("trace_hdr")
            .num_columns(6)
            .min_col_width(50.0)
            .show(ui, |ui| {
                for h in ["TIME(s)", "CAN-ID", "DLC", "HEX DATA", "PGN", "DECODED"] {
                    ui.label(
                        RichText::new(h)
                            .size(10.5)
                            .color(Color32::from_gray(155))
                            .strong(),
                    );
                }
                ui.end_row();
            });
        ui.separator();
        let snap = &self.trace_snap;
        // stick_to_bottom only when live; when frozen the user can scroll freely
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .max_height(ui.available_height())
            .stick_to_bottom(!self.can_freeze)
            .show(ui, |ui| {
                for (ts, raw_id, sa, dlc, hex, pgn_sa, decoded) in snap.iter().rev().take(500).rev()
                {
                    let sc = match sa {
                        0x00 => Color32::from_rgb(80, 220, 80),
                        0x03 => Color32::from_rgb(0, 210, 210),
                        0x27 => Color32::LIGHT_BLUE,
                        0x1E => Color32::YELLOW,
                        0x0B => Color32::WHITE,
                        _ => Color32::from_gray(175),
                    };
                    egui::Grid::new(format!("tr{:.3}{}", ts, raw_id))
                        .num_columns(6)
                        .min_col_width(50.0)
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!("{:8.3}", ts))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(110)),
                            );
                            ui.label(
                                RichText::new(format!("{:08X}", raw_id))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::LIGHT_BLUE),
                            );
                            ui.label(
                                RichText::new(format!("{}", dlc))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(130)),
                            );
                            ui.label(
                                RichText::new(hex)
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(195)),
                            );
                            ui.label(RichText::new(pgn_sa).size(10.0).color(sc));
                            ui.label(
                                RichText::new(decoded)
                                    .size(10.0)
                                    .color(Color32::from_gray(200)),
                            );
                            ui.end_row();
                        });
                }
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB EVENTS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_events(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            let pc = if self.ev_pause {
                Color32::RED
            } else {
                Color32::from_gray(38)
            };
            if ui
                .add(
                    Button::new(
                        RichText::new(if self.ev_pause {
                            "⏸ FROZEN"
                        } else {
                            "⏸ Pause"
                        })
                        .size(12.0),
                    )
                    .fill(pc),
                )
                .clicked()
            {
                self.ev_pause = !self.ev_pause;
            }
            if ui.button("🗑 Clear").clicked() {
                self.events.clear();
            }
            ui.separator();
            ui.label(RichText::new("Filter:").size(11.0).color(Color32::GRAY));
            ui.add_sized(
                [150.0, 20.0],
                TextEdit::singleline(&mut self.ev_filter).hint_text("text filter…"),
            );
            ui.separator();
            // Level filter buttons
            for (lvl, lbl, col) in [
                (EventLevel::Debug, "DBG", Color32::from_gray(140)),
                (EventLevel::Info, "INFO", Color32::LIGHT_BLUE),
                (EventLevel::Ok, "OK", Color32::GREEN),
                (EventLevel::Warn, "WARN", Color32::YELLOW),
                (EventLevel::Critical, "CRIT", Color32::RED),
            ] {
                let sel = self.ev_min_level == lvl;
                let fill = if sel {
                    Color32::from_gray(60)
                } else {
                    Color32::from_gray(30)
                };
                if ui
                    .add(Button::new(RichText::new(lbl).size(11.0).color(col)).fill(fill))
                    .clicked()
                {
                    self.ev_min_level = lvl;
                }
            }
            ui.separator();
            ui.label(
                RichText::new(format!("{} events", self.events.len()))
                    .size(11.0)
                    .color(Color32::from_gray(140)),
            );
        });
        ui.separator();

        // Column headers
        egui::Grid::new("ev_hdr")
            .num_columns(4)
            .min_col_width(40.0)
            .show(ui, |ui| {
                for h in ["TIME(s)", "LVL", "SOURCE", "EVENT"] {
                    ui.label(
                        RichText::new(h)
                            .size(10.5)
                            .color(Color32::from_gray(150))
                            .strong(),
                    );
                }
                ui.end_row();
            });
        ui.separator();

        let filt = self.ev_filter.to_lowercase();
        let min_lvl = self.ev_min_level;
        let level_order = |l: &EventLevel| -> u8 {
            match l {
                EventLevel::Debug => 0,
                EventLevel::Info => 1,
                EventLevel::Ok => 2,
                EventLevel::Warn => 3,
                EventLevel::Critical => 4,
            }
        };
        let min_ord = level_order(&min_lvl);

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .stick_to_bottom(!self.ev_pause)
            .show(ui, |ui| {
                for ev in self
                    .events
                    .iter()
                    .rev()
                    .filter(|e| level_order(&e.lvl) >= min_ord)
                    .filter(|e| {
                        filt.is_empty()
                            || e.msg.to_lowercase().contains(&filt)
                            || e.source.to_lowercase().contains(&filt)
                    })
                    .take(300)
                {
                    let (icon, col) = match ev.lvl {
                        EventLevel::Debug => ("·", Color32::from_gray(100)),
                        EventLevel::Info => ("ℹ", Color32::LIGHT_BLUE),
                        EventLevel::Ok => ("✓", Color32::GREEN),
                        EventLevel::Warn => ("⚠", Color32::YELLOW),
                        EventLevel::Critical => ("✗", Color32::RED),
                    };
                    egui::Grid::new(format!("ev{:.3}{}", ev.ts, ev.source))
                        .num_columns(4)
                        .min_col_width(40.0)
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new(format!("{:7.2}", ev.ts))
                                    .size(10.5)
                                    .monospace()
                                    .color(Color32::from_gray(115)),
                            );
                            ui.label(RichText::new(format!("{} ", icon)).size(11.0).color(col));
                            ui.label(
                                RichText::new(format!("[{:<5}]", ev.source))
                                    .size(10.5)
                                    .monospace()
                                    .color(Color32::from_gray(165)),
                            );
                            ui.label(RichText::new(&ev.msg).size(10.5).color(col));
                            ui.end_row();
                        });
                }
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB ECU NETWORK
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_ecu_net(&self, ui: &mut Ui) {
        let gw = &self.bench.gateway;
        ui.label(
            RichText::new(format!(
                "J1939 HS-CAN 500kbps | Bus:{} | Nodes:{}/{} | TotalFrames:{}",
                gw.bus_state,
                self.bench
                    .boot
                    .ecus
                    .iter()
                    .filter(|e| e.is_online())
                    .count(),
                self.bench.boot.ecus.len(),
                gw.total_tx
            ))
            .size(12.0)
            .color(Color32::from_gray(185)),
        );
        ui.separator();

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::Grid::new("ecu_grid")
                    .num_columns(7)
                    .striped(true)
                    .show(ui, |ui| {
                        for h in [
                            "MODULE",
                            "SA",
                            "STAGE",
                            "ONLINE AT",
                            "TEC",
                            "REC",
                            "BUS STATE",
                        ] {
                            ui.label(RichText::new(h).size(11.0).color(Color32::from_gray(150)));
                        }
                        ui.end_row();
                        for ecu in &self.bench.boot.ecus {
                            let sc = match ecu.stage {
                                EcuBootStage::Running => Color32::GREEN,
                                EcuBootStage::Fault => Color32::RED,
                                EcuBootStage::Unpowered => Color32::from_gray(55),
                                EcuBootStage::AddressClaiming => Color32::YELLOW,
                                _ => Color32::from_gray(185),
                            };
                            let node = gw.nodes.iter().find(|n| n.source_addr == ecu.sa);
                            let (tec, rec, bs) = node.map_or((0u16, 0u16, "N/A"), |n| {
                                (
                                    n.tec,
                                    n.rec,
                                    match n.state {
                                        BusState::ErrorActive => "EA",
                                        BusState::ErrorPassive => "EP",
                                        BusState::BusOff => "BO",
                                    },
                                )
                            });
                            ui.label(
                                RichText::new(ecu.name)
                                    .size(11.0)
                                    .color(Color32::from_gray(220)),
                            );
                            ui.label(
                                RichText::new(format!("0x{:02X}", ecu.sa))
                                    .size(11.0)
                                    .monospace()
                                    .color(Color32::YELLOW),
                            );
                            ui.label(RichText::new(format!("{}", ecu.stage)).size(11.0).color(sc));
                            ui.label(
                                RichText::new(
                                    ecu.online_at.map_or("—".into(), |t| format!("{:.2}s", t)),
                                )
                                .size(11.0)
                                .color(Color32::from_gray(150)),
                            );
                            ui.label(
                                RichText::new(format!("{}", tec))
                                    .size(11.0)
                                    .monospace()
                                    .color(if tec > 0 {
                                        Color32::YELLOW
                                    } else {
                                        Color32::GREEN
                                    }),
                            );
                            ui.label(
                                RichText::new(format!("{}", rec))
                                    .size(11.0)
                                    .monospace()
                                    .color(if rec > 0 {
                                        Color32::YELLOW
                                    } else {
                                        Color32::GREEN
                                    }),
                            );
                            ui.label(RichText::new(bs).size(11.0).color(match bs {
                                "BO" => Color32::RED,
                                "EP" => Color32::YELLOW,
                                _ => Color32::GREEN,
                            }));
                            ui.end_row();
                        }
                    });
                ui.separator();
                ui.label(
                    RichText::new("BUS ERROR LOG:")
                        .size(11.0)
                        .color(Color32::from_gray(140)),
                );
                for err in &gw.error_log {
                    let sa = err
                        .source_sa
                        .map_or("----".to_string(), |s| format!("0x{:02X}", s));
                    ui.label(
                        RichText::new(format!(
                            "{:6.2}s SA:{}  {:?}  {}",
                            err.timestamp, sa, err.kind, err.description
                        ))
                        .size(10.5)
                        .monospace()
                        .color(Color32::RED),
                    );
                }
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB ENGINE (detailed ECM)
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_engine(&self, ui: &mut Ui) {
        let e = &self.bench.ecm;
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.columns(3, |cols| {
                    // Col 0: Engine core
                    egui::Frame::group(&cols[0].style())
                        .fill(Color32::from_gray(16))
                        .show(&mut cols[0], |ui| {
                            ui.label(
                                RichText::new("ENGINE CORE")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            let w = 140.0;
                            digital_readout(
                                ui,
                                "Governor",
                                &format!("{}", e.governor_mode),
                                Color32::YELLOW,
                            );
                            digital_readout(
                                ui,
                                "Fuel Map",
                                &format!("{}", e.fuel_map),
                                Color32::YELLOW,
                            );
                            bar_gauge(ui, "RPM", e.rpm, 3000.0, "rpm", w, Color32::GREEN);
                            bar_gauge(ui, "Load", e.percent_load, 150.0, "%", w, Color32::YELLOW);
                            bar_gauge(
                                ui,
                                "Torque",
                                e.actual_torque_nm,
                                1200.0,
                                "Nm",
                                w,
                                Color32::from_rgb(80, 180, 255),
                            );
                            bar_gauge(
                                ui,
                                "Power",
                                e.power_kw(),
                                260.0,
                                "kW",
                                w,
                                Color32::from_rgb(100, 220, 100),
                            );
                            bar_gauge(
                                ui,
                                "Throttle",
                                e.active_throttle,
                                100.0,
                                "%",
                                w,
                                Color32::YELLOW,
                            );
                            ui.separator();
                            bar_gauge(
                                ui,
                                "Rail P",
                                e.fuel_rail_pressure_mpa * 10.0,
                                2000.0,
                                "bar",
                                w,
                                Color32::from_rgb(0, 210, 210),
                            );
                            bar_gauge(
                                ui,
                                "Fuel Prs",
                                e.fuel_pressure_kpa,
                                600.0,
                                "kPa",
                                w,
                                Color32::from_rgb(0, 210, 210),
                            );
                            bar_gauge(
                                ui,
                                "Fuel/h",
                                e.fuel_rate_lph,
                                30.0,
                                "L/h",
                                w,
                                Color32::YELLOW,
                            );
                            bar_gauge(
                                ui,
                                "Boost",
                                e.boost_pressure_kpa,
                                300.0,
                                "kPa",
                                w,
                                Color32::from_rgb(180, 80, 255),
                            );
                            bar_gauge(
                                ui,
                                "VGT",
                                e.vgt_position_pct,
                                100.0,
                                "%",
                                w,
                                Color32::from_gray(200),
                            );
                            bar_gauge(
                                ui,
                                "EGR",
                                e.egr_valve_pct,
                                100.0,
                                "%",
                                w,
                                Color32::from_gray(200),
                            );
                        });
                    // Col 1: Temperatures & pressures
                    egui::Frame::group(&cols[1].style())
                        .fill(Color32::from_gray(16))
                        .show(&mut cols[1], |ui| {
                            ui.label(
                                RichText::new("THERMAL & PRESSURE")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            let w = 140.0;
                            bar_gauge(
                                ui,
                                "Coolant",
                                e.coolant_temp_c,
                                130.0,
                                "°C",
                                w,
                                if e.coolant_temp_c > 100.0 {
                                    Color32::RED
                                } else {
                                    Color32::GREEN
                                },
                            );
                            bar_gauge(
                                ui,
                                "Oil",
                                e.oil_temp_c,
                                160.0,
                                "°C",
                                w,
                                Color32::from_rgb(200, 150, 50),
                            );
                            bar_gauge(
                                ui,
                                "Exhaust",
                                e.exhaust_temp_c,
                                900.0,
                                "°C",
                                w,
                                Color32::from_rgb(255, 120, 30),
                            );
                            bar_gauge(
                                ui,
                                "Intake",
                                e.intake_temp_c,
                                80.0,
                                "°C",
                                w,
                                Color32::from_gray(200),
                            );
                            bar_gauge(
                                ui,
                                "Fuel T",
                                e.fuel_temp_c,
                                80.0,
                                "°C",
                                w,
                                Color32::from_gray(200),
                            );
                            ui.separator();
                            bar_gauge(
                                ui,
                                "Oil Prs",
                                e.oil_pressure_kpa,
                                700.0,
                                "kPa",
                                w,
                                if e.oil_pressure_kpa < 100.0 {
                                    Color32::RED
                                } else {
                                    Color32::GREEN
                                },
                            );
                            bar_gauge(
                                ui,
                                "Coolant P",
                                e.coolant_pres_kpa,
                                150.0,
                                "kPa",
                                w,
                                Color32::from_gray(200),
                            );
                            bar_gauge(
                                ui,
                                "Air Filter",
                                e.air_filter_dp_kpa,
                                10.0,
                                "kPa",
                                w,
                                if e.air_filter_dp_kpa > 5.0 {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            ui.separator();
                            digital_readout(
                                ui,
                                "Alt V",
                                &format!("{:.1}V", e.alternator_v),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Batt V",
                                &format!("{:.1}V", self.bench.bcm.battery_voltage),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Engine H",
                                &format!("{:.1}h", e.engine_hours),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Svc due",
                                &format!("{:.1}h", e.service_hours_left),
                                if e.service_hours_left < 50.0 {
                                    Color32::RED
                                } else {
                                    Color32::GREEN
                                },
                            );
                        });
                    // Col 2: Aftertreatment + DTCs
                    egui::Frame::group(&cols[2].style())
                        .fill(Color32::from_gray(16))
                        .show(&mut cols[2], |ui| {
                            ui.label(
                                RichText::new("AFTERTREATMENT & DTCS")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            let w = 130.0;
                            let ac = match e.aftertreatment {
                                AftertreatmentState::Normal => Color32::GREEN,
                                AftertreatmentState::DpfRegen => Color32::from_rgb(200, 80, 200),
                                AftertreatmentState::ScrDegraded => Color32::YELLOW,
                                _ => Color32::RED,
                            };
                            digital_readout(ui, "State", &format!("{}", e.aftertreatment), ac);
                            bar_gauge(
                                ui,
                                "DPF Soot",
                                e.dpf_soot_pct,
                                100.0,
                                "%",
                                w,
                                if e.dpf_soot_pct > 75.0 {
                                    Color32::RED
                                } else {
                                    Color32::from_rgb(180, 100, 220)
                                },
                            );
                            bar_gauge(
                                ui,
                                "DPF Temp",
                                e.dpf_temp_c,
                                700.0,
                                "°C",
                                w,
                                Color32::from_rgb(255, 130, 30),
                            );
                            bar_gauge(
                                ui,
                                "DEF Level",
                                e.def_level_pct,
                                100.0,
                                "%",
                                w,
                                if e.def_level_pct < 10.0 {
                                    Color32::RED
                                } else {
                                    Color32::from_rgb(0, 210, 210)
                                },
                            );
                            bar_gauge(
                                ui,
                                "SCR Eff",
                                e.scr_efficiency_pct,
                                100.0,
                                "%",
                                w,
                                if e.scr_efficiency_pct < 80.0 {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "NOx raw",
                                &format!("{:.0} ppm", e.nox_raw_ppm),
                                Color32::from_gray(180),
                            );
                            digital_readout(
                                ui,
                                "NOx tail",
                                &format!("{:.0} ppm", e.nox_tailpipe_ppm),
                                if e.nox_tailpipe_ppm > 500.0 {
                                    Color32::RED
                                } else {
                                    Color32::from_gray(180)
                                },
                            );
                            ui.separator();
                            ui.label(
                                RichText::new("ACTIVE DTCs:")
                                    .size(11.0)
                                    .color(Color32::from_gray(140)),
                            );
                            if e.active_dtcs.is_empty() {
                                ui.label(RichText::new("✓ None").size(11.0).color(Color32::GREEN));
                            }
                            for dtc in &e.active_dtcs {
                                let c = match dtc.severity {
                                    DtcSeverity::Red => Color32::RED,
                                    DtcSeverity::Amber => Color32::GOLD,
                                    _ => Color32::LIGHT_BLUE,
                                };
                                ui.label(
                                    RichText::new(format!(
                                        "SPN:{} FMI:{} ×{}  {}",
                                        dtc.spn, dtc.fmi, dtc.count, dtc.desc
                                    ))
                                    .size(10.5)
                                    .color(c),
                                );
                            }
                        });
                });
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB FAULTS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_faults(&mut self, ui: &mut Ui) {
        ui.columns(2, |cols| {
            // Left: active DTCs
            let ui = &mut cols[0];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("ACTIVE DTCs (DM1)")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    ui.separator();
                    let dtcs_empty = self.bench.ecm.active_dtcs.is_empty();
                    if dtcs_empty {
                        ui.add_space(10.0);
                        ui.label(
                            RichText::new("✓  No active fault codes")
                                .size(13.0)
                                .color(Color32::GREEN),
                        );
                    } else {
                        ScrollArea::vertical()
                            .max_height(250.0)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                for dtc in &self.bench.ecm.active_dtcs {
                                    let c = match dtc.severity {
                                        DtcSeverity::Red => Color32::RED,
                                        DtcSeverity::Amber => Color32::GOLD,
                                        DtcSeverity::Mil => Color32::LIGHT_BLUE,
                                        DtcSeverity::Protect => Color32::from_rgb(0, 210, 210),
                                    };
                                    egui::Frame::group(ui.style())
                                        .fill(Color32::from_gray(20))
                                        .show(ui, |ui| {
                                            ui.horizontal(|ui| {
                                                ui.label(
                                                    RichText::new(format!("SPN {:6}", dtc.spn))
                                                        .size(13.0)
                                                        .color(c)
                                                        .strong(),
                                                );
                                                ui.label(
                                                    RichText::new(format!("FMI {:2}", dtc.fmi))
                                                        .size(12.0)
                                                        .color(Color32::GOLD),
                                                );
                                                ui.label(
                                                    RichText::new(format!("×{}", dtc.count))
                                                        .size(11.0)
                                                        .color(Color32::GRAY),
                                                );
                                                ui.label(
                                                    RichText::new(format!("[{}]", dtc.severity))
                                                        .size(11.0)
                                                        .color(c),
                                                );
                                            });
                                            ui.label(
                                                RichText::new(dtc.desc)
                                                    .size(11.0)
                                                    .color(Color32::from_gray(200)),
                                            );
                                        });
                                }
                            });
                    }
                    ui.separator();
                    // Warning lamps
                    ui.horizontal(|ui| {
                        let e = &self.bench.ecm;
                        for (lbl, on, c) in [
                            ("🔴 RED", e.red_lamp, Color32::RED),
                            ("🟡 AMB", e.amber_lamp, Color32::GOLD),
                            ("🔵 MIL", e.mil_active, Color32::LIGHT_BLUE),
                            ("🛡 PROT", e.protect_lamp, Color32::from_rgb(0, 210, 210)),
                        ] {
                            ui.label(
                                RichText::new(lbl)
                                    .size(12.0)
                                    .color(if on { c } else { Color32::from_gray(55) })
                                    .strong(),
                            );
                            ui.add_space(4.0);
                        }
                    });
                    ui.add_space(6.0);
                    if ui
                        .add(
                            Button::new(RichText::new("🗑 CLEAR ALL DTCs").size(12.0))
                                .fill(Color32::from_rgb(50, 22, 22)),
                        )
                        .clicked()
                    {
                        self.cmds.push(Cmd::ClearDtcs);
                    }
                });

            // Right: fault injection
            let ui = &mut cols[1];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("FAULT INJECTION PANEL")
                            .size(12.0)
                            .color(Color32::from_rgb(200, 80, 80)),
                    );
                    ui.separator();
                    ScrollArea::vertical()
                        .max_height(340.0)
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            for (i, ft) in FAULT_TYPES.iter().enumerate() {
                                let sel = i == self.fault_idx;
                                let act =
                                    self.bench.fault_active && self.bench.selected_fault == *ft;
                                let fill = if act {
                                    Color32::from_rgb(70, 15, 15)
                                } else if sel {
                                    Color32::from_rgb(25, 45, 75)
                                } else {
                                    Color32::TRANSPARENT
                                };
                                let col = if act {
                                    Color32::RED
                                } else if sel {
                                    Color32::from_rgb(180, 220, 255)
                                } else {
                                    Color32::from_gray(155)
                                };
                                let btn = Button::new(
                                    RichText::new(format!("{}", ft)).size(11.0).color(col),
                                )
                                .fill(fill)
                                .min_size(Vec2::new(ui.available_width(), 22.0));
                                if ui.add(btn).clicked() {
                                    self.cmds.push(Cmd::SelectFault(i));
                                }
                            }
                        });
                    ui.add_space(6.0);
                    ui.horizontal(|ui| {
                        let ic = if self.bench.fault_active {
                            Color32::RED
                        } else {
                            Color32::from_rgb(160, 35, 35)
                        };
                        let lbl = if self.bench.fault_active {
                            "⚡ ACTIVE — Inject Again"
                        } else {
                            "⚡ INJECT FAULT"
                        };
                        if ui
                            .add(
                                Button::new(RichText::new(lbl).size(13.0))
                                    .fill(ic)
                                    .min_size(Vec2::new(185.0, 36.0)),
                            )
                            .clicked()
                        {
                            self.cmds.push(Cmd::InjectFault);
                        }
                        if ui
                            .add(
                                Button::new(RichText::new("🛡 CLEAR").size(13.0))
                                    .fill(Color32::from_rgb(22, 68, 32))
                                    .min_size(Vec2::new(90.0, 36.0)),
                            )
                            .clicked()
                        {
                            self.cmds.push(Cmd::ClearFaults);
                        }
                    });
                    if self.bench.fault_active {
                        ui.add_space(4.0);
                        ui.label(
                            RichText::new(format!("▶ ACTIVE FAULT: {}", self.bench.selected_fault))
                                .size(11.5)
                                .color(Color32::RED)
                                .strong(),
                        );
                    }
                });
        });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB BOOT
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_boot(&self, ui: &mut Ui) {
        let ign = self.bench.ignition();
        ui.horizontal(|ui| {
            for (lbl, s) in [
                ("OFF", IgnitionState::Off),
                ("ACC", IgnitionState::Accessory),
                ("ON", IgnitionState::On),
                ("CRANK", IgnitionState::Cranking),
                ("RUN", IgnitionState::Running),
            ] {
                let cur = ign == s;
                let c = if cur {
                    Color32::GREEN
                } else {
                    Color32::from_gray(80)
                };
                ui.label(
                    RichText::new(format!("[{}]", lbl))
                        .size(13.0)
                        .color(c)
                        .strong(),
                );
                if s != IgnitionState::Running {
                    ui.label(RichText::new("→").size(11.0).color(Color32::from_gray(80)));
                }
            }
            ui.separator();
            ui.label(
                RichText::new(format!(
                    "t={:.1}s  Online:{}/{}",
                    self.bench.elapsed,
                    self.bench
                        .boot
                        .ecus
                        .iter()
                        .filter(|e| e.is_online())
                        .count(),
                    self.bench.boot.ecus.len()
                ))
                .size(11.0)
                .color(Color32::from_gray(170)),
            );
        });
        ui.separator();
        ui.label(
            RichText::new("SAFETY INTERLOCKS:")
                .size(11.0)
                .color(Color32::from_gray(140)),
        );
        ui.columns(3, |cols| {
            let ils = &self.bench.boot.safety_interlocks;
            let per = (ils.len() + 2) / 3;
            for (ci, col) in cols.iter_mut().enumerate() {
                for il in ils.iter().skip(ci * per).take(per) {
                    let (sym, c) = if il.satisfied {
                        ("✓", Color32::GREEN)
                    } else if il.blocks_start {
                        ("✗", Color32::RED)
                    } else {
                        ("△", Color32::YELLOW)
                    };
                    col.horizontal(|ui| {
                        ui.label(RichText::new(sym).size(13.0).color(c));
                        ui.label(
                            RichText::new(&format!("{}: {}", il.id, il.description))
                                .size(10.5)
                                .color(Color32::from_gray(185)),
                        );
                        if il.blocks_start && !il.satisfied {
                            ui.label(RichText::new("[BLOCKS]").size(10.0).color(Color32::RED));
                        }
                    });
                }
            }
        });
        if self.bench.boot.crank_inhibited {
            ui.label(
                RichText::new(format!(
                    "⚠ CRANK INHIBITED: {}",
                    self.bench.boot.crank_inhibit_reason.unwrap_or("?")
                ))
                .size(12.0)
                .color(Color32::RED),
            );
        } else {
            ui.label(
                RichText::new("✓ CRANK CLEAR")
                    .size(12.0)
                    .color(Color32::GREEN),
            );
        }
        ui.separator();
        ui.label(
            RichText::new("BOOT EVENT LOG:")
                .size(11.0)
                .color(Color32::from_gray(140)),
        );
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for ev in self.bench.boot.event_log.iter().rev().take(80) {
                    let c = match ev.event {
                        auto_breaking::boot_sequence::BootEventKind::Running => Color32::GREEN,
                        auto_breaking::boot_sequence::BootEventKind::Fault => Color32::RED,
                        auto_breaking::boot_sequence::BootEventKind::AddressClaimed => {
                            Color32::from_rgb(0, 210, 210)
                        }
                        _ => Color32::from_gray(195),
                    };
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("{:7.3}s", ev.timestamp))
                                .size(10.5)
                                .monospace()
                                .color(Color32::from_gray(115)),
                        );
                        ui.label(
                            RichText::new(format!("[{:<12}]", ev.ecu_name))
                                .size(10.5)
                                .monospace()
                                .color(Color32::from_gray(165)),
                        );
                        ui.label(RichText::new(&ev.description).size(10.5).color(c));
                    });
                }
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB IMPLEMENTS — fully interactive
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_implements(&mut self, ui: &mut Ui) {
        // Snapshot display values
        let pto_rpm = self.bench.implement.pto_rear_rpm;
        let pto_mode_s = format!("{}", self.bench.implement.pto_mode);
        let pto_target = self.bench.implement.pto_target_rpm;
        let pto_slip = self.bench.implement.pto_slip_pct;
        let pto_torq = self.bench.implement.pto_shaft_torque_nm;
        let pto_over = self.bench.implement.overload_protection_active;
        let _pto_en = self.bench.implement.pto_rear_enabled;
        let hitch_pos = self.bench.implement.hitch_position_pct;
        let hitch_draft = self.bench.implement.hitch_draft_force_kn;
        let hitch_mode = format!("{}", self.bench.implement.hitch_control_mode);
        let hitch_gp = self.bench.implement.hitch_ground_pressure_kpa;
        let hcm_sys_p = self.bench.hcm.system_pressure_bar;
        let hcm_flow = self.bench.hcm.pump_flow_lpm;
        let hcm_temp = self.bench.hcm.fluid_temp_c;
        let hcm_mode = format!("{}", self.bench.hcm.pump_mode);
        let hcm_pwr = self.bench.hcm.hydraulic_power_kw;
        let aux_flows: Vec<f64> = self
            .bench
            .implement
            .aux_banks
            .iter()
            .map(|b| b.flow_lpm)
            .collect();
        let aux_pres: Vec<f64> = self
            .bench
            .implement
            .aux_banks
            .iter()
            .map(|b| b.pressure_bar)
            .collect();
        let aux_dirs: Vec<String> = self
            .bench
            .implement
            .aux_banks
            .iter()
            .map(|b| format!("{}", b.direction))
            .collect();
        let loader_lift = self.bench.hcm.loader_lift.position_pct();
        let _loader_tilt = self.bench.hcm.loader_tilt.position_pct();

        ui.columns(3, |cols| {
            // ── PTO ──────────────────────────────────────────────────────────
            let ui = &mut cols[0];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("POWER TAKE-OFF (TDP)")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    egui::Frame::dark_canvas(ui.style()).show(ui, |ui| {
                        arc_gauge(
                            ui, pto_rpm, 0.0, 1100.0, "PTO REAR", "rpm", 140.0, 900.0, 1050.0, None,
                        );
                    });
                    digital_readout(ui, "Mode", &pto_mode_s, Color32::YELLOW);
                    digital_readout(
                        ui,
                        "Target",
                        &format!("{:.0} rpm", pto_target),
                        Color32::from_gray(185),
                    );
                    digital_readout(
                        ui,
                        "Shaft Tq",
                        &format!("{:.0} Nm", pto_torq),
                        Color32::from_rgb(0, 210, 210),
                    );
                    bar_gauge(
                        ui,
                        "Slip",
                        pto_slip,
                        100.0,
                        "%",
                        120.0,
                        if pto_slip > 10.0 {
                            Color32::YELLOW
                        } else {
                            Color32::GREEN
                        },
                    );
                    if pto_over {
                        ui.label(
                            RichText::new("⚠ OVERLOAD!")
                                .size(12.0)
                                .color(Color32::RED)
                                .strong(),
                        );
                    }
                    ui.separator();
                    ui.label(
                        RichText::new("Mode select:")
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    ui.horizontal_wrapped(|ui| {
                        for (lbl, mode) in [
                            ("OFF", PtoMode::Off),
                            ("540", PtoMode::Std540),
                            ("1000", PtoMode::Std1000),
                            ("540eco", PtoMode::Economy540),
                            ("1000eco", PtoMode::Economy1000),
                        ] {
                            let active = pto_mode_s.trim() == format!("{}", mode).trim();
                            let fill = if active {
                                Color32::from_rgb(20, 80, 20)
                            } else {
                                Color32::from_gray(35)
                            };
                            if ui
                                .add(Button::new(RichText::new(lbl).size(11.0)).fill(fill))
                                .clicked()
                            {
                                self.cmds.push(Cmd::SetPtoMode(mode));
                            }
                        }
                    });
                });

            // ── 3-Point Hitch ─────────────────────────────────────────────────
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("3-POINT HITCH")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    // Visual position bar
                    let (rect, _) = ui
                        .allocate_exact_size(Vec2::new(ui.available_width(), 70.0), Sense::hover());
                    let p = ui.painter_at(rect);
                    p.rect_filled(rect, 4.0, Color32::from_gray(20));
                    let frac = 1.0 - hitch_pos as f32 / 100.0;
                    let fill_h = rect.height() * frac;
                    let fill_r = Rect::from_min_size(
                        Pos2::new(rect.min.x, rect.min.y + fill_h),
                        Vec2::new(rect.width(), rect.height() - fill_h),
                    );
                    p.rect_filled(fill_r, 4.0, Color32::from_rgb(30, 140, 200));
                    p.text(
                        rect.center(),
                        Align2::CENTER_CENTER,
                        format!("{:.1}%", hitch_pos),
                        FontId::proportional(18.0),
                        Color32::WHITE,
                    );

                    bar_gauge(ui, "Draft", hitch_draft, 50.0, "kN", 120.0, Color32::YELLOW);
                    digital_readout(ui, "Mode", &hitch_mode, Color32::from_gray(200));
                    digital_readout(
                        ui,
                        "Gnd P",
                        &format!("{:.1} kPa", hitch_gp),
                        Color32::from_gray(180),
                    );
                    ui.separator();
                    // Target slider
                    ui.label(
                        RichText::new("Target position:")
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    let mut ht = self.hitch_target as f32;
                    ui.add_sized(
                        [ui.available_width() - 10.0, 22.0],
                        Slider::new(&mut ht, 0.0..=100.0).suffix(" %"),
                    );
                    if ht as f64 != self.hitch_target {
                        self.cmds.push(Cmd::SetHitchTarget(ht as f64));
                    }
                    ui.horizontal(|ui| {
                        if ui.button("▲ RAISE").clicked() {
                            self.cmds.push(Cmd::SetHitchJoystick(1.0));
                        }
                        if ui.button("■ HOLD").clicked() {
                            self.cmds.push(Cmd::SetHitchJoystick(0.0));
                        }
                        if ui.button("▼ LOWER").clicked() {
                            self.cmds.push(Cmd::SetHitchJoystick(-1.0));
                        }
                    });
                });

            // ── Hydraulics + Aux Valves ───────────────────────────────────────
            let ui = &mut cols[1];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("HYDRAULIC SYSTEM")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    let w = 110.0;
                    bar_gauge(ui, "Sys Pres", hcm_sys_p, 230.0, "bar", w, Color32::YELLOW);
                    bar_gauge(
                        ui,
                        "Flow",
                        hcm_flow,
                        300.0,
                        "L/m",
                        w,
                        Color32::from_rgb(0, 210, 210),
                    );
                    bar_gauge(
                        ui,
                        "Temp",
                        hcm_temp,
                        120.0,
                        "°C",
                        w,
                        if hcm_temp > 90.0 {
                            Color32::RED
                        } else {
                            Color32::from_rgb(80, 180, 255)
                        },
                    );
                    digital_readout(ui, "Mode", &hcm_mode, Color32::from_gray(200));
                    digital_readout(ui, "Power", &format!("{:.1} kW", hcm_pwr), Color32::YELLOW);
                    ui.separator();
                    ui.label(
                        RichText::new("LOADER:")
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    // Loader lift slider
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new("Lift")
                                .size(11.0)
                                .color(Color32::from_gray(150)),
                        );
                        let fill_w = (loader_lift as f32 / 100.0 * 80.0).max(2.0);
                        let (bar, _) =
                            ui.allocate_exact_size(Vec2::new(80.0, 14.0), Sense::hover());
                        let p = ui.painter_at(bar);
                        p.rect_filled(bar, 3.0, Color32::from_gray(22));
                        p.rect_filled(
                            Rect::from_min_size(bar.min, Vec2::new(fill_w, bar.height())),
                            3.0,
                            Color32::from_rgb(80, 180, 255),
                        );
                        ui.label(
                            RichText::new(format!("{:.0}%", loader_lift))
                                .size(10.5)
                                .color(Color32::LIGHT_BLUE),
                        );
                    });
                    let mut ll = self.bench.loader_lift_cmd as f32;
                    ui.add_sized(
                        [ui.available_width() - 10.0, 20.0],
                        Slider::new(&mut ll, -1.0..=1.0).text("Lift cmd"),
                    );
                    if ll as f64 != self.bench.loader_lift_cmd {
                        self.cmds.push(Cmd::SetLoaderLift(ll as f64));
                    }
                    let mut lt = self.bench.loader_tilt_cmd as f32;
                    ui.add_sized(
                        [ui.available_width() - 10.0, 20.0],
                        Slider::new(&mut lt, -1.0..=1.0).text("Tilt cmd"),
                    );
                    if lt as f64 != self.bench.loader_tilt_cmd {
                        self.cmds.push(Cmd::SetLoaderTilt(lt as f64));
                    }
                    ui.separator();
                    ui.label(
                        RichText::new("AUX HYDRAULIC VALVES:")
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    for i in 0..4 {
                        let cmd_val = self.aux_cmds[i];
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(format!("Bank{}:", i))
                                    .size(11.0)
                                    .color(Color32::from_gray(155)),
                            );
                            // - button
                            if ui.small_button("◀").clicked() {
                                self.cmds.push(Cmd::SetAuxValve(i, -1.0));
                            }
                            if ui.small_button("■").clicked() {
                                self.cmds.push(Cmd::SetAuxValve(i, 0.0));
                            }
                            if ui.small_button("▶").clicked() {
                                self.cmds.push(Cmd::SetAuxValve(i, 1.0));
                            }
                            let c = if cmd_val.abs() > 0.05 {
                                Color32::GREEN
                            } else {
                                Color32::from_gray(80)
                            };
                            ui.label(
                                RichText::new(format!(
                                    "{} {:.0}L/m {:.0}bar",
                                    aux_dirs[i], aux_flows[i], aux_pres[i]
                                ))
                                .size(10.5)
                                .monospace()
                                .color(c),
                            );
                        });
                    }
                });

            // ── Implement info ────────────────────────────────────────────────
            let ui = &mut cols[2];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("ATTACHED IMPLEMENT")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    let imp = &self.bench.implement;
                    digital_readout(
                        ui,
                        "Type",
                        &imp.implement_attached
                            .as_ref()
                            .map_or("None".into(), |t| format!("{}", t)),
                        Color32::YELLOW,
                    );
                    digital_readout(
                        ui,
                        "Width",
                        &format!("{:.1} m", imp.implement_width_m),
                        Color32::from_gray(200),
                    );
                    digital_readout(
                        ui,
                        "Depth",
                        &format!("{:.1} cm", imp.implement_working_depth_cm),
                        Color32::from_gray(200),
                    );
                    digital_readout(
                        ui,
                        "ISOBUS",
                        &if imp.isobus_connected {
                            "CONNECTED"
                        } else {
                            "disconnected"
                        },
                        if imp.isobus_connected {
                            Color32::GREEN
                        } else {
                            Color32::from_gray(100)
                        },
                    );
                    ui.separator();
                    ui.label(
                        RichText::new("FRONT PTO:")
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    digital_readout(
                        ui,
                        "Front RPM",
                        &format!("{:.0}", imp.pto_front_rpm),
                        if imp.pto_front_enabled {
                            Color32::GREEN
                        } else {
                            Color32::from_gray(100)
                        },
                    );
                    let fen = imp.pto_front_enabled;
                    if ui
                        .add(
                            Button::new(
                                RichText::new(if fen { "Front PTO ON" } else { "Front PTO OFF" })
                                    .size(11.0),
                            )
                            .fill(if fen {
                                Color32::from_rgb(10, 80, 10)
                            } else {
                                Color32::from_gray(35)
                            }),
                        )
                        .clicked()
                    {
                        self.bench.implement.pto_front_enabled = !fen;
                    }
                    ui.separator();
                    // Hitch control mode
                    ui.label(
                        RichText::new("HITCH CONTROL MODE:")
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    for (mode_s, mode) in [
                        ("Position", &auto_breaking::implement::HitchMode::Position),
                        ("Draft", &auto_breaking::implement::HitchMode::Draft),
                        ("Mixed", &auto_breaking::implement::HitchMode::Mixed),
                        ("Float", &auto_breaking::implement::HitchMode::Float),
                    ] {
                        let cur = hitch_mode.trim() == format!("{}", mode).trim();
                        let fill = if cur {
                            Color32::from_rgb(25, 75, 25)
                        } else {
                            Color32::from_gray(35)
                        };
                        if ui
                            .add(
                                Button::new(RichText::new(mode_s).size(11.0))
                                    .fill(fill)
                                    .min_size(Vec2::new(90.0, 22.0)),
                            )
                            .clicked()
                        {
                            self.bench.implement.hitch_control_mode = *mode;
                        }
                    }
                });
        });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB PARAMS — edit simulation parameters
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_params(&mut self, ui: &mut Ui) {
        ui.label(
            RichText::new("SIMULATION PARAMETERS — edit and apply directly to running simulation")
                .size(12.0)
                .color(Color32::from_rgb(80, 155, 255)),
        );
        if self.params_dirty {
            ui.label(
                RichText::new("⚠ Modified — values are overriding simulation")
                    .size(11.0)
                    .color(Color32::YELLOW),
            );
        }
        ui.separator();

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.columns(3, |cols| {
                    // ── Fuel & DEF ──────────────────────────────────────────────
                    egui::Frame::group(&cols[0].style())
                        .fill(Color32::from_gray(15))
                        .show(&mut cols[0], |ui| {
                            ui.label(
                                RichText::new("FUEL / DEF")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            ui.add_space(4.0);
                            let (mut v, mut changed) = (self.p_fuel as f32, false);
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new("Fuel Level %")
                                        .size(11.0)
                                        .color(Color32::YELLOW),
                                );
                            });
                            if ui
                                .add_sized(
                                    [ui.available_width() - 10.0, 22.0],
                                    Slider::new(&mut v, 0.0..=100.0).suffix(" %"),
                                )
                                .changed()
                            {
                                self.p_fuel = v as f64;
                                changed = true;
                            }
                            if changed {
                                self.cmds.push(Cmd::SetFuelLevel(self.p_fuel));
                                self.params_dirty = true;
                            }

                            let (mut v, mut changed) = (self.p_def as f32, false);
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new("DEF / AdBlue %")
                                        .size(11.0)
                                        .color(Color32::from_rgb(0, 210, 210)),
                                );
                            });
                            if ui
                                .add_sized(
                                    [ui.available_width() - 10.0, 22.0],
                                    Slider::new(&mut v, 0.0..=100.0).suffix(" %"),
                                )
                                .changed()
                            {
                                self.p_def = v as f64;
                                changed = true;
                            }
                            if changed {
                                self.cmds.push(Cmd::SetDefLevel(self.p_def));
                                self.params_dirty = true;
                            }

                            let (mut v, mut changed) = (self.p_engine_h as f32, false);
                            ui.horizontal(|ui| {
                                ui.label(
                                    RichText::new("Engine Hours")
                                        .size(11.0)
                                        .color(Color32::from_gray(200)),
                                );
                            });
                            if ui
                                .add_sized(
                                    [ui.available_width() - 10.0, 22.0],
                                    Slider::new(&mut v, 0.0..=100000.0).suffix(" h"),
                                )
                                .changed()
                            {
                                self.p_engine_h = v as f64;
                                changed = true;
                            }
                            if changed {
                                self.cmds.push(Cmd::SetEngineHours(self.p_engine_h));
                                self.params_dirty = true;
                            }

                            ui.add_space(8.0);
                            if ui
                                .add(
                                    Button::new(RichText::new("↺ Sync from simulation").size(11.0))
                                        .fill(Color32::from_gray(40)),
                                )
                                .clicked()
                            {
                                self.params_dirty = false;
                            } // triggers sync_params_from_bench next tick
                        });

                    // ── Temperatures ────────────────────────────────────────────
                    egui::Frame::group(&cols[1].style())
                        .fill(Color32::from_gray(15))
                        .show(&mut cols[1], |ui| {
                            ui.label(
                                RichText::new("TEMPERATURES & PRESSURES")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            ui.add_space(4.0);

                            let (mut v, mut changed) = (self.p_coolant as f32, false);
                            ui.label(
                                RichText::new("Coolant Temp °C")
                                    .size(11.0)
                                    .color(Color32::from_rgb(255, 130, 30)),
                            );
                            if ui
                                .add_sized(
                                    [ui.available_width() - 10.0, 22.0],
                                    Slider::new(&mut v, -40.0..=130.0).suffix(" °C"),
                                )
                                .changed()
                            {
                                self.p_coolant = v as f64;
                                changed = true;
                            }
                            if changed {
                                self.cmds.push(Cmd::SetCoolantTemp(self.p_coolant));
                                self.params_dirty = true;
                            }

                            let (mut v, mut changed) = (self.p_oil_prs as f32, false);
                            ui.label(
                                RichText::new("Oil Pressure kPa")
                                    .size(11.0)
                                    .color(Color32::GREEN),
                            );
                            if ui
                                .add_sized(
                                    [ui.available_width() - 10.0, 22.0],
                                    Slider::new(&mut v, 0.0..=800.0).suffix(" kPa"),
                                )
                                .changed()
                            {
                                self.p_oil_prs = v as f64;
                                changed = true;
                            }
                            if changed {
                                self.cmds.push(Cmd::SetOilPressure(self.p_oil_prs));
                                self.params_dirty = true;
                            }

                            let (mut v, mut changed) = (self.p_boost as f32, false);
                            ui.label(
                                RichText::new("Boost Pressure kPa")
                                    .size(11.0)
                                    .color(Color32::from_rgb(180, 80, 255)),
                            );
                            if ui
                                .add_sized(
                                    [ui.available_width() - 10.0, 22.0],
                                    Slider::new(&mut v, 0.0..=350.0).suffix(" kPa"),
                                )
                                .changed()
                            {
                                self.p_boost = v as f64;
                                changed = true;
                            }
                            if changed {
                                self.cmds.push(Cmd::SetBoostPressure(self.p_boost));
                                self.params_dirty = true;
                            }

                            ui.add_space(8.0);
                            ui.label(
                                RichText::new(
                                    "Simulate sensor faults by moving sliders to extreme values.",
                                )
                                .size(10.0)
                                .color(Color32::from_gray(140))
                                .italics(),
                            );
                        });

                    // ── Transmission & BCM ──────────────────────────────────────
                    egui::Frame::group(&cols[2].style())
                        .fill(Color32::from_gray(15))
                        .show(&mut cols[2], |ui| {
                            ui.label(
                                RichText::new("TRANSMISSION CONTROLS")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            ui.separator();
                            ui.horizontal(|ui| {
                                if ui.button("◀ DN").clicked() {
                                    self.cmds.push(Cmd::ManualDn);
                                }
                                if ui.button("▶ UP").clicked() {
                                    self.cmds.push(Cmd::ManualUp);
                                }
                                if ui
                                    .add(Button::new(RichText::new("AUTO").size(11.0)).fill(
                                        if self.bench.tcm.auto_mode == AutoShiftMode::Auto {
                                            Color32::from_rgb(20, 80, 20)
                                        } else {
                                            Color32::from_gray(35)
                                        },
                                    ))
                                    .clicked()
                                {
                                    self.cmds.push(Cmd::ToggleAutoShift);
                                }
                                if ui
                                    .add(Button::new(RichText::new("CREEP").size(11.0)).fill(
                                        if self.bench.tcm.creeper_engaged {
                                            Color32::from_rgb(80, 50, 10)
                                        } else {
                                            Color32::from_gray(35)
                                        },
                                    ))
                                    .clicked()
                                {
                                    self.cmds.push(Cmd::ToggleCreeper);
                                }
                            });
                            ui.separator();
                            ui.label(
                                RichText::new("BCM CONTROLS")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            ui.horizontal_wrapped(|ui| {
                                if ui.button("Work Lights").clicked() {
                                    self.cmds.push(Cmd::ToggleWorkLights);
                                }
                                if ui.button("Road Lights").clicked() {
                                    self.cmds.push(Cmd::ToggleRoadLights);
                                }
                                if ui.button("Beacon").clicked() {
                                    self.cmds.push(Cmd::ToggleBeacon);
                                }
                                if ui.button("Honk 📯").clicked() {
                                    self.cmds.push(Cmd::Honk);
                                }
                                if ui.button("Wiper").clicked() {
                                    self.cmds.push(Cmd::CycleWiper);
                                }
                            });
                            ui.separator();
                            ui.label(
                                RichText::new("BCM STATUS:")
                                    .size(11.0)
                                    .color(Color32::from_gray(140)),
                            );
                            let b = &self.bench.bcm;
                            bar_gauge(
                                ui,
                                "Battery",
                                b.battery_voltage - 9.0,
                                7.0,
                                "V",
                                110.0,
                                if b.battery_voltage < 11.5 {
                                    Color32::RED
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "SoC",
                                &format!("{:.0}%", b.battery_soc_pct),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Load",
                                &format!("{:.0} A", b.total_load_amps),
                                Color32::from_gray(180),
                            );
                            digital_readout(
                                ui,
                                "Wiper",
                                &format!("{:?}", b.wiper_speed),
                                Color32::from_gray(180),
                            );
                            for fuse in &b.fuses {
                                if fuse.blown {
                                    ui.label(
                                        RichText::new(format!("💥 FUSE BLOWN: {}", fuse.id))
                                            .size(10.5)
                                            .color(Color32::RED),
                                    );
                                }
                            }
                        });
                });
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB LEAK LAB
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_leak_lab(&mut self, ui: &mut Ui) {
        let circuits: Vec<(usize, String, String, String, String)> = self
            .bench
            .leak_rig
            .circuits
            .iter()
            .enumerate()
            .map(|(i, c)| {
                (
                    i,
                    c.name.clone(),
                    c.application.clone(),
                    format!("{:?}", c.component),
                    format!("{:?}", c.oil_type),
                )
            })
            .collect();
        let reports = self.bench.leak_reports.clone();
        let alerts = self.bench.leak_rig.alerts.clone();
        let predictions = self.leak_predictions.clone();

        ui.horizontal(|ui| {
            ui.label(
                RichText::new("LEAK PHYSICS LAB — industrial manual + automatic workflow")
                    .size(12.0)
                    .color(Color32::from_rgb(80, 155, 255))
                    .strong(),
            );
            ui.separator();
            ui.label(
                RichText::new(format!("{} circuits", circuits.len()))
                    .size(10.5)
                    .color(Color32::from_gray(150)),
            );
            ui.separator();
            ui.label(
                RichText::new(format!(
                    "Total leak {:.3} L/min",
                    self.bench.leak_rig.total_leak_lpm
                ))
                .size(10.5)
                .color(Color32::YELLOW),
            );
            if !self.leak_note.is_empty() {
                ui.separator();
                ui.label(
                    RichText::new(&self.leak_note)
                        .size(10.5)
                        .color(Color32::LIGHT_BLUE),
                );
            }
        });
        ui.separator();

        let mut do_apply = false;
        let mut do_predict = false;
        let mut do_add_custom = false;
        let mut do_export_report_csv = false;
        let mut do_export_report_json = false;
        let mut do_export_pred_csv = false;
        let mut do_export_pred_json = false;
        let mut do_monte_carlo = false;
        let mut pending_select: Option<usize> = None;

        ScrollArea::vertical().auto_shrink([false, false]).max_height(ui.available_height()).show(ui, |ui| {
            ui.columns(3, |cols| {
                // Left: circuit list + runtime status
                egui::Frame::group(&cols[0].style()).fill(Color32::from_gray(15)).show(&mut cols[0], |ui| {
                    ui.label(RichText::new("RUNTIME CIRCUITS").size(11.5).color(Color32::from_rgb(80,155,255)));
                    ui.separator();
                    for (i, name, app, comp, oil) in &circuits {
                        let sel = *i == self.leak_sel_idx;
                        let fill = if sel { Color32::from_rgb(30,60,100) } else { Color32::from_gray(25) };
                        if ui.add(Button::new(RichText::new(format!("{} [{} / {}]", name, comp, oil)).size(10.5)).fill(fill).min_size(Vec2::new(ui.available_width()-8.0,22.0))).clicked() {
                            pending_select = Some(*i);
                        }
                        ui.label(RichText::new(app).size(9.8).color(Color32::from_gray(155)));
                    }
                    ui.separator();
                    ui.label(RichText::new("REAL-TIME REPORT").size(11.0).color(Color32::from_rgb(80,155,255)));
                    for r in &reports {
                        let c = match r.alert {
                            auto_breaking::LeakAlertLevel::Normal => Color32::GREEN,
                            auto_breaking::LeakAlertLevel::Watch => Color32::YELLOW,
                            auto_breaking::LeakAlertLevel::Warning => Color32::from_rgb(255,170,60),
                            auto_breaking::LeakAlertLevel::Critical => Color32::RED,
                            auto_breaking::LeakAlertLevel::Ruptured => Color32::from_rgb(255,0,0),
                        };
                        ui.label(RichText::new(format!("{} | {} | leak {:.3} L/min | risk {:.0}%", r.name, r.alert, r.leak_lpm, r.rupture_probability_pct)).size(10.0).color(c));
                    }
                    if !alerts.is_empty() {
                        ui.separator();
                        ui.label(RichText::new("ACTIVE ALERTS").size(11.0).color(Color32::RED));
                        for a in &alerts {
                            ui.label(RichText::new(format!("{}: {}", a.circuit_name, a.message)).size(10.0).color(Color32::RED));
                        }
                    }
                });

                // Middle: manual params
                egui::Frame::group(&cols[1].style()).fill(Color32::from_gray(15)).show(&mut cols[1], |ui| {
                    ui.label(RichText::new("MANUAL ENGINEERING INPUT").size(11.5).color(Color32::from_rgb(80,155,255)));
                    ui.label(RichText::new("Use this for production-calibrated pressure/oring/oil settings").size(9.8).color(Color32::from_gray(140)));
                    ui.separator();

                    const OIL_NAMES: [&str; 7] = ["Hydraulic ISO46", "Hydraulic ISO68", "Engine 15W40", "Engine 10W30", "PAG46", "POE68", "Custom"];
                    ComboBox::from_label("Oil Type")
                        .selected_text(OIL_NAMES[self.leak_manual.oil_type_idx.min(6)])
                        .show_ui(ui, |ui| {
                            for (i, n) in OIL_NAMES.iter().enumerate() {
                                ui.selectable_value(&mut self.leak_manual.oil_type_idx, i, *n);
                            }
                        });

                    ui.horizontal(|ui| {
                        ui.label("Piston bar");
                        ui.add(DragValue::new(&mut self.leak_manual.piston_pressure_bar).speed(0.2).range(0.0..=1000.0));
                        ui.label("Op bar");
                        ui.add(DragValue::new(&mut self.leak_manual.operation_pressure_bar).speed(0.2).range(0.0..=1000.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Min"); ui.add(DragValue::new(&mut self.leak_manual.pressure_min_bar).speed(0.2));
                        ui.label("Mean"); ui.add(DragValue::new(&mut self.leak_manual.pressure_mean_bar).speed(0.2));
                        ui.label("Ideal"); ui.add(DragValue::new(&mut self.leak_manual.pressure_ideal_bar).speed(0.2));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Max"); ui.add(DragValue::new(&mut self.leak_manual.pressure_max_bar).speed(0.2));
                        ui.label("Rupture"); ui.add(DragValue::new(&mut self.leak_manual.pressure_rupture_bar).speed(0.2));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Squeeze %"); ui.add(DragValue::new(&mut self.leak_manual.squeeze_pct).speed(0.1).range(5.0..=45.0));
                        ui.label("Comp set %"); ui.add(DragValue::new(&mut self.leak_manual.compression_set_pct).speed(0.2).range(0.0..=80.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Base leak mm²");
                        ui.add(DragValue::new(&mut self.leak_manual.base_leak_area_mm2).speed(0.0001).range(0.0..=0.1).max_decimals(6));
                    });

                    if ui.add(Button::new(RichText::new("APPLY MANUAL PARAMETERS").size(11.0)).fill(Color32::from_rgb(20,80,30)).min_size(Vec2::new(ui.available_width()-8.0,28.0))).clicked() {
                        do_apply = true;
                    }

                    ui.separator();
                    ui.label(RichText::new("AUTO PREDICTION").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ui.horizontal(|ui| {
                        ui.label("Horizon s");
                        ui.add(DragValue::new(&mut self.leak_horizon_s).speed(5.0).range(60.0..=86400.0));
                        ui.label("dt s");
                        ui.add(DragValue::new(&mut self.leak_scenario_dt).speed(0.01).range(0.01..=1.0));
                    });
                    if ui.add(Button::new(RichText::new("RUN SCENARIO PREDICTION").size(11.0)).fill(Color32::from_rgb(20,50,90)).min_size(Vec2::new(ui.available_width()-8.0,28.0))).clicked() {
                        do_predict = true;
                    }
                    if ui.add(Button::new(RichText::new("RUN MONTE CARLO (120 runs)").size(11.0)).fill(Color32::from_rgb(70,35,90)).min_size(Vec2::new(ui.available_width()-8.0,26.0))).clicked() {
                        do_monte_carlo = true;
                    }
                });

                // Right: custom circuit + scenario table
                egui::Frame::group(&cols[2].style()).fill(Color32::from_gray(15)).show(&mut cols[2], |ui| {
                    ui.label(RichText::new("CUSTOM CIRCUIT BUILDER").size(11.5).color(Color32::from_rgb(80,155,255)));
                    ui.horizontal(|ui| {
                        ui.label("Name");
                        ui.add_sized([120.0, 20.0], TextEdit::singleline(&mut self.leak_custom.name));
                    });
                    ui.horizontal(|ui| {
                        ui.label("App");
                        ui.add_sized([180.0, 20.0], TextEdit::singleline(&mut self.leak_custom.application));
                    });
                    const COMP_NAMES: [&str; 3] = ["O-ring", "Seal", "A/C Hose"];
                    const OIL_NAMES: [&str; 7] = ["Hydraulic ISO46", "Hydraulic ISO68", "Engine 15W40", "Engine 10W30", "PAG46", "POE68", "Custom"];
                    const MAT_NAMES: [&str; 4] = ["NBR", "HNBR", "FKM", "EPDM"];
                    ComboBox::from_label("Component").selected_text(COMP_NAMES[self.leak_custom.component_idx.min(2)]).show_ui(ui, |ui| {
                        for (i, n) in COMP_NAMES.iter().enumerate() { ui.selectable_value(&mut self.leak_custom.component_idx, i, *n); }
                    });
                    ComboBox::from_label("Oil").selected_text(OIL_NAMES[self.leak_custom.oil_type_idx.min(6)]).show_ui(ui, |ui| {
                        for (i, n) in OIL_NAMES.iter().enumerate() { ui.selectable_value(&mut self.leak_custom.oil_type_idx, i, *n); }
                    });
                    ComboBox::from_label("Material").selected_text(MAT_NAMES[self.leak_custom.material_idx.min(3)]).show_ui(ui, |ui| {
                        for (i, n) in MAT_NAMES.iter().enumerate() { ui.selectable_value(&mut self.leak_custom.material_idx, i, *n); }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Seals"); ui.add(DragValue::new(&mut self.leak_custom.seal_count).range(1..=128));
                        ui.label("ShoreA"); ui.add(DragValue::new(&mut self.leak_custom.shore_a).range(40.0..=95.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Piston"); ui.add(DragValue::new(&mut self.leak_custom.piston_pressure_bar).speed(0.2));
                        ui.label("Op"); ui.add(DragValue::new(&mut self.leak_custom.operation_pressure_bar).speed(0.2));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Min"); ui.add(DragValue::new(&mut self.leak_custom.min_bar).speed(0.2));
                        ui.label("Mean"); ui.add(DragValue::new(&mut self.leak_custom.mean_bar).speed(0.2));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Ideal"); ui.add(DragValue::new(&mut self.leak_custom.ideal_bar).speed(0.2));
                        ui.label("Max"); ui.add(DragValue::new(&mut self.leak_custom.max_bar).speed(0.2));
                        ui.label("Rupt"); ui.add(DragValue::new(&mut self.leak_custom.rupture_bar).speed(0.2));
                    });
                    ui.horizontal(|ui| {
                        ui.label("cs mm"); ui.add(DragValue::new(&mut self.leak_custom.cross_section_mm).speed(0.01));
                        ui.label("sq %"); ui.add(DragValue::new(&mut self.leak_custom.squeeze_pct).speed(0.1));
                        ui.label("gap"); ui.add(DragValue::new(&mut self.leak_custom.extrusion_gap_mm).speed(0.01));
                    });
                    ui.horizontal(|ui| {
                        ui.label("comp set"); ui.add(DragValue::new(&mut self.leak_custom.compression_set_pct).speed(0.1));
                        ui.label("life h"); ui.add(DragValue::new(&mut self.leak_custom.design_life_hours).speed(50.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("base leak"); ui.add(DragValue::new(&mut self.leak_custom.base_leak_area_mm2).speed(0.0001).max_decimals(6));
                        ui.label("Cd"); ui.add(DragValue::new(&mut self.leak_custom.discharge_coeff).speed(0.01).range(0.05..=1.0));
                    });
                    ui.horizontal(|ui| {
                        ui.label("Reservoir L"); ui.add(DragValue::new(&mut self.leak_custom.reservoir_volume_l).speed(0.5));
                        ui.label("Support LPM"); ui.add(DragValue::new(&mut self.leak_custom.support_lpm).speed(0.5));
                    });
                    if ui.add(Button::new(RichText::new("ADD CUSTOM CIRCUIT").size(11.0)).fill(Color32::from_rgb(60,40,10)).min_size(Vec2::new(ui.available_width()-8.0,26.0))).clicked() {
                        do_add_custom = true;
                    }

                    ui.separator();
                    ui.label(RichText::new("SCENARIO RANKING").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ScrollArea::vertical().max_height(190.0).auto_shrink([false, false]).show(ui, |ui| {
                        for p in predictions.iter().take(50) {
                            let c = match p.final_alert {
                                auto_breaking::LeakAlertLevel::Normal => Color32::GREEN,
                                auto_breaking::LeakAlertLevel::Watch => Color32::YELLOW,
                                auto_breaking::LeakAlertLevel::Warning => Color32::from_rgb(255,170,60),
                                auto_breaking::LeakAlertLevel::Critical => Color32::RED,
                                auto_breaking::LeakAlertLevel::Ruptured => Color32::from_rgb(255,0,0),
                            };
                            ui.label(RichText::new(format!("{} | {} | {} | risk {:.0}% | pmax {:.1}bar", p.circuit_name, p.scenario_name, p.final_alert, p.final_rupture_probability_pct, p.peak_pressure_bar)).size(9.8).color(c));
                            ui.label(RichText::new(format!("mode: {} | ttf: {}", p.likely_failure_mode, p.hours_to_rupture.map(|h| format!("{:.1}h", h)).unwrap_or_else(|| "n/a".into()))).size(9.4).color(Color32::from_gray(165)));
                        }
                    });
                    ui.separator();
                    ui.label(RichText::new("EXPORT REPORTS").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("CSV Runtime").clicked() { do_export_report_csv = true; }
                        if ui.button("JSON Runtime").clicked() { do_export_report_json = true; }
                        if ui.button("CSV Prediction").clicked() { do_export_pred_csv = true; }
                        if ui.button("JSON Prediction").clicked() { do_export_pred_json = true; }
                    });
                });
            });
        });

        if let Some(i) = pending_select {
            self.cmds.push(Cmd::LeakSelectCircuit(i));
        }
        if do_apply {
            self.cmds.push(Cmd::LeakApplyManual);
        }
        if do_predict {
            self.cmds.push(Cmd::LeakPredictScenarios);
        }
        if do_add_custom {
            self.cmds.push(Cmd::LeakAddCustomCircuit);
        }
        if do_export_report_csv {
            self.cmds.push(Cmd::LeakExportReportCsv);
        }
        if do_export_report_json {
            self.cmds.push(Cmd::LeakExportReportJson);
        }
        if do_export_pred_csv {
            self.cmds.push(Cmd::LeakExportPredCsv);
        }
        if do_export_pred_json {
            self.cmds.push(Cmd::LeakExportPredJson);
        }
        if do_monte_carlo {
            self.cmds.push(Cmd::LeakRunMonteCarlo);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB SENSORS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_sensors(&self, ui: &mut Ui) {
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.columns(2, |cols| {
                    // GPS
                    egui::Frame::group(&cols[0].style())
                        .fill(Color32::from_gray(15))
                        .show(&mut cols[0], |ui| {
                            let g = &self.bench.gps;
                            ui.label(
                                RichText::new("🛰 GPS/GNSS — u-blox ZED-F9P")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            let fc = match g.fix_quality {
                                auto_breaking::gps::GpsFixQuality::RtkFix => Color32::GREEN,
                                auto_breaking::gps::GpsFixQuality::DgpsFix => {
                                    Color32::from_rgb(100, 220, 100)
                                }
                                auto_breaking::gps::GpsFixQuality::SpsFix => Color32::YELLOW,
                                _ => Color32::RED,
                            };
                            ui.label(
                                RichText::new(format!("Fix: {}", g.fix_quality))
                                    .size(12.0)
                                    .color(fc)
                                    .strong(),
                            );
                            digital_readout(
                                ui,
                                "Position",
                                &g.position_string(),
                                Color32::LIGHT_BLUE,
                            );
                            digital_readout(
                                ui,
                                "Altitude",
                                &format!("{:.1} m MSL", g.altitude_msl),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Speed",
                                &format!("{:.2} km/h  {:.1}°", g.speed_kmh, g.course_deg),
                                Color32::GREEN,
                            );
                            digital_readout(
                                ui,
                                "HDOP",
                                &format!("{:.2}  PDOP:{:.2}", g.hdop, g.pdop),
                                Color32::from_gray(180),
                            );
                            digital_readout(
                                ui,
                                "Sats",
                                &format!("{} used / {} view", g.sats_used, g.sats_in_view),
                                Color32::YELLOW,
                            );
                            digital_readout(
                                ui,
                                "H-Acc",
                                &format!("{:.2} m (1σ)", g.hacc_m),
                                Color32::from_rgb(100, 200, 100),
                            );
                            digital_readout(ui, "UTC", &g.utc_time, Color32::from_gray(155));
                            ui.separator();
                            ui.label(
                                RichText::new("LATEST NMEA:")
                                    .size(10.5)
                                    .color(Color32::from_gray(130)),
                            );
                            for nmea in g.nmea_queue.iter().take(4) {
                                ui.label(
                                    RichText::new(nmea.trim())
                                        .size(9.5)
                                        .monospace()
                                        .color(Color32::from_gray(180)),
                                );
                            }
                            ui.separator();
                            ui.label(
                                RichText::new("SATELLITES:")
                                    .size(10.5)
                                    .color(Color32::from_gray(130)),
                            );
                            for sat in &g.satellites {
                                let c = if !sat.used {
                                    Color32::from_gray(55)
                                } else if sat.snr > 35.0 {
                                    Color32::GREEN
                                } else {
                                    Color32::YELLOW
                                };
                                let bar: String =
                                    "█".repeat(((sat.snr / 52.0 * 12.0) as usize).min(12));
                                ui.label(
                                    RichText::new(format!(
                                        "PRN{:02} {} El:{:2.0}° Az:{:3.0}° SNR:{:4.1} {} {}",
                                        sat.prn,
                                        sat.constellation,
                                        sat.elevation,
                                        sat.azimuth,
                                        sat.snr,
                                        bar,
                                        if sat.used { "✓" } else { "·" }
                                    ))
                                    .size(9.5)
                                    .monospace()
                                    .color(c),
                                );
                            }
                        });
                    // IMU
                    egui::Frame::group(&cols[1].style())
                        .fill(Color32::from_gray(15))
                        .show(&mut cols[1], |ui| {
                            let imu = &self.bench.imu;
                            ui.label(
                                RichText::new("📐 IMU — Madgwick AHRS 9-DOF")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            ui.columns(3, |c| {
                                egui::Frame::dark_canvas(&c[0].style()).show(&mut c[0], |ui| {
                                    arc_gauge(
                                        ui,
                                        imu.roll_deg,
                                        -180.0,
                                        180.0,
                                        "ROLL",
                                        "°",
                                        95.0,
                                        25.0,
                                        45.0,
                                        Some(-25.0),
                                    )
                                });
                                egui::Frame::dark_canvas(&c[1].style()).show(&mut c[1], |ui| {
                                    arc_gauge(
                                        ui,
                                        imu.pitch_deg,
                                        -90.0,
                                        90.0,
                                        "PITCH",
                                        "°",
                                        95.0,
                                        15.0,
                                        30.0,
                                        Some(-15.0),
                                    )
                                });
                                egui::Frame::dark_canvas(&c[2].style()).show(&mut c[2], |ui| {
                                    arc_gauge(
                                        ui,
                                        imu.yaw_deg,
                                        0.0,
                                        360.0,
                                        "YAW/HDG",
                                        "°",
                                        95.0,
                                        0.0,
                                        0.0,
                                        None,
                                    )
                                });
                            });
                            let w = 110.0;
                            ui.label(
                                RichText::new("ACCELEROMETER m/s²:")
                                    .size(10.5)
                                    .color(Color32::from_gray(130)),
                            );
                            bar_gauge(
                                ui,
                                "Ax (fwd)",
                                imu.accel_x,
                                20.0,
                                "m/s²",
                                w,
                                Color32::from_rgb(80, 180, 255),
                            );
                            bar_gauge(
                                ui,
                                "Ay (lat)",
                                imu.accel_y.abs(),
                                20.0,
                                "m/s²",
                                w,
                                Color32::from_rgb(80, 180, 255),
                            );
                            bar_gauge(
                                ui,
                                "Az (up)",
                                imu.accel_z.abs(),
                                15.0,
                                "m/s²",
                                w,
                                Color32::from_rgb(80, 180, 255),
                            );
                            ui.label(
                                RichText::new("G-FORCES:")
                                    .size(10.5)
                                    .color(Color32::from_gray(130)),
                            );
                            bar_gauge(
                                ui,
                                "Long G",
                                imu.longitudinal_g.abs(),
                                1.5,
                                "g",
                                w,
                                Color32::RED,
                            );
                            bar_gauge(ui, "Lat G", imu.lateral_g.abs(), 1.5, "g", w, Color32::RED);
                            digital_readout(
                                ui,
                                "IMU Temp",
                                &format!("{:.1}°C", imu.temperature_c),
                                Color32::from_gray(180),
                            );
                            if imu.accel_fault {
                                ui.label(
                                    RichText::new("⚠ ACCEL FAULT")
                                        .size(11.0)
                                        .color(Color32::RED),
                                );
                            }
                            if imu.gyro_fault {
                                ui.label(
                                    RichText::new("⚠ GYRO FAULT").size(11.0).color(Color32::RED),
                                );
                            }
                            ui.separator();
                            // RADAR summary
                            let r = &self.bench.radar;
                            ui.label(
                                RichText::new("📡 RADAR 77GHz")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            let ttc_col = if r.ttc_front < 2.0 {
                                Color32::RED
                            } else if r.ttc_front < 4.0 {
                                Color32::YELLOW
                            } else {
                                Color32::GREEN
                            };
                            digital_readout(
                                ui,
                                "Front TTC",
                                &format!("{:.1}s", r.ttc_front.min(99.9)),
                                ttc_col,
                            );
                            digital_readout(
                                ui,
                                "Lead dist",
                                &format!("{:.1}m", r.closest_front_m.min(999.0)),
                                Color32::LIGHT_BLUE,
                            );
                            digital_readout(
                                ui,
                                "BSM L/R",
                                &format!(
                                    "{}/{}",
                                    if r.bsm_left { "⚠" } else { "✓" },
                                    if r.bsm_right { "⚠" } else { "✓" }
                                ),
                                if r.bsm_left || r.bsm_right {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "Targets",
                                &format!("{}", r.total_targets()),
                                Color32::from_gray(180),
                            );
                            for t in r.front_center.targets.iter().take(5) {
                                let c = if t.ttc_s < 2.0 {
                                    Color32::RED
                                } else if t.ttc_s < 4.0 {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                };
                                ui.label(
                                    RichText::new(format!(
                                        "ID{:02} {:5.1}m Az:{:+.1}° {:.1}m/s TTC:{:.1}s {}",
                                        t.id,
                                        t.range_m,
                                        t.azimuth_deg,
                                        -t.range_rate_ms,
                                        t.ttc_s.min(99.9),
                                        t.object_class
                                    ))
                                    .size(10.0)
                                    .monospace()
                                    .color(c),
                                );
                            }
                            ui.separator();
                            let l = &self.bench.lidar;
                            ui.label(
                                RichText::new("🔴 LIDAR VLP-16")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            digital_readout(
                                ui,
                                "Pts/scan",
                                &format!("{}", l.points_per_scan),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Clusters",
                                &format!("{}", l.obstacles_seen),
                                Color32::YELLOW,
                            );
                            for cl in l.clusters.iter().take(4) {
                                ui.label(
                                    RichText::new(format!(
                                        "ID{:03} {} {:.1}m  {:.1}×{:.1}m  {}pts",
                                        cl.id,
                                        cl.object_type,
                                        cl.distance_m,
                                        cl.length_m,
                                        cl.width_m,
                                        cl.point_count
                                    ))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(190)),
                                );
                            }
                        });
                });
            });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB AUTONOMOUS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_autonomous(&mut self, ui: &mut Ui) {
        // All display snapshots
        let eng = self.bench.ad.engaged;
        let lvl_s = format!("{}", self.bench.ad.sae_level);
        let acc_s = self.bench.ad.acc_state;
        let acc_spd = self.bench.ad.acc_set_speed_kmh;
        let acc_hdw = self.bench.ad.acc_headway_s;
        let aeb_s = self.bench.ad.aeb_state;
        let aeb_br = self.bench.ad.aeb_brake_cmd;
        let aeb_pc = self.bench.ad.aeb_pre_charge;
        let lka_s = self.bench.ad.lka_state;
        let lka_st = self.bench.ad.lka_steer_cmd;
        let tja_s = self.bench.ad.tja_state;
        let ttc = self.bench.ad.ttc_s;
        let thw = self.bench.ad.thw_s;
        let lead = self.bench.ad.lead_range_m;
        let lead_v = self.bench.ad.lead_speed_ms;
        let bsml = self.bench.ad.bsm_left;
        let bsmr = self.bench.ad.bsm_right;
        let thr_c = self.bench.ad.throttle_cmd;
        let brk_c = self.bench.ad.brake_cmd;
        let str_c = self.bench.ad.steer_cmd_deg;
        let det_s = self.bench.ad.lane.detected;
        let conf_s = self.bench.ad.lane.confidence;
        let off_s = self.bench.ad.lane.offset_m;
        let w_s = self.bench.ad.lane.lane_width_m;
        let lt_s = format!("{}", self.bench.ad.lane.left_type);
        let rt_s = format!("{}", self.bench.ad.lane.right_type);
        let safe_s = format!("{}", self.bench.ad.safety_status);
        let deg_r = self.bench.ad.degrade_reason;
        let drows = self.bench.ad.drowsiness_pct;
        let hands = self.bench.ad.hands_off_s;
        let alert = self.bench.ad.driver_alert;
        let _ego_spd = self.bench.tcm.ground_speed_kmh;
        let path_v: Vec<_> = self
            .bench
            .ad
            .planned_path
            .iter()
            .map(|w| (w.x, w.y))
            .collect();

        let mut do_engage = false;
        let mut do_lka = false;
        let mut do_acc_up = false;
        let mut do_acc_dn = false;

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.columns(3, |cols| {
                    // ── Left: Controls ──────────────────────────────────────────
                    let ui = &mut cols[0];
                    egui::Frame::group(ui.style())
                        .fill(Color32::from_gray(15))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new("🤖 AD CONTROLLER")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            let lc = if lvl_s.contains("L2") {
                                Color32::GREEN
                            } else if lvl_s.contains("L3") {
                                Color32::from_rgb(80, 220, 255)
                            } else {
                                Color32::YELLOW
                            };
                            ui.label(
                                RichText::new(format!("SAE {}", lvl_s))
                                    .size(14.0)
                                    .color(lc)
                                    .strong(),
                            );
                            let fill = if eng {
                                Color32::from_rgb(100, 20, 20)
                            } else {
                                Color32::from_rgb(20, 80, 20)
                            };
                            if ui
                                .add(
                                    Button::new(
                                        RichText::new(if eng {
                                            "⏹ DISENGAGE AD"
                                        } else {
                                            "▶ ENGAGE AD"
                                        })
                                        .size(13.0),
                                    )
                                    .fill(fill)
                                    .min_size(Vec2::new(180.0, 34.0)),
                                )
                                .clicked()
                            {
                                do_engage = true;
                            }
                            ui.separator();
                            // ACC
                            let ac = match acc_s {
                                FeatureState::Active => Color32::GREEN,
                                FeatureState::Ready => Color32::YELLOW,
                                _ => Color32::from_gray(80),
                            };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("ACC:").size(12.0).color(ac).strong());
                                ui.label(
                                    RichText::new(format!("{}  Set:{:.0}km/h", acc_s, acc_spd))
                                        .size(11.0)
                                        .color(ac),
                                );
                                if ui.small_button("+5").clicked() {
                                    do_acc_up = true;
                                }
                                if ui.small_button("-5").clicked() {
                                    do_acc_dn = true;
                                }
                            });
                            bar_gauge(
                                ui,
                                "Set spd",
                                acc_spd,
                                150.0,
                                "km/h",
                                120.0,
                                Color32::LIGHT_BLUE,
                            );
                            bar_gauge(ui, "Headway", acc_hdw, 4.0, "s", 120.0, Color32::GREEN);
                            ui.separator();
                            // AEB
                            let ab = match aeb_s {
                                FeatureState::Active => Color32::RED,
                                FeatureState::Ready => Color32::GREEN,
                                _ => Color32::from_gray(80),
                            };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("AEB:").size(12.0).color(ab).strong());
                                ui.label(RichText::new(format!("{}", aeb_s)).size(11.0).color(ab));
                                if aeb_pc {
                                    ui.label(
                                        RichText::new("PRE-CHG").size(10.5).color(Color32::YELLOW),
                                    );
                                }
                                ui.label(
                                    RichText::new(format!("{:.0}%", aeb_br * 100.0))
                                        .size(11.0)
                                        .color(Color32::RED),
                                );
                            });
                            ui.separator();
                            // LKA
                            let lc2 = match lka_s {
                                FeatureState::Active => Color32::GREEN,
                                _ => Color32::from_gray(80),
                            };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("LKA:").size(12.0).color(lc2).strong());
                                ui.label(RichText::new(format!("{}", lka_s)).size(11.0).color(lc2));
                                if ui.small_button("Toggle").clicked() {
                                    do_lka = true;
                                }
                            });
                            digital_readout(
                                ui,
                                "Steer cmd",
                                &format!("{:+.2}°", lka_st),
                                Color32::YELLOW,
                            );
                            let tj = match tja_s {
                                FeatureState::Active => Color32::GREEN,
                                _ => Color32::from_gray(80),
                            };
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("TJA:").size(12.0).color(tj).strong());
                                ui.label(RichText::new(format!("{}", tja_s)).size(11.0).color(tj));
                            });
                        });

                    // ── Centre: 2D path canvas ─────────────────────────────────
                    let ui = &mut cols[1];
                    egui::Frame::group(ui.style())
                        .fill(Color32::from_gray(12))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new("PATH PLANNING — Click to add waypoints")
                                    .size(11.0)
                                    .color(Color32::from_gray(150)),
                            );
                            let (rect, resp) = ui.allocate_exact_size(
                                Vec2::new(ui.available_width(), ui.available_height() - 80.0),
                                Sense::click(),
                            );
                            let p = ui.painter_at(rect);
                            p.rect_filled(rect, 4.0, Color32::from_gray(10));
                            // Grid lines
                            let cx = rect.center().x;
                            let cy = rect.center().y;
                            let scale = 3.0_f32;
                            for i in -5..=5 {
                                let x = cx + i as f32 * 10.0 * scale;
                                let y = cy + i as f32 * 10.0 * scale;
                                p.line_segment(
                                    [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                                    Stroke::new(0.5, Color32::from_gray(30)),
                                );
                                p.line_segment(
                                    [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                                    Stroke::new(0.5, Color32::from_gray(30)),
                                );
                            }
                            // Scale label
                            p.text(
                                Pos2::new(rect.left() + 4.0, rect.bottom() - 14.0),
                                Align2::LEFT_BOTTOM,
                                "Grid: 10m",
                                FontId::proportional(9.0),
                                Color32::from_gray(80),
                            );
                            // Ego vehicle
                            p.rect_filled(
                                Rect::from_center_size(Pos2::new(cx, cy), Vec2::new(8.0, 14.0)),
                                2.0,
                                Color32::LIGHT_BLUE,
                            );
                            p.text(
                                Pos2::new(cx, cy - 20.0),
                                Align2::CENTER_CENTER,
                                "🚜",
                                FontId::proportional(14.0),
                                Color32::WHITE,
                            );
                            // Planned path (from AD system)
                            if !path_v.is_empty() {
                                let pts: Vec<Pos2> = path_v
                                    .iter()
                                    .map(|(x, y)| {
                                        Pos2::new(cx + *y as f32 * scale, cy - *x as f32 * scale)
                                    })
                                    .collect();
                                for w in pts.windows(2) {
                                    p.line_segment(
                                        [w[0], w[1]],
                                        Stroke::new(1.5, Color32::from_rgb(0, 200, 100)),
                                    );
                                }
                            }
                            // User waypoints
                            let mut add_wp: Option<[f64; 2]> = None;
                            for (i, wp) in self.ad_waypoints.iter().enumerate() {
                                let px = cx + wp[1] as f32 * scale;
                                let py = cy - wp[0] as f32 * scale;
                                p.circle_filled(
                                    Pos2::new(px, py),
                                    5.0,
                                    Color32::from_rgb(255, 150, 30),
                                );
                                p.text(
                                    Pos2::new(px + 6.0, py),
                                    Align2::LEFT_CENTER,
                                    format!("W{}", i),
                                    FontId::proportional(9.0),
                                    Color32::from_rgb(255, 150, 30),
                                );
                            }
                            // Connect waypoints
                            let all_pts: Vec<Pos2> = self
                                .ad_waypoints
                                .iter()
                                .map(|wp| {
                                    Pos2::new(cx + wp[1] as f32 * scale, cy - wp[0] as f32 * scale)
                                })
                                .collect();
                            for w in all_pts.windows(2) {
                                p.line_segment(
                                    [w[0], w[1]],
                                    Stroke::new(1.5, Color32::from_rgb(255, 100, 0)),
                                );
                            }
                            // Click to add waypoint
                            if resp.clicked() {
                                if let Some(pos) = resp.interact_pointer_pos() {
                                    let world_x = (cy - pos.y) as f64 / scale as f64;
                                    let world_y = (pos.x - cx) as f64 / scale as f64;
                                    add_wp = Some([world_x, world_y]);
                                }
                            }
                            if let Some(wp) = add_wp {
                                self.cmds.push(Cmd::AddWaypoint(wp[0], wp[1]));
                            }
                            // Buttons below canvas
                            ui.horizontal(|ui| {
                                if ui.button("🗑 Clear Path").clicked() {
                                    self.cmds.push(Cmd::ClearWaypoints);
                                }
                                ui.label(
                                    RichText::new(format!("{} waypoints", self.ad_waypoints.len()))
                                        .size(11.0)
                                        .color(Color32::from_gray(150)),
                                );
                            });
                        });

                    // ── Right: Sensor fusion + safety ──────────────────────────
                    let ui = &mut cols[2];
                    egui::Frame::group(ui.style())
                        .fill(Color32::from_gray(15))
                        .show(ui, |ui| {
                            ui.label(
                                RichText::new("CONTROL OUTPUTS")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            bar_gauge(
                                ui,
                                "Throttle",
                                thr_c * 100.0,
                                100.0,
                                "%",
                                120.0,
                                Color32::YELLOW,
                            );
                            bar_gauge(ui, "Brake", brk_c * 100.0, 100.0, "%", 120.0, Color32::RED);
                            digital_readout(
                                ui,
                                "Steer",
                                &format!("{:+.1}°", str_c),
                                Color32::LIGHT_BLUE,
                            );
                            ui.separator();
                            ui.label(
                                RichText::new("SENSOR FUSION:")
                                    .size(11.0)
                                    .color(Color32::from_gray(140)),
                            );
                            let tc = if ttc < 2.0 {
                                Color32::RED
                            } else if ttc < 4.0 {
                                Color32::YELLOW
                            } else {
                                Color32::GREEN
                            };
                            digital_readout(ui, "TTC front", &format!("{:.1}s", ttc.min(99.9)), tc);
                            digital_readout(
                                ui,
                                "THW front",
                                &format!("{:.1}s", thw.min(99.9)),
                                Color32::LIGHT_BLUE,
                            );
                            digital_readout(
                                ui,
                                "Lead dist",
                                &format!("{:.1}m", lead.min(999.0)),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "Lead spd",
                                &format!("{:.1}m/s", lead_v),
                                Color32::from_gray(200),
                            );
                            digital_readout(
                                ui,
                                "BSM L",
                                &if bsml { "⚠ OBJECT" } else { "clear  " },
                                if bsml {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "BSM R",
                                &if bsmr { "⚠ OBJECT" } else { "clear  " },
                                if bsmr {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            ui.separator();
                            ui.label(
                                RichText::new("LANE INFO:")
                                    .size(11.0)
                                    .color(Color32::from_gray(140)),
                            );
                            digital_readout(
                                ui,
                                "Detected",
                                &if det_s { "YES" } else { "NO" },
                                if det_s { Color32::GREEN } else { Color32::RED },
                            );
                            digital_readout(
                                ui,
                                "Conf",
                                &format!("{:.0}%", conf_s * 100.0),
                                if conf_s > 0.7 {
                                    Color32::GREEN
                                } else {
                                    Color32::YELLOW
                                },
                            );
                            digital_readout(
                                ui,
                                "Offset",
                                &format!("{:+.3}m", off_s),
                                if off_s.abs() > 0.8 {
                                    Color32::RED
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "Width",
                                &format!("{:.1}m", w_s),
                                Color32::from_gray(180),
                            );
                            digital_readout(ui, "L-mark", &lt_s, Color32::from_gray(180));
                            digital_readout(ui, "R-mark", &rt_s, Color32::from_gray(180));
                            ui.separator();
                            ui.label(
                                RichText::new("DRIVER MONITORING:")
                                    .size(11.0)
                                    .color(Color32::from_gray(140)),
                            );
                            bar_gauge(
                                ui,
                                "Drowsy",
                                drows,
                                100.0,
                                "%",
                                120.0,
                                if drows > 70.0 {
                                    Color32::RED
                                } else if drows > 40.0 {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "Hands off",
                                &format!("{:.0}s", hands),
                                Color32::from_gray(180),
                            );
                            if alert {
                                ui.label(
                                    RichText::new("⚠ DRIVER ATTENTION!")
                                        .size(12.0)
                                        .color(Color32::RED)
                                        .strong(),
                                );
                            }
                            ui.separator();
                            let sc = if safe_s.contains("Normal") {
                                Color32::GREEN
                            } else if safe_s.contains("Degraded") {
                                Color32::YELLOW
                            } else {
                                Color32::RED
                            };
                            ui.label(
                                RichText::new(format!("SAFETY: {}", safe_s))
                                    .size(12.0)
                                    .color(sc)
                                    .strong(),
                            );
                            if let Some(r) = deg_r {
                                ui.label(
                                    RichText::new(format!("→ {}", r))
                                        .size(10.5)
                                        .color(Color32::RED),
                                );
                            }
                        });
                });
            });

        // Deferred mutations
        if do_engage {
            self.cmds
                .push(if eng { Cmd::DisengageAD } else { Cmd::EngageAD });
        }
        if do_lka {
            self.cmds.push(Cmd::ToggleLka);
        }
        if do_acc_up {
            self.cmds.push(Cmd::AccSpeedSet(acc_spd + 5.0));
        }
        if do_acc_dn {
            self.cmds.push(Cmd::AccSpeedSet(acc_spd - 5.0));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB V2X
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_v2x(&mut self, ui: &mut Ui) {
        let v_dsrc = self.bench.v2x.dsrc_active;
        let v_cv2x = self.bench.v2x.cv2x_active;
        let v_rng = self.bench.v2x.range_m;
        let v_loss = self.bench.v2x.packet_loss_pct;
        let v_lat = self.bench.v2x.latency_ms;
        let v_tx = self.bench.v2x.bsm_tx_count;
        let v_rx = self.bench.v2x.rx_count;
        let fca = self.bench.v2x.forward_collision_alert;
        let intx = self.bench.v2x.intersection_alert;
        let emrg = self.bench.v2x.emergency_vehicle_alert;
        let haz = self.bench.v2x.road_hazard_alert;
        let gps_lat = self.bench.gps.latitude_deg;
        let gps_lon = self.bench.gps.longitude_deg;
        let nearby: Vec<_> = self
            .bench
            .v2x
            .nearby_vehicles
            .iter()
            .map(|v| {
                (
                    v.vehicle_id,
                    v.lat_deg,
                    v.lon_deg,
                    v.speed_ms,
                    v.heading_deg,
                )
            })
            .collect();
        let spat: Vec<_> = self
            .bench
            .v2x
            .spat_messages
            .iter()
            .map(|s| {
                (
                    s.intersection_id,
                    s.phase_state,
                    s.time_to_change_s,
                    s.distance_m,
                )
            })
            .collect();
        let wz: Vec<_> = self
            .bench
            .v2x
            .work_zones
            .iter()
            .map(|w| (w.description.clone(), w.distance_m, w.speed_limit_kmh))
            .collect();
        let tel_conn = self.bench.telematics.connect_state;
        let tel_bars = self.bench.telematics.signal_bars;
        let tel_rsrp = self.bench.telematics.rsrp_dbm;
        let tel_ping = self.bench.telematics.ping_ms;
        let tel_dl = self.bench.telematics.download_kbps;
        let tel_ul = self.bench.telematics.upload_kbps;
        let tel_data = self.bench.telematics.data_used_mb;
        let tel_srv = self.bench.telematics.fleet_server.clone();
        let tel_vid = self.bench.telematics.vehicle_id.clone();
        let tel_sync = self.bench.telematics.last_sync_s;
        let tel_gf = self.bench.telematics.inside_geofence;
        let tel_ota_a = self.bench.telematics.ota_available;
        let tel_ota_d = self.bench.telematics.ota_downloading;
        let tel_ota_v = self.bench.telematics.ota_version.clone();
        let tel_ota_mb = self.bench.telematics.ota_size_mb;
        let tel_ota_p = self.bench.telematics.ota_progress_pct;
        let tel_cmds: Vec<_> = self
            .bench
            .telematics
            .pending_commands
            .iter()
            .map(|c| (format!("{}", c.cmd_type), c.id, c.params.clone()))
            .collect();
        let tel_evs: Vec<_> = self
            .bench
            .telematics
            .events
            .iter()
            .map(|e| {
                (
                    e.timestamp,
                    e.ack,
                    e.sent,
                    format!("{:?}", e.event_type),
                    e.description[..e.description.len().min(60)].to_string(),
                )
            })
            .collect();
        let tel_total = self.bench.telematics.total_events_sent;
        let elapsed = self.bench.elapsed;
        let mut do_ota = false;

        ui.columns(2, |cols| {
            let ui = &mut cols[0];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("📶 V2X — DSRC 5.9GHz + C-V2X")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    ui.horizontal(|ui| {
                        let dc = if v_dsrc {
                            Color32::GREEN
                        } else {
                            Color32::from_gray(60)
                        };
                        let cc = if v_cv2x {
                            Color32::GREEN
                        } else {
                            Color32::from_gray(60)
                        };
                        ui.label(
                            RichText::new(if v_dsrc { "DSRC:ON" } else { "DSRC:off" })
                                .size(11.0)
                                .color(dc),
                        );
                        ui.label(
                            RichText::new(if v_cv2x { "C-V2X:ON" } else { "C-V2X:off" })
                                .size(11.0)
                                .color(cc),
                        );
                        ui.label(
                            RichText::new(format!(
                                "{:.0}m  Loss:{:.0}%  Lat:{:.0}ms",
                                v_rng, v_loss, v_lat
                            ))
                            .size(11.0)
                            .color(Color32::from_gray(180)),
                        );
                    });
                    digital_readout(
                        ui,
                        "TX/RX",
                        &format!("{}/{}", v_tx, v_rx),
                        Color32::from_gray(180),
                    );
                    ui.separator();
                    for (a, l, c) in [
                        (fca, "⚠ FORWARD COLLISION", Color32::RED),
                        (intx, "🚦 INTERSECTION RED", Color32::RED),
                        (emrg, "🚨 EMERGENCY VEH", Color32::from_rgb(255, 150, 0)),
                        (haz, "🚧 ROAD HAZARD", Color32::YELLOW),
                    ] {
                        if a {
                            ui.label(RichText::new(l).size(12.0).color(c).strong());
                        }
                    }
                    ui.separator();
                    ui.label(
                        RichText::new("V2V NEARBY:")
                            .size(10.5)
                            .color(Color32::from_gray(130)),
                    );
                    ScrollArea::vertical()
                        .max_height(130.0)
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            for (vid, vlat, vlon, vspd, vhdg) in &nearby {
                                let d = ((*vlat - gps_lat).powi(2) + (*vlon - gps_lon).powi(2))
                                    .sqrt()
                                    * 111320.0;
                                ui.label(
                                    RichText::new(format!(
                                        "ID:{:04X} {:5.0}m {:.1}m/s Hdg:{:.0}°",
                                        vid, d, vspd, vhdg
                                    ))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(190)),
                                );
                            }
                        });
                    ui.separator();
                    ui.label(
                        RichText::new("SPaT SIGNALS:")
                            .size(10.5)
                            .color(Color32::from_gray(130)),
                    );
                    for (id, ph, ttc, dist) in &spat {
                        let c = match ph {
                            auto_breaking::v2x_telematics::TrafficPhase::Green => Color32::GREEN,
                            auto_breaking::v2x_telematics::TrafficPhase::Yellow => Color32::YELLOW,
                            auto_breaking::v2x_telematics::TrafficPhase::Red => Color32::RED,
                            _ => Color32::GRAY,
                        };
                        ui.horizontal(|ui| {
                            ui.label(
                                RichText::new(format!("{:04X}", id))
                                    .size(11.0)
                                    .color(Color32::from_gray(180)),
                            );
                            ui.label(
                                RichText::new(format!("{}", ph))
                                    .size(13.0)
                                    .color(c)
                                    .strong(),
                            );
                            ui.label(
                                RichText::new(format!("{:.1}s  {:.0}m", ttc, dist))
                                    .size(11.0)
                                    .color(Color32::from_gray(180)),
                            );
                        });
                    }
                    ui.separator();
                    for (d, wd, spd) in &wz {
                        let c = if *wd < 100.0 {
                            Color32::RED
                        } else if *wd < 300.0 {
                            Color32::YELLOW
                        } else {
                            Color32::from_gray(180)
                        };
                        ui.label(
                            RichText::new(format!("{} — {:.0}m — {:.0}km/h", d, wd, spd))
                                .size(10.5)
                                .color(c),
                        );
                    }
                });

            let ui = &mut cols[1];
            egui::Frame::group(ui.style())
                .fill(Color32::from_gray(15))
                .show(ui, |ui| {
                    ui.label(
                        RichText::new("📡 TELEMATICS")
                            .size(12.0)
                            .color(Color32::from_rgb(80, 155, 255)),
                    );
                    let cc = match tel_conn {
                        ConnectState::Connected5G => Color32::GREEN,
                        ConnectState::Connected4G => Color32::from_rgb(100, 220, 100),
                        ConnectState::Connecting => Color32::YELLOW,
                        ConnectState::Offline => Color32::RED,
                    };
                    ui.horizontal(|ui| {
                        ui.label(
                            RichText::new(format!("{}", tel_conn))
                                .size(13.0)
                                .color(cc)
                                .strong(),
                        );
                        for _ in 0..tel_bars {
                            ui.label(RichText::new("▌").size(13.0).color(cc));
                        }
                        for _ in tel_bars..5 {
                            ui.label(RichText::new("▌").size(13.0).color(Color32::from_gray(35)));
                        }
                    });
                    digital_readout(ui, "Server", &tel_srv, Color32::from_gray(180));
                    digital_readout(ui, "Vehicle", &tel_vid, Color32::YELLOW);
                    digital_readout(
                        ui,
                        "RSRP",
                        &format!("{:.0} dBm", tel_rsrp),
                        Color32::from_gray(180),
                    );
                    digital_readout(
                        ui,
                        "Ping",
                        &format!("{:.0} ms", tel_ping),
                        Color32::from_gray(180),
                    );
                    digital_readout(
                        ui,
                        "DL/UL",
                        &format!("{:.0}/{:.0} kbps", tel_dl, tel_ul),
                        Color32::from_gray(180),
                    );
                    digital_readout(
                        ui,
                        "Data",
                        &format!("{:.2} MB", tel_data),
                        Color32::from_gray(180),
                    );
                    digital_readout(
                        ui,
                        "Last sync",
                        &format!("{:.0}s ago", elapsed - tel_sync),
                        Color32::from_gray(155),
                    );
                    if let Some(gf) = tel_gf {
                        ui.label(
                            RichText::new(format!("📍 Inside Geofence #{}", gf))
                                .size(11.0)
                                .color(Color32::GREEN),
                        );
                    }
                    if tel_ota_a || tel_ota_d {
                        ui.separator();
                        ui.label(
                            RichText::new("OTA UPDATE:")
                                .size(11.0)
                                .color(Color32::from_rgb(80, 155, 255)),
                        );
                        ui.label(
                            RichText::new(format!("{} ({:.1}MB)", tel_ota_v, tel_ota_mb))
                                .size(11.0)
                                .color(Color32::YELLOW),
                        );
                        if tel_ota_d {
                            bar_gauge(
                                ui,
                                "Progress",
                                tel_ota_p,
                                100.0,
                                "%",
                                130.0,
                                Color32::from_rgb(0, 210, 210),
                            );
                        } else if ui
                            .add(
                                Button::new(RichText::new("⬇ Install").size(12.0))
                                    .fill(Color32::from_rgb(25, 70, 130)),
                            )
                            .clicked()
                        {
                            do_ota = true;
                        }
                        ui.separator();
                    }
                    if !tel_cmds.is_empty() {
                        ui.label(
                            RichText::new("PENDING COMMANDS:")
                                .size(10.5)
                                .color(Color32::YELLOW),
                        );
                        for (ct, id, p) in &tel_cmds {
                            ui.label(
                                RichText::new(format!("→ {} [{}] {}", ct, id, p))
                                    .size(10.5)
                                    .color(Color32::YELLOW),
                            );
                        }
                    }
                    ui.separator();
                    ui.label(
                        RichText::new(format!("EVENTS ({}):", tel_total))
                            .size(11.0)
                            .color(Color32::from_gray(140)),
                    );
                    ScrollArea::vertical()
                        .max_height(200.0)
                        .auto_shrink([false, false])
                        .stick_to_bottom(true)
                        .show(ui, |ui| {
                            for (ts, ack, sent, et, desc) in &tel_evs {
                                let c = if et.contains("Dtc") {
                                    Color32::RED
                                } else if et.contains("Speed") {
                                    Color32::YELLOW
                                } else if et.contains("Ota") {
                                    Color32::from_rgb(0, 210, 210)
                                } else {
                                    Color32::from_gray(190)
                                };
                                let a = if *ack {
                                    "✓"
                                } else if *sent {
                                    "→"
                                } else {
                                    "?"
                                };
                                ui.label(
                                    RichText::new(format!("{:.0}s [{}] {} {}", ts, a, et, desc))
                                        .size(10.0)
                                        .monospace()
                                        .color(c),
                                );
                            }
                        });
                });
        });
        if do_ota {
            self.cmds.push(Cmd::StartOta);
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB UDS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_uds(&mut self, ui: &mut Ui) {
        let uds = &self.bench.uds_ecm;
        let sess_s = format!("{}", uds.session);
        let sec_s = format!("{:?}", uds.security);
        let vin_s = uds.vin.clone();
        let sw_s = uds.sw_version.clone();
        let hw_s = uds.hw_version.clone();
        let cal_s = uds.cal_id.clone();
        let ser_s = uds.ecu_serial.clone();
        let idle_s = format!("{:.0}", uds.idle_rpm_cal);
        let rated_s = format!("{:.0}", uds.rated_rpm_cal);
        let torq_s = format!("{:.0} Nm", uds.max_torque_cal);
        let dpf_s = format!("{:.0}%", uds.dpf_regen_threshold);
        let fm_s = ["Standard", "Economy", "Power"][uds.fuel_map_select as usize];
        let uds_events: Vec<_> = uds
            .event_log
            .iter()
            .map(|e| {
                (
                    e.timestamp,
                    e.service,
                    e.detail.clone(),
                    format!("{:?}", e.result),
                )
            })
            .collect();
        let elapsed = self.bench.elapsed;
        let mut send_bytes: Option<(Vec<u8>, u8)> = None;

        ui.columns(2, |cols| {
            let ui = &mut cols[0];
            ui.label(
                RichText::new("UDS ISO 14229 CONSOLE")
                    .size(12.0)
                    .color(Color32::from_rgb(80, 155, 255)),
            );
            ui.separator();
            ui.horizontal(|ui| {
                ui.label(RichText::new("SA:").size(11.0).color(Color32::GRAY));
                let mut sa_s = format!("0x{:02X}", self.uds_sa);
                if ui
                    .add_sized([65.0, 20.0], TextEdit::singleline(&mut sa_s))
                    .changed()
                {
                    if let Ok(v) = u8::from_str_radix(sa_s.trim().trim_start_matches("0x"), 16) {
                        self.uds_sa = v;
                    }
                }
            });
            ui.label(
                RichText::new("Request (hex):")
                    .size(11.0)
                    .color(Color32::GRAY),
            );
            ui.add_sized(
                [ui.available_width() - 10.0, 22.0],
                TextEdit::singleline(&mut self.uds_input).font(FontId::monospace(12.0)),
            );
            ui.label(
                RichText::new("Presets:")
                    .size(11.0)
                    .color(Color32::from_gray(140)),
            );
            ui.horizontal_wrapped(|ui| {
                for (l, h) in [
                    ("ExtSess", "10 02"),
                    ("Prog", "10 03"),
                    ("Default", "10 01"),
                    ("Seed", "27 01"),
                    ("Keep", "3E 00"),
                    ("ClrDTC", "14 FF FF FF"),
                    ("RdDTC", "19 02 FF"),
                    ("RdRPM", "22 DD 01"),
                    ("RdCool", "22 DD 03"),
                    ("RdDPF", "22 DD 08"),
                    ("RdVIN", "22 F1 90"),
                    ("DPFRgn", "31 01 DF 01"),
                ] {
                    if ui.small_button(l).clicked() {
                        self.uds_input = h.into();
                    }
                }
            });
            ui.add_space(4.0);
            if ui
                .add(
                    Button::new(RichText::new("📤 SEND").size(12.0))
                        .fill(Color32::from_rgb(25, 70, 130))
                        .min_size(Vec2::new(120.0, 28.0)),
                )
                .clicked()
            {
                let bytes: Vec<u8> = self
                    .uds_input
                    .split_whitespace()
                    .filter_map(|s| u8::from_str_radix(s, 16).ok())
                    .collect();
                if !bytes.is_empty() {
                    send_bytes = Some((bytes, self.uds_sa));
                }
            }
            if ui.small_button("🗑 Clear").clicked() {
                self.uds_log.clear();
            }
            ui.separator();
            ScrollArea::vertical()
                .max_height(200.0)
                .auto_shrink([false, false])
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for (is_req, line) in &self.uds_log {
                        let c = if line.contains("❌") {
                            Color32::RED
                        } else if !is_req {
                            Color32::GREEN
                        } else {
                            Color32::from_gray(200)
                        };
                        ui.label(RichText::new(line).size(10.5).monospace().color(c));
                    }
                });

            let ui = &mut cols[1];
            ui.label(
                RichText::new("ECM UDS SERVER STATE")
                    .size(12.0)
                    .color(Color32::from_rgb(80, 155, 255)),
            );
            let sc = match sess_s.trim() {
                "DEFAULT" => Color32::from_gray(180),
                "EXTENDED" => Color32::YELLOW,
                _ => Color32::RED,
            };
            digital_readout(ui, "Session", &sess_s, sc);
            digital_readout(
                ui,
                "Security",
                &sec_s,
                if sec_s.contains("Locked") {
                    Color32::from_gray(120)
                } else {
                    Color32::GREEN
                },
            );
            digital_readout(ui, "VIN", &vin_s, Color32::from_rgb(0, 210, 210));
            digital_readout(ui, "SW", &sw_s, Color32::from_gray(200));
            digital_readout(ui, "HW", &hw_s, Color32::from_gray(200));
            digital_readout(ui, "Cal ID", &cal_s, Color32::from_gray(200));
            digital_readout(ui, "Serial", &ser_s, Color32::from_gray(200));
            ui.separator();
            ui.label(
                RichText::new("CALIBRATION:")
                    .size(11.0)
                    .color(Color32::from_gray(140)),
            );
            digital_readout(ui, "Idle RPM", &idle_s, Color32::YELLOW);
            digital_readout(ui, "Rated RPM", &rated_s, Color32::YELLOW);
            digital_readout(ui, "Max Torq", &torq_s, Color32::YELLOW);
            digital_readout(ui, "DPF Regen", &dpf_s, Color32::YELLOW);
            digital_readout(ui, "Fuel Map", fm_s, Color32::YELLOW);
            ui.separator();
            ui.label(
                RichText::new("UDS LOG:")
                    .size(11.0)
                    .color(Color32::from_gray(140)),
            );
            ScrollArea::vertical()
                .max_height(160.0)
                .auto_shrink([false, false])
                .stick_to_bottom(true)
                .show(ui, |ui| {
                    for (ts, svc, det, res) in &uds_events {
                        let c = if res.contains("Positive") {
                            Color32::GREEN
                        } else if res.contains("Security") {
                            Color32::RED
                        } else {
                            Color32::YELLOW
                        };
                        ui.label(
                            RichText::new(format!("{:.2}s 0x{:02X} {}", ts, svc, det))
                                .size(10.5)
                                .monospace()
                                .color(c),
                        );
                    }
                });
        });

        // Execute UDS send AFTER column closure released borrows
        if let Some((bytes, sa)) = send_bytes {
            let ts = elapsed;
            let req_hex = bytes
                .iter()
                .map(|b| format!("{:02X}", b))
                .collect::<Vec<_>>()
                .join(" ");
            let resp = if sa == 0x00 {
                self.bench.uds_ecm.process(&bytes, ts)
            } else if sa == 0x03 {
                self.bench.uds_tcm.process(&bytes, ts)
            } else {
                vec![0x7F, bytes[0], 0x11]
            };
            let resp_hex = resp
                .iter()
                .map(|b| format!("{:02X}", b))
                .collect::<Vec<_>>()
                .join(" ");
            let nrc = resp.first().copied() == Some(0x7F);
            let svc = auto_breaking::uds::UdsServer::service_name(bytes[0]);
            self.uds_log.push((
                true,
                format!("[{:.2}s] 0x{:02X} {} → {}", ts, sa, svc, req_hex),
            ));
            self.uds_log.push((
                false,
                format!(
                    "            RESP {}{}",
                    resp_hex,
                    if nrc { " ❌ NRC" } else { " ✓" }
                ),
            ));
            if self.uds_log.len() > 200 {
                self.uds_log.drain(0..40);
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB PLOTS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_plots(&self, ui: &mut Ui) {
        let h = 155.0_f32;
        // Pre-collect data to avoid borrow conflicts inside columns closure
        let left = [
            (&self.pl_rpm, "RPM", Color32::GREEN, 3000.0f64),
            (
                &self.pl_torque,
                "TORQUE Nm",
                Color32::from_rgb(80, 180, 255),
                1200.0,
            ),
            (
                &self.pl_coolant,
                "COOLANT°C",
                Color32::from_rgb(255, 130, 30),
                130.0,
            ),
            (
                &self.pl_boost,
                "BOOST kPa",
                Color32::from_rgb(180, 80, 255),
                300.0,
            ),
        ];
        let right = [
            (&self.pl_spd, "SPD km/h", Color32::LIGHT_BLUE, 55.0f64),
            (&self.pl_fuel, "FUEL L/h", Color32::YELLOW, 30.0),
            (
                &self.pl_dpf,
                "DPF SOOT%",
                Color32::from_rgb(200, 80, 200),
                100.0,
            ),
            (
                &self.pl_boost,
                "HYD press",
                Color32::from_rgb(0, 210, 210),
                350.0,
            ),
        ];
        // Clone plot data so we can move into closure without borrow issues
        let ld: Vec<(Vec<[f64; 2]>, &str, Color32, f64)> = left
            .iter()
            .map(|(v, l, c, m)| ((*v).clone(), *l, *c, *m))
            .collect();
        let rd: Vec<(Vec<[f64; 2]>, &str, Color32, f64)> = right
            .iter()
            .map(|(v, l, c, m)| ((*v).clone(), *l, *c, *m))
            .collect();
        ui.columns(2, |cols| {
            for (v, lbl, c, y_max) in &ld {
                let last = v.last().map(|p| p[1]).unwrap_or(0.0);
                cols[0].horizontal(|ui| {
                    ui.label(RichText::new(*lbl).size(11.0).color(*c));
                    ui.label(
                        RichText::new(format!("{:.1}", last))
                            .size(13.0)
                            .color(*c)
                            .strong()
                            .monospace(),
                    );
                });
                let pts_c = v.clone();
                let cc = *c;
                Plot::new(*lbl)
                    .height(h)
                    .include_y(0.0)
                    .include_y(*y_max)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .show_axes([false, true])
                    .show(&mut cols[0], |pu| {
                        let pts: PlotPoints = pts_c.iter().map(|p| [p[0], p[1]]).collect();
                        pu.line(Line::new(pts).color(cc).width(1.8));
                    });
                cols[0].add_space(4.0);
            }
            for (v, lbl, c, y_max) in &rd {
                let last = v.last().map(|p| p[1]).unwrap_or(0.0);
                cols[1].horizontal(|ui| {
                    ui.label(RichText::new(*lbl).size(11.0).color(*c));
                    ui.label(
                        RichText::new(format!("{:.1}", last))
                            .size(13.0)
                            .color(*c)
                            .strong()
                            .monospace(),
                    );
                });
                let pts_c = v.clone();
                let cc = *c;
                Plot::new(*lbl)
                    .height(h)
                    .include_y(0.0)
                    .include_y(*y_max)
                    .allow_drag(false)
                    .allow_zoom(false)
                    .allow_scroll(false)
                    .show_axes([false, true])
                    .show(&mut cols[1], |pu| {
                        let pts: PlotPoints = pts_c.iter().map(|p| [p[0], p[1]]).collect();
                        pu.line(Line::new(pts).color(cc).width(1.8));
                    });
                cols[1].add_space(4.0);
            }
        });
    }
}
