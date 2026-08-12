//! Heavy Machinery ECU Bench — GUI v3.0
//! Command-queue architecture: all UI→simulation mutations go through Cmd enum,
//! executed AFTER the UI frame, eliminating all borrow-checker closure conflicts.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
#![allow(float_literal_f32_fallback)]

pub mod io;
mod ecm_mock_provider;
mod widgets;

use auto_breaking::{
    autonomous::FeatureState,
    boot_sequence::EcuBootStage,
    can_gateway::BusState,
    CalibrationReport,
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
use rfd::FileDialog;
use std::collections::{BTreeMap, HashMap, VecDeque};
use std::path::PathBuf;
use std::time::Duration;
use widgets::{arc_gauge, bar_gauge, digital_readout, direction_selector, warning_lamp};

const DT: f64 = 1.0 / 60.0;
const PLOT_WINDOW: usize = 600;
const EVENT_MAX: usize = 500;
type TraceRow = (f64, u32, u8, u8, String, String, String);

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
    LeakExportReportCsv(String),
    LeakExportReportJson(String),
    LeakExportPredCsv(String),
    LeakExportPredJson(String),
    LeakExportCatalogMaterialsCsv(String),
    LeakExportCatalogMaterialsJson(String),
    LeakExportCatalogOilsCsv(String),
    LeakExportCatalogOilsJson(String),
    LeakCalibrateFromCsv(String),
    LeakExportCalibrationCsv(String),
    LeakExportCalibrationJson(String),
    LeakRunMonteCarlo,
    LeakClearScenarioOutputs,
    LeakResetSimulation,
    LeakApplyAndPredict,
    LeakApplyAndMonteCarlo,
    // CAN network controls
    CanInjectBitError(usize),
    CanInjectAckError(usize),
    CanInjectBusOff(usize),
    CanInjectBabbling(usize),
    CanClearBusInjections(usize),
    CanClearAllInjections,
    CanExportSnapshotCsv(String),
    CanExportSnapshotJson(String),
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
    Help,
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
    throttle_target: f32,
    brake_target: f32,
    throttle_cmd_last: f32,
    brake_cmd_last: f32,
    throttle_cmd_last_tick: u64,
    brake_cmd_last_tick: u64,

    // CAN monitor
    can_mode: CanMode,
    can_freeze: bool,
    can_filter: String,
    can_bus_idx: usize,
    can_note: String,
    can_trace_tree: bool,
    can_trace_limit: usize,
    can_signal_limit: usize,
    can_hide_stale: bool,
    can_trace_update_every_ticks: u64,
    can_trace_tick_counter: u64,
    can_trace_pause_on_scroll: bool,
    can_trace_pause_ticks_left: u32,
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
    fault_feedback: String,
    fault_last_dtc_count: usize,

    // Implements operator feedback
    implement_feedback: String,

    // V2X smoothed KPIs
    v2x_range_ema: f64,
    v2x_loss_ema: f64,
    v2x_lat_ema: f64,

    // UDS console
    uds_input: String,
    uds_sa: u8,
    uds_flash_path: String,
    uds_log: Vec<(bool, String)>,

    // Leak physics lab
    leak_sel_idx: usize,
    leak_manual: LeakManualUi,
    leak_custom: LeakCustomUi,
    leak_horizon_s: f64,
    leak_scenario_dt: f64,
    leak_predictions: Vec<ScenarioPrediction>,
    leak_temporal_trace: VecDeque<(f64, f64, f64, f64)>, // (t, pressure, risk, leak)
    leak_calibration_csv_path: String,
    leak_calibration_report: Option<CalibrationReport>,
    leak_note: String,
    leak_view_yaw_deg: f32,
    leak_view_pitch_deg: f32,
    leak_view_zoom: f32,
    leak_timeback_window_s: f64,
    leak_timeback_latest_first: bool,
    leak_timeback_stride: usize,

    // Plots
    pl_rpm: Vec<[f64; 2]>,
    pl_spd: Vec<[f64; 2]>,
    pl_torque: Vec<[f64; 2]>,
    pl_fuel: Vec<[f64; 2]>,
    pl_coolant: Vec<[f64; 2]>,
    pl_dpf: Vec<[f64; 2]>,
    pl_boost: Vec<[f64; 2]>,
    pl_hyd: Vec<[f64; 2]>,

    ticks: u64,
    ux_show_guide: bool,
    ux_compact_mode: bool,
}

impl App {
    fn new(cc: &eframe::CreationContext, hw_cfg: io::hw::HwConfig) -> Self {
        let mut vis = Visuals::dark();
        vis.panel_fill = Color32::from_gray(14);
        cc.egui_ctx.set_visuals(vis);

        let mut bench = HeavyMachinery::new();
        bench.tcm.set_direction(Direction::Forward);
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
            throttle_target: 0.0,
            brake_target: 0.0,
            throttle_cmd_last: 0.0,
            brake_cmd_last: 0.0,
            throttle_cmd_last_tick: 0,
            brake_cmd_last_tick: 0,
            can_mode: CanMode::Signals,
            can_freeze: false,
            can_filter: String::new(),
            sig_map: HashMap::new(),
            trace_snap: Vec::new(),
            can_bus_idx: 0,
            can_note: String::new(),
            can_trace_tree: true,
            can_trace_limit: 500,
            can_signal_limit: 300,
            can_hide_stale: false,
            can_trace_update_every_ticks: 1,
            can_trace_tick_counter: 0,
            can_trace_pause_on_scroll: true,
            can_trace_pause_ticks_left: 0,
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
            fault_feedback: String::new(),
            fault_last_dtc_count: 0,
            implement_feedback: String::new(),
            v2x_range_ema: 300.0,
            v2x_loss_ema: 2.0,
            v2x_lat_ema: 15.0,
            uds_input: "10 02".into(),
            uds_sa: 0x00,
            uds_flash_path: String::new(),
            uds_log: Vec::new(),
            leak_sel_idx: 0,
            leak_manual: LeakManualUi::default(),
            leak_custom: LeakCustomUi::default(),
            leak_horizon_s: 600.0,
            leak_scenario_dt: 0.05,
            leak_predictions: Vec::new(),
            leak_temporal_trace: VecDeque::new(),
            leak_calibration_csv_path: String::new(),
            leak_calibration_report: None,
            leak_note: String::new(),
            leak_view_yaw_deg: 25.0,
            leak_view_pitch_deg: 20.0,
            leak_view_zoom: 1.0,
            leak_timeback_window_s: 120.0,
            leak_timeback_latest_first: false,
            leak_timeback_stride: 1,
            pl_rpm: vec![],
            pl_spd: vec![],
            pl_torque: vec![],
            pl_fuel: vec![],
            pl_coolant: vec![],
            pl_dpf: vec![],
            pl_boost: vec![],
            pl_hyd: vec![],
            ticks: 0,
            ux_show_guide: true,
            ux_compact_mode: false,
        }
    }

    fn pick_save_path(default_name: &str, ext: &str) -> Option<String> {
        let default_dir = PathBuf::from("reports");
        let _ = std::fs::create_dir_all(&default_dir);
        FileDialog::new()
            .set_directory(default_dir)
            .set_file_name(default_name)
            .add_filter(ext, &[ext])
            .save_file()
            .map(|p| p.display().to_string())
    }

    fn pick_open_path(ext: &str) -> Option<String> {
        FileDialog::new()
            .add_filter(ext, &[ext])
            .pick_file()
            .map(|p| p.display().to_string())
    }

    fn refresh_mock_ecm_live_from_sim(&mut self) {
        let e = &self.bench.ecm;
        self.ecm_live_snapshot.engine_speed_rpm = Some(e.rpm);
        self.ecm_live_snapshot.accel_pedal_pct = Some(e.active_throttle);
        self.ecm_live_snapshot.coolant_temp_c = Some(e.coolant_temp_c);
        self.ecm_live_snapshot.fuel_temp_c = Some(e.fuel_temp_c);
        self.ecm_live_snapshot.oil_pressure_kpa = Some(e.oil_pressure_kpa);
        self.ecm_live_snapshot.last_seen_pgn = Some(61444);
        self.ecm_live_snapshot.source_address = Some(0x00);
        self.ecm_live_last_update_ms = now_ms();
    }

    fn ecm_treeview(&self, ui: &mut Ui, mock_mode: bool) {
        ui.separator();
        ui.label(if mock_mode {
            "ECM Explorer (Mock/CAT ET style)"
        } else {
            "ECM Explorer (CAT ET style)"
        });

        let nodes = ecm_mock_provider::build_mock_ecm_tree(&self.bench, &self.ecm_live_snapshot);
        for node in nodes {
            CollapsingHeader::new(format!("{} (SA 0x{:02X})", node.name, node.source_address))
                .default_open(node.source_address == 0x00)
                .show(ui, |ui| {
                    for f in node.functions {
                        CollapsingHeader::new(f.name).default_open(false).show(ui, |ui| {
                            Grid::new(format!("ecm_tree_{}_{}", node.source_address, f.name))
                                .num_columns(3)
                                .show(ui, |ui| {
                                    for p in f.params {
                                        ui.label(p.key);
                                        ui.label(format!("{:.3}", p.value));
                                        ui.label(p.unit);
                                        ui.end_row();
                                    }
                                });
                        });
                    }
                });
        }
    }

    fn oil_type_from_idx(idx: usize) -> OilType {
        OilType::all().get(idx).copied().unwrap_or(OilType::Custom)
    }

    fn component_from_idx(idx: usize) -> CircuitComponent {
        match idx {
            0 => CircuitComponent::Oring,
            1 => CircuitComponent::Seal,
            _ => CircuitComponent::AcHose,
        }
    }

    fn material_from_idx(idx: usize) -> OringMaterial {
        OringMaterial::all()
            .get(idx)
            .copied()
            .unwrap_or(OringMaterial::Nbr)
    }

    fn idx_from_oil_type(o: OilType) -> usize {
        OilType::all().iter().position(|x| *x == o).unwrap_or(0)
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

    fn sanitize_leak_manual(&mut self) {
        self.leak_manual.piston_pressure_bar = self.leak_manual.piston_pressure_bar.clamp(0.0, 1000.0);
        self.leak_manual.operation_pressure_bar = self.leak_manual.operation_pressure_bar.clamp(0.0, 1000.0);
        self.leak_manual.pressure_min_bar = self.leak_manual.pressure_min_bar.max(0.0);
        self.leak_manual.pressure_mean_bar = self
            .leak_manual
            .pressure_mean_bar
            .max(self.leak_manual.pressure_min_bar);
        self.leak_manual.pressure_ideal_bar = self
            .leak_manual
            .pressure_ideal_bar
            .max(self.leak_manual.pressure_mean_bar);
        self.leak_manual.pressure_max_bar = self
            .leak_manual
            .pressure_max_bar
            .max(self.leak_manual.pressure_ideal_bar);
        self.leak_manual.pressure_rupture_bar = self
            .leak_manual
            .pressure_rupture_bar
            .max(self.leak_manual.pressure_max_bar + 0.1);
        self.leak_manual.squeeze_pct = self.leak_manual.squeeze_pct.clamp(5.0, 45.0);
        self.leak_manual.compression_set_pct = self.leak_manual.compression_set_pct.clamp(0.0, 80.0);
        self.leak_manual.base_leak_area_mm2 = self.leak_manual.base_leak_area_mm2.clamp(0.0, 0.1);
    }

    fn sanitize_leak_custom(&mut self) {
        self.leak_custom.name = self.leak_custom.name.trim().to_string();
        self.leak_custom.application = self.leak_custom.application.trim().to_string();
        self.leak_custom.seal_count = self.leak_custom.seal_count.clamp(1, 128);
        self.leak_custom.shore_a = self.leak_custom.shore_a.clamp(40.0, 95.0);
        self.leak_custom.piston_pressure_bar = self.leak_custom.piston_pressure_bar.clamp(0.0, 1000.0);
        self.leak_custom.operation_pressure_bar = self.leak_custom.operation_pressure_bar.clamp(0.0, 1000.0);
        self.leak_custom.min_bar = self.leak_custom.min_bar.max(0.0);
        self.leak_custom.mean_bar = self.leak_custom.mean_bar.max(self.leak_custom.min_bar);
        self.leak_custom.ideal_bar = self.leak_custom.ideal_bar.max(self.leak_custom.mean_bar);
        self.leak_custom.max_bar = self.leak_custom.max_bar.max(self.leak_custom.ideal_bar);
        self.leak_custom.rupture_bar = self.leak_custom.rupture_bar.max(self.leak_custom.max_bar + 0.1);
        self.leak_custom.cross_section_mm = self.leak_custom.cross_section_mm.clamp(0.5, 20.0);
        self.leak_custom.squeeze_pct = self.leak_custom.squeeze_pct.clamp(5.0, 45.0);
        self.leak_custom.extrusion_gap_mm = self.leak_custom.extrusion_gap_mm.clamp(0.01, 3.0);
        self.leak_custom.compression_set_pct = self.leak_custom.compression_set_pct.clamp(0.0, 80.0);
        self.leak_custom.design_life_hours = self.leak_custom.design_life_hours.clamp(1.0, 200000.0);
        self.leak_custom.base_leak_area_mm2 = self.leak_custom.base_leak_area_mm2.clamp(0.0, 0.1);
        self.leak_custom.discharge_coeff = self.leak_custom.discharge_coeff.clamp(0.05, 1.0);
        self.leak_custom.reservoir_volume_l = self.leak_custom.reservoir_volume_l.clamp(0.1, 2000.0);
        self.leak_custom.support_lpm = self.leak_custom.support_lpm.clamp(0.0, 5000.0);
    }

    fn validate_leak_manual(&self) -> Result<(), String> {
        if !(self.leak_manual.pressure_min_bar <= self.leak_manual.pressure_mean_bar
            && self.leak_manual.pressure_mean_bar <= self.leak_manual.pressure_ideal_bar
            && self.leak_manual.pressure_ideal_bar <= self.leak_manual.pressure_max_bar
            && self.leak_manual.pressure_max_bar < self.leak_manual.pressure_rupture_bar)
        {
            return Err("Envelope invalido: requer min <= mean <= ideal <= max < rupture".into());
        }
        if self.leak_manual.base_leak_area_mm2 <= 0.0 {
            return Err("Base leak deve ser > 0".into());
        }
        Ok(())
    }

    fn validate_leak_custom(&self) -> Result<(), String> {
        if self.leak_custom.name.is_empty() {
            return Err("Nome do circuito custom nao pode ser vazio".into());
        }
        if !(self.leak_custom.min_bar <= self.leak_custom.mean_bar
            && self.leak_custom.mean_bar <= self.leak_custom.ideal_bar
            && self.leak_custom.ideal_bar <= self.leak_custom.max_bar
            && self.leak_custom.max_bar < self.leak_custom.rupture_bar)
        {
            return Err("Envelope custom invalido: requer min <= mean <= ideal <= max < rupture".into());
        }
        if self.leak_custom.base_leak_area_mm2 <= 0.0 {
            return Err("Base leak custom deve ser > 0".into());
        }
        Ok(())
    }

    fn build_manual_leak_params(&self) -> ManualCircuitParams {
        ManualCircuitParams {
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
        }
    }

    fn apply_manual_to_selected_circuit(&mut self) -> Result<String, String> {
        self.validate_leak_manual()?;
        let Some(c) = self.bench.leak_rig.circuits.get(self.leak_sel_idx) else {
            return Err("Nenhum circuito selecionado".into());
        };
        let name = c.name.clone();
        let ok = self
            .bench
            .apply_leak_manual_params(&name, self.build_manual_leak_params());
        if ok {
            Ok(name)
        } else {
            Err("Falha no backend ao aplicar parametros".into())
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

    fn parse_u32_filter(input: &str) -> Option<u32> {
        let s = input.trim().to_lowercase();
        if let Some(hex) = s.strip_prefix("0x") {
            u32::from_str_radix(hex, 16).ok()
        } else {
            s.parse::<u32>().ok()
        }
    }

    fn can_filter_match_signal(filter: &str, pgn: u32, sa: u8, sig: &Signal) -> bool {
        let filt = filter.trim().to_lowercase();
        if filt.is_empty() {
            return true;
        }
        if let Some(v) = filt.strip_prefix("sa:") {
            return Self::parse_u32_filter(v) == Some(sa as u32);
        }
        if let Some(v) = filt.strip_prefix("pgn:") {
            return Self::parse_u32_filter(v) == Some(pgn);
        }
        if let Some(v) = filt.strip_prefix("id:") {
            return Self::parse_u32_filter(v) == Some(sig.raw_id);
        }
        sig.pgn_name.to_lowercase().contains(&filt)
            || sig.sa_name.to_lowercase().contains(&filt)
            || format!("{:02x}", sa).contains(&filt)
            || format!("0x{:02x}", sa).contains(&filt)
            || format!("{:05}", pgn).contains(&filt)
            || format!("{:08x}", sig.raw_id).contains(&filt)
            || sig
                .decoded
                .iter()
                .any(|(name, _, _)| name.to_lowercase().contains(&filt))
    }

    fn can_filter_match_trace(filter: &str, row: &TraceRow) -> bool {
        let filt = filter.trim().to_lowercase();
        if filt.is_empty() {
            return true;
        }
        let (_ts, raw_id, sa, _dlc, hex, pgn_sa, decoded) = row;
        if let Some(v) = filt.strip_prefix("sa:") {
            return Self::parse_u32_filter(v) == Some(*sa as u32);
        }
        if let Some(v) = filt.strip_prefix("id:") {
            return Self::parse_u32_filter(v) == Some(*raw_id);
        }
        pgn_sa.to_lowercase().contains(&filt)
            || decoded.to_lowercase().contains(&filt)
            || hex.to_lowercase().contains(&filt)
            || format!("{:02x}", sa).contains(&filt)
            || format!("0x{:02x}", sa).contains(&filt)
            || format!("{:08x}", raw_id).contains(&filt)
    }

    fn ema(prev: f64, value: f64, alpha: f64) -> f64 {
        prev + alpha * (value - prev)
    }

    fn update_v2x_kpis(&mut self) {
        self.v2x_range_ema = Self::ema(self.v2x_range_ema, self.bench.v2x.range_m, 0.10);
        self.v2x_loss_ema = Self::ema(self.v2x_loss_ema, self.bench.v2x.packet_loss_pct, 0.12);
        self.v2x_lat_ema = Self::ema(self.v2x_lat_ema, self.bench.v2x.latency_ms, 0.12);
    }

    fn approach_f32(current: f32, target: f32, max_step: f32) -> f32 {
        if (target - current).abs() <= max_step {
            target
        } else if target > current {
            current + max_step
        } else {
            current - max_step
        }
    }

    fn shape_pedal(x: f32, deadzone: f32) -> f64 {
        let n = ((x - deadzone) / (1.0 - deadzone)).clamp(0.0, 1.0);
        let shaped = n * n * (3.0 - 2.0 * n); // smoothstep
        shaped as f64
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
                    self.throttle_target = 0.0;
                    self.brake_target = 0.0;
                    self.throttle_cmd_last = 0.0;
                    self.brake_cmd_last = 0.0;
                    self.throttle_cmd_last_tick = self.ticks;
                    self.brake_cmd_last_tick = self.ticks;
                }
                Cmd::SetThrottle(v) => {
                    self.throttle_target = v.clamp(0.0, 1.0);
                }
                Cmd::SetBrake(v) => {
                    self.brake_target = v.clamp(0.0, 1.0);
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
                    self.implement_feedback = format!("Rear PTO mode -> {}", m);
                }
                Cmd::SetHitchTarget(v) => {
                    self.bench.implement.hitch_target_pct = v.clamp(0.0, 100.0);
                    self.hitch_target = v;
                    self.implement_feedback =
                        format!("Hitch target set -> {:.1}%", self.hitch_target);
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
                        self.implement_feedback =
                            format!("Aux bank {} command -> {:+.1}", i, self.aux_cmds[i]);
                    }
                }
                Cmd::SetLoaderLift(v) => {
                    self.bench.loader_lift_cmd = v;
                    self.implement_feedback = format!("Loader lift cmd -> {:+.2}", v);
                }
                Cmd::SetLoaderTilt(v) => {
                    self.bench.loader_tilt_cmd = v;
                    self.implement_feedback = format!("Loader tilt cmd -> {:+.2}", v);
                }
                Cmd::SetHitchJoystick(v) => {
                    self.bench.hitch_joystick = v;
                    self.implement_feedback = format!("Hitch joystick -> {:+.1}", v);
                }
                // AD
                Cmd::EngageAD => {
                    self.bench.ad.engage(self.bench.tcm.ground_speed_kmh);
                    if self.bench.tcm.direction == Direction::Neutral
                        || self.bench.tcm.direction == Direction::Park
                    {
                        self.bench.tcm.set_direction(Direction::Forward);
                    }
                    if self.bench.tcm.auto_mode != AutoShiftMode::Auto {
                        self.bench.tcm.toggle_auto();
                    }
                }
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
                    self.fault_feedback =
                        format!("Selected fault: {}", self.bench.selected_fault);
                }
                Cmd::InjectFault => {
                    self.bench.inject_fault();
                    self.fault_last_dtc_count = self.bench.ecm.active_dtcs.len();
                    self.fault_feedback = format!(
                        "Injected {}. Waiting ECM/DM1 propagation...",
                        self.bench.selected_fault
                    );
                }
                Cmd::ClearFaults => {
                    self.bench.clear_faults();
                    self.fault_last_dtc_count = self.bench.ecm.active_dtcs.len();
                    self.fault_feedback = "All injected faults cleared".into();
                }
                // Telematics
                Cmd::StartOta => self.bench.telematics.start_ota(),
                // Leak Lab
                Cmd::LeakSelectCircuit(i) => {
                    self.leak_sel_idx = i.min(self.bench.leak_rig.circuits.len().saturating_sub(1));
                    self.sync_leak_manual_from_selected();
                }
                Cmd::LeakApplyManual => {
                    self.leak_note = match self.apply_manual_to_selected_circuit() {
                        Ok(name) => format!("Parametros aplicados com sucesso em {}", name),
                        Err(e) => format!("Falha ao aplicar parametros: {}", e),
                    };
                }
                Cmd::LeakPredictScenarios => {
                    self.leak_predictions = self.bench.predict_leak_scenarios(
                        self.leak_horizon_s,
                        self.leak_scenario_dt.max(0.01),
                    );
                    self.leak_note = format!("{} cenarios avaliados", self.leak_predictions.len());
                }
                Cmd::LeakAddCustomCircuit => {
                    if let Err(e) = self.validate_leak_custom() {
                        self.leak_note = format!("Falha ao adicionar circuito custom: {}", e);
                        continue;
                    }
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
                Cmd::LeakExportReportCsv(path) => {
                    self.leak_note = match self.bench.export_leak_report_csv(&path) {
                        Ok(_) => format!("CSV salvo em {}", path),
                        Err(e) => format!("Falha export CSV: {}", e),
                    };
                }
                Cmd::LeakExportReportJson(path) => {
                    self.leak_note = match self.bench.export_leak_report_json(&path) {
                        Ok(_) => format!("JSON salvo em {}", path),
                        Err(e) => format!("Falha export JSON: {}", e),
                    };
                }
                Cmd::LeakExportPredCsv(path) => {
                    self.leak_note = match self
                        .bench
                        .export_leak_predictions_csv(&path, &self.leak_predictions)
                    {
                        Ok(_) => format!("Pred CSV salvo em {}", path),
                        Err(e) => format!("Falha export pred CSV: {}", e),
                    };
                }
                Cmd::LeakExportPredJson(path) => {
                    self.leak_note = match self
                        .bench
                        .export_leak_predictions_json(&path, &self.leak_predictions)
                    {
                        Ok(_) => format!("Pred JSON salvo em {}", path),
                        Err(e) => format!("Falha export pred JSON: {}", e),
                    };
                }
                Cmd::LeakExportCatalogMaterialsCsv(path) => {
                    self.leak_note = match self.bench.leak_rig.export_material_catalog_csv(&path) {
                        Ok(_) => format!("Catalogo materiais CSV salvo em {}", path),
                        Err(e) => format!("Falha export catalogo materiais CSV: {}", e),
                    };
                }
                Cmd::LeakExportCatalogMaterialsJson(path) => {
                    self.leak_note =
                        match self.bench.leak_rig.export_material_catalog_json(&path) {
                            Ok(_) => format!("Catalogo materiais JSON salvo em {}", path),
                            Err(e) => format!("Falha export catalogo materiais JSON: {}", e),
                        };
                }
                Cmd::LeakExportCatalogOilsCsv(path) => {
                    self.leak_note = match self.bench.leak_rig.export_oil_catalog_csv(&path) {
                        Ok(_) => format!("Catalogo oleos CSV salvo em {}", path),
                        Err(e) => format!("Falha export catalogo oleos CSV: {}", e),
                    };
                }
                Cmd::LeakExportCatalogOilsJson(path) => {
                    self.leak_note = match self.bench.leak_rig.export_oil_catalog_json(&path) {
                        Ok(_) => format!("Catalogo oleos JSON salvo em {}", path),
                        Err(e) => format!("Falha export catalogo oleos JSON: {}", e),
                    };
                }
                Cmd::LeakCalibrateFromCsv(path) => {
                    self.leak_note = match self.bench.calibrate_leak_model_from_csv(&path) {
                        Ok(report) => {
                            let circuits = report.calibrated_circuits;
                            let samples = report.total_samples;
                            self.leak_calibration_report = Some(report);
                            self.leak_calibration_csv_path = path.clone();
                            format!(
                                "Calibracao concluida: {} circuitos, {} amostras",
                                circuits, samples
                            )
                        }
                        Err(e) => format!("Falha calibracao CSV: {}", e),
                    };
                }
                Cmd::LeakExportCalibrationCsv(path) => {
                    self.leak_note = match self.leak_calibration_report.as_ref() {
                        Some(report) => match self
                            .bench
                            .export_leak_calibration_report_csv(&path, report)
                        {
                            Ok(_) => format!("Calibracao CSV salva em {}", path),
                            Err(e) => format!("Falha export calibracao CSV: {}", e),
                        },
                        None => "Execute calibracao antes de exportar".into(),
                    };
                }
                Cmd::LeakExportCalibrationJson(path) => {
                    self.leak_note = match self.leak_calibration_report.as_ref() {
                        Some(report) => match self
                            .bench
                            .export_leak_calibration_report_json(&path, report)
                        {
                            Ok(_) => format!("Calibracao JSON salva em {}", path),
                            Err(e) => format!("Falha export calibracao JSON: {}", e),
                        },
                        None => "Execute calibracao antes de exportar".into(),
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
                Cmd::LeakClearScenarioOutputs => {
                    self.leak_predictions.clear();
                    self.leak_temporal_trace.clear();
                    self.leak_note = "Saida de simulacao limpa. Pronto para nova rodada.".into();
                }
                Cmd::LeakResetSimulation => {
                    self.bench.leak_rig.reset();
                    self.bench.leak_reports.clear();
                    self.leak_predictions.clear();
                    self.leak_temporal_trace.clear();
                    self.leak_calibration_report = None;
                    self.leak_note =
                        "Leak Lab resetado: estado fisico, alertas, historico e cenarios limpos."
                            .into();
                }
                Cmd::LeakApplyAndPredict => {
                    match self.apply_manual_to_selected_circuit() {
                        Ok(_) => {
                            self.leak_temporal_trace.clear();
                            self.leak_predictions = self.bench.predict_leak_scenarios(
                                self.leak_horizon_s,
                                self.leak_scenario_dt.max(0.01),
                            );
                            self.leak_note =
                                format!("Nova rodada: parametros aplicados + {} cenarios", self.leak_predictions.len());
                        }
                        Err(e) => {
                            self.leak_note = format!("Nova rodada: falha ao aplicar parametros: {}", e);
                        }
                    }
                }
                Cmd::LeakApplyAndMonteCarlo => {
                    match self.apply_manual_to_selected_circuit() {
                        Ok(_) => {
                            self.leak_temporal_trace.clear();
                            self.leak_predictions = self.bench.monte_carlo_leak_predictions(
                                120,
                                self.leak_horizon_s,
                                self.leak_scenario_dt.max(0.01),
                                25.0,
                            );
                            self.leak_note =
                                format!("Nova rodada MC: parametros aplicados + {} amostras", self.leak_predictions.len());
                        }
                        Err(e) => {
                            self.leak_note = format!("Nova rodada MC: falha ao aplicar parametros: {}", e);
                        }
                    }
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
                Cmd::CanExportSnapshotCsv(path) => {
                    self.can_note = match self.bench.export_can_snapshot_csv(&path) {
                        Ok(_) => format!("Snapshot CSV salvo em {}", path),
                        Err(e) => format!("Falha snapshot CSV: {}", e),
                    };
                }
                Cmd::CanExportSnapshotJson(path) => {
                    self.can_note = match self.bench.export_can_snapshot_json(&path) {
                        Ok(_) => format!("Snapshot JSON salvo em {}", path),
                        Err(e) => format!("Falha snapshot JSON: {}", e),
                    };
                }
                // Simulation
                Cmd::Reset => {
                    self.bench.reset();
                    self.bench.tcm.set_direction(Direction::Forward);
                    self.throttle = 0.0;
                    self.brake = 0.0;
                    self.throttle_target = 0.0;
                    self.brake_target = 0.0;
                    self.throttle_cmd_last = 0.0;
                    self.brake_cmd_last = 0.0;
                    self.throttle_cmd_last_tick = self.ticks;
                    self.brake_cmd_last_tick = self.ticks;
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
                        &mut self.pl_hyd,
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
        self.can_trace_tick_counter = self.can_trace_tick_counter.saturating_add(1);
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
        let trace_due = self
            .can_trace_tick_counter
            .is_multiple_of(self.can_trace_update_every_ticks.max(1));
        if self.can_trace_pause_ticks_left > 0 {
            self.can_trace_pause_ticks_left = self.can_trace_pause_ticks_left.saturating_sub(1);
        }
        if !self.can_freeze && trace_due && self.can_trace_pause_ticks_left == 0 {
            self.trace_snap = self
                .bench
                .gateway
                .bus
                .frames
                .iter()
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
                    if self.bench
                        .boot
                        .crank_inhibited { "INHIBITED" } else { "OK" }
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
        if ecm.coolant_temp_c > 102.0 && self.prev_rpm > 400.0
            && (t as u32).is_multiple_of(5) {
                // throttle event rate
                ev!(
                    "ECM",
                    format!("HIGH COOLANT TEMP {:.1}°C", ecm.coolant_temp_c),
                    EventLevel::Critical
                );
            }
        // DEF warning
        if ecm.def_level_pct < 10.0 && ecm.def_level_pct > 0.0
            && (t as u32).is_multiple_of(30) {
                ev!(
                    "SCR",
                    format!(
                        "DEF LOW {:.1}%  NOx={:.0}ppm",
                        ecm.def_level_pct, ecm.nox_tailpipe_ppm
                    ),
                    EventLevel::Warn
                );
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
        if abs.tcs_system_active
            && (t as u32).is_multiple_of(2) {
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
        p!(self.pl_hyd, self.bench.hcm.system_pressure_bar);
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

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
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
        if !self.bench.ad.engaged {
            let thr_step = (DT as f32) * 1.8; // ~0.56s 0->100%
            let brk_step = (DT as f32) * 3.0; // brakes react faster
            self.throttle = Self::approach_f32(self.throttle, self.throttle_target, thr_step);
            self.brake = Self::approach_f32(self.brake, self.brake_target, brk_step);

            let thr_eff = Self::shape_pedal(self.throttle, 0.03);
            let brk_eff = Self::shape_pedal(self.brake, 0.02);
            self.bench.throttle_pct = (thr_eff * 100.0).clamp(0.0, 100.0);
            self.bench.brake_pct = (brk_eff * 100.0).clamp(0.0, 100.0);
        }
        self.auto_sequence();
        self.bench.tick(DT);
        if self.bench.ad.engaged {
            // Mirror AD control outputs back into UI sliders while engaged.
            self.throttle = (self.bench.throttle_pct / 100.0) as f32;
            self.brake = (self.bench.brake_pct / 100.0) as f32;
        }
        if matches!(self.tab, Tab::CanBus)
            && matches!(self.can_mode, CanMode::Trace)
            && self.can_trace_pause_on_scroll
            && !self.can_freeze
        {
            let scroll_delta = ctx.input(|i| i.raw_scroll_delta.length());
            if scroll_delta > 0.0 {
                let hold = (self.can_trace_update_every_ticks.max(1) * 4) as u32;
                self.can_trace_pause_ticks_left = self.can_trace_pause_ticks_left.max(hold);
            }
        }
        if let Some(sel) = self.bench.leak_rig.circuits.get(self.leak_sel_idx) {
            if let Some(rep) = self.bench.leak_reports.iter().find(|r| r.name == sel.name) {
                self.leak_temporal_trace.push_back((
                    self.bench.elapsed,
                    rep.current_pressure_bar,
                    rep.rupture_probability_pct,
                    rep.leak_lpm,
                ));
                if self.leak_temporal_trace.len() > 7200 {
                    self.leak_temporal_trace.pop_front();
                }
            }
        }
        self.ticks += 1;
        self.update_v2x_kpis();
        self.update_signals();
        self.detect_events();
        self.push_plots();
        self.sync_params_from_bench();
        let mut clear_live_feed = false;
        if let Some(feed) = &self.ecm_live_feed {
            self.ecm_live_snapshot = feed.latest_snapshot();
            self.ecm_live_last_update_ms = feed.last_update_ms();
            self.ecm_live_frames_total = feed.frames_total();
            if !feed.is_alive() {
                if let Some(err) = feed.last_error() {
                    self.ecm_live_status = format!("Retrieve stopped with error: {}", err);
                } else {
                    self.ecm_live_status = "Retrieve stopped.".into();
                }
                clear_live_feed = true;
            }
        } else if matches!(self.hw_cfg.mode, io::hw::HwMode::Sim) {
            self.refresh_mock_ecm_live_from_sim();
            self.ecm_live_frames_total = self.ecm_live_frames_total.saturating_add(1);
        }
        if clear_live_feed {
            self.ecm_live_feed = None;
            self.ecm_connected = false;
        }
        ctx.request_repaint();

        // ── Panels ──────────────────────────────────────────────────────────
        TopBottomPanel::top("tb")
            .min_height(56.0)
            .show(ctx, |ui| self.toolbar(ui));
        TopBottomPanel::top("tabs")
            .exact_height(58.0)
            .show(ctx, |ui| self.tab_bar(ui));
        TopBottomPanel::bottom("sb")
            .exact_height(22.0)
            .show(ctx, |ui| self.statusbar(ui));
        CentralPanel::default().show(ctx, |ui| {
            self.render_ux_guide(ui);
            if self.ux_compact_mode {
                ui.spacing_mut().item_spacing = Vec2::new(6.0, 5.0);
            }
            ui.separator();
            match self.tab {
                Tab::Help => ui.push_id("tab::help", |ui| self.tab_help(ui)),
                Tab::Cluster => ui.push_id("tab::cluster", |ui| self.tab_cluster(ui)),
                Tab::CanBus => ui.push_id("tab::can", |ui| self.tab_can(ui)),
                Tab::Events => ui.push_id("tab::events", |ui| self.tab_events(ui)),
                Tab::EcuNet => ui.push_id("tab::ecu_net", |ui| self.tab_ecu_net(ui)),
                Tab::Engine => ui.push_id("tab::engine", |ui| self.tab_engine(ui)),
                Tab::Faults => ui.push_id("tab::faults", |ui| self.tab_faults(ui)),
                Tab::Boot => ui.push_id("tab::boot", |ui| self.tab_boot(ui)),
                Tab::Implements => ui.push_id("tab::implements", |ui| self.tab_implements(ui)),
                Tab::Params => ui.push_id("tab::params", |ui| self.tab_params(ui)),
                Tab::Sensors => ui.push_id("tab::sensors", |ui| self.tab_sensors(ui)),
                Tab::Autonomous => ui.push_id("tab::autonomous", |ui| self.tab_autonomous(ui)),
                Tab::V2X => ui.push_id("tab::v2x", |ui| self.tab_v2x(ui)),
                Tab::Uds => ui.push_id("tab::uds", |ui| self.tab_uds(ui)),
                Tab::EcmLiveData => ui.push_id("tab::ecm_live", |ui| self.tab_ecm_live_data(ui)),
                Tab::LeakLab => ui.push_id("tab::leak", |ui| self.tab_leak_lab(ui)),
                Tab::Plots => ui.push_id("tab::plots", |ui| self.tab_plots(ui)),
            }
        });
    }
}

// ── Tab bar ──────────────────────────────────────────────────────────────────
impl App {
    fn tab_bar(&mut self, ui: &mut Ui) {
        ui.push_id("ui::tab_bar", |ui| {
            let dtcs = self.bench.ecm.active_dtcs.len();
            let nav_sections: [(&str, &[(Tab, &str)]); 2] = [
                (
                    "OPERACAO",
                    &[
                        (Tab::Help, "❓ HELP"),
                        (Tab::Cluster, "🎛 CLUSTER"),
                        (Tab::Engine, "⚙ ENGINE"),
                        (Tab::Implements, "🌾 IMPL"),
                        (Tab::Autonomous, "🤖 AD"),
                        (Tab::LeakLab, "🧪 LEAK LAB"),
                        (Tab::Plots, "📈 PLOTS"),
                    ],
                ),
                (
                    "DIAGNOSTICO",
                    &[
                        (Tab::CanBus, "📡 CAN"),
                        (Tab::Events, "📋 EVENTS"),
                        (Tab::EcuNet, "🔌 ECU NET"),
                        (Tab::Faults, "⚠ FAULTS"),
                        (Tab::Boot, "🔑 BOOT"),
                        (Tab::Params, "🎚 PARAMS"),
                        (Tab::Sensors, "🛰 SENSORS"),
                        (Tab::V2X, "📶 V2X"),
                        (Tab::Uds, "🔧 UDS"),
                        (Tab::EcmLiveData, "🧲 ECM LIVE"),
                    ],
                ),
            ];

            for (section_name, tabs) in nav_sections {
                ui.horizontal_wrapped(|ui| {
                    ui.label(
                        RichText::new(section_name)
                            .size(9.6)
                            .color(Color32::from_gray(145))
                            .strong(),
                    );
                    ui.separator();
                    for (t, lbl) in tabs {
                        let sel = self.tab == *t;
                        let fill = if sel {
                            Color32::from_rgb(45, 105, 210)
                        } else {
                            Color32::from_gray(30)
                        };
                        let label = if *t == Tab::Faults && dtcs > 0 {
                            format!("⚠ FAULTS({})", dtcs)
                        } else {
                            (*lbl).into()
                        };
                        let col = if *t == Tab::Faults && dtcs > 0 {
                            Color32::RED
                        } else if sel {
                            Color32::WHITE
                        } else {
                            Color32::from_gray(160)
                        };
                        if ui
                            .add(Button::new(RichText::new(label).size(10.8).color(col)).fill(fill))
                            .clicked()
                        {
                            self.tab = *t;
                        }
                    }
                });
                if section_name == "OPERACAO" {
                    ui.add_space(2.0);
                }
            }
        });
    }

    fn ad_readiness_issues(&self) -> Vec<String> {
        let mut issues = Vec::new();
        if !self.bench.engine_running() {
            issues.push("Motor desligado: o AD nao consegue acelerar nem sustentar velocidade.".to_string());
        }
        if matches!(self.bench.tcm.direction, Direction::Park) {
            issues.push("Transmissao em Park: selecione Forward para permitir deslocamento.".to_string());
        }
        if self.bench.tcm.auto_mode != AutoShiftMode::Auto {
            issues.push("Transmissao fora de AUTO: o AD perde previsibilidade de troca.".to_string());
        }
        if self.bench.ad.lane.confidence <= 0.4 {
            issues.push("Lane confidence baixa: o LKA permanece em espera abaixo de 40%.".to_string());
        }
        if self.bench.tcm.ground_speed_kmh < 5.0 {
            issues.push("Abaixo de 5 km/h o LKA ainda nao atua; o ACC pode acelerar a maquina.".to_string());
        }
        issues
    }

    fn ux_alerts(&self) -> Vec<(Color32, String)> {
        let mut alerts = Vec::new();
        match self.tab {
            Tab::Cluster | Tab::Engine | Tab::Implements => {
                if !self.bench.engine_running() {
                    alerts.push((
                        Color32::YELLOW,
                        "Motor desligado: esta aba so reflete dinamica real depois de RUN.".to_string(),
                    ));
                }
                if self.bench.ecm.red_lamp {
                    alerts.push((
                        Color32::RED,
                        "RED STOP ativo: existe falha critica impactando a operacao.".to_string(),
                    ));
                }
            }
            Tab::Autonomous => {
                if !self.bench.engine_running() {
                    alerts.push((
                        Color32::YELLOW,
                        "Motor desligado: o AD nao consegue gerar tracao nem validar longitudinal.".to_string(),
                    ));
                }
                if self.bench.ad.engaged {
                    let msg = if let Some(reason) = self.bench.ad.degrade_reason {
                        format!("AD engatado em modo degradado: {}.", reason)
                    } else {
                        "AD engatado: comandos manuais principais ficam travados para evitar disputa.".to_string()
                    };
                    alerts.push((Color32::from_rgb(90, 210, 130), msg));
                }
            }
            Tab::CanBus | Tab::Events | Tab::EcuNet | Tab::Faults | Tab::Uds => {
                if self.bench.gateway.bus_load_pct > 85.0 {
                    alerts.push((
                        Color32::YELLOW,
                        format!("Carga CAN alta ({:.0}%): diagnostico pode ficar ruidoso nesta aba.", self.bench.gateway.bus_load_pct),
                    ));
                }
                if matches!(self.tab, Tab::Faults | Tab::Uds) && self.bench.ecm.red_lamp {
                    alerts.push((
                        Color32::RED,
                        "Falha critica ativa: compare Faults, Events e Engine antes de continuar.".to_string(),
                    ));
                }
            }
            Tab::EcmLiveData => {
                if self.ecm_live_feed.as_ref().is_some_and(|feed| !feed.is_alive()) {
                    alerts.push((
                        Color32::YELLOW,
                        "ECM Live sem stream valido: reconecte ou reinicie Retrieve Data.".to_string(),
                    ));
                }
            }
            Tab::Help | Tab::Boot | Tab::Params | Tab::Sensors | Tab::V2X | Tab::LeakLab | Tab::Plots => {}
        }
        alerts
    }

    fn ux_context_items(&self) -> Vec<(String, Color32)> {
        match self.tab {
            Tab::Help => vec![
                (
                    format!("Workspace {:.1}s", self.bench.elapsed),
                    Color32::from_gray(180),
                ),
                (
                    format!("{} abas operacionais", 17),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("{} DTC ativos", self.bench.ecm.active_dtcs.len()),
                    if self.bench.ecm.active_dtcs.is_empty() {
                        Color32::GREEN
                    } else {
                        Color32::YELLOW
                    },
                ),
            ],
            Tab::Cluster => vec![
                (
                    format!("RPM {:.0}", self.bench.ecm.rpm),
                    Color32::GREEN,
                ),
                (
                    format!("SPD {:.1} km/h", self.bench.tcm.ground_speed_kmh),
                    Color32::LIGHT_BLUE,
                ),
                (
                    format!("Gear {}", self.bench.tcm.gear_label),
                    Color32::YELLOW,
                ),
            ],
            Tab::CanBus => vec![
                (
                    format!("Bus load {:.0}%", self.bench.gateway.bus_load_pct),
                    if self.bench.gateway.bus_load_pct > 85.0 {
                        Color32::RED
                    } else {
                        Color32::GOLD
                    },
                ),
                (
                    format!("Signals {}", self.sig_map.len()),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Trace {}", self.trace_snap.len()),
                    Color32::from_gray(180),
                ),
            ],
            Tab::Events => vec![
                (
                    format!("Eventos {}", self.events.len()),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Filtro {}", if self.ev_filter.is_empty() { "livre" } else { "ativo" }),
                    if self.ev_filter.is_empty() {
                        Color32::from_gray(180)
                    } else {
                        Color32::YELLOW
                    },
                ),
                (
                    format!("Pause {}", if self.ev_pause { "on" } else { "off" }),
                    if self.ev_pause { Color32::YELLOW } else { Color32::GREEN },
                ),
            ],
            Tab::EcuNet => vec![
                (
                    format!(
                        "Online {}/{}",
                        self.bench.boot.ecus.iter().filter(|e| e.is_online()).count(),
                        self.bench.boot.ecus.len()
                    ),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("NM {}", self.bench.net_mgmt.bus_state),
                    Color32::from_gray(180),
                ),
            ],
            Tab::Engine => vec![
                (
                    format!("Load {:.0}%", self.bench.ecm.percent_load),
                    Color32::YELLOW,
                ),
                (
                    format!("Torque {:.0} Nm", self.bench.ecm.actual_torque_nm),
                    Color32::from_rgb(80, 180, 255),
                ),
                (
                    format!("Coolant {:.0} C", self.bench.ecm.coolant_temp_c),
                    if self.bench.ecm.coolant_temp_c > 105.0 {
                        Color32::RED
                    } else {
                        Color32::from_rgb(255, 130, 30)
                    },
                ),
            ],
            Tab::Faults => vec![
                (
                    format!("Active DTC {}", self.bench.ecm.active_dtcs.len()),
                    if self.bench.ecm.active_dtcs.is_empty() {
                        Color32::GREEN
                    } else {
                        Color32::RED
                    },
                ),
                (
                    format!("Selected {}", self.bench.selected_fault),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Fault inject {}", if self.bench.fault_active { "armed" } else { "idle" }),
                    if self.bench.fault_active { Color32::YELLOW } else { Color32::GREEN },
                ),
            ],
            Tab::Boot => vec![
                (
                    format!("Ignition {}", self.bench.ignition()),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Engine {}", if self.bench.engine_running() { "RUN" } else { "OFF" }),
                    if self.bench.engine_running() { Color32::GREEN } else { Color32::YELLOW },
                ),
            ],
            Tab::Implements => vec![
                (
                    format!("PTO {}", if self.bench.implement.pto_rear_enabled { "on" } else { "off" }),
                    if self.bench.implement.pto_rear_enabled { Color32::GREEN } else { Color32::from_gray(170) },
                ),
                (
                    format!("Hyd {:.0} bar", self.bench.hcm.system_pressure_bar),
                    Color32::from_rgb(0, 210, 210),
                ),
            ],
            Tab::Params => vec![
                (
                    "Live tuning panel".to_string(),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Mode {}", if self.ux_compact_mode { "compact" } else { "full" }),
                    Color32::from_gray(180),
                ),
            ],
            Tab::Sensors => vec![
                (
                    format!("GPS {:.1} km/h", self.bench.gps.speed_kmh),
                    Color32::LIGHT_BLUE,
                ),
                (
                    format!("Radar TTC {:.1}s", self.bench.radar.ttc_front.min(99.9)),
                    if self.bench.radar.ttc_front < 2.0 { Color32::RED } else { Color32::YELLOW },
                ),
                (
                    format!("Lane conf {:.0}%", self.bench.ad.lane.confidence * 100.0),
                    if self.bench.ad.lane.confidence > 0.7 { Color32::GREEN } else { Color32::YELLOW },
                ),
            ],
            Tab::Autonomous => vec![
                (
                    format!("AD {}", if self.bench.ad.engaged { "engaged" } else { "standby" }),
                    if self.bench.ad.engaged { Color32::GREEN } else { Color32::from_gray(180) },
                ),
                (
                    format!("ACC set {:.0} km/h", self.bench.ad.acc_set_speed_kmh),
                    Color32::LIGHT_BLUE,
                ),
                (
                    format!("Lane {:.0}%", self.bench.ad.lane.confidence * 100.0),
                    if self.bench.ad.lane.confidence > 0.7 { Color32::GREEN } else { Color32::YELLOW },
                ),
            ],
            Tab::V2X => vec![
                (
                    format!("Link {:.0}%", self.v2x_range_ema.mul_add(0.0, (100.0 - (self.v2x_loss_ema * 2.0).clamp(0.0, 60.0) - ((self.v2x_lat_ema - 10.0) * 1.6).clamp(0.0, 40.0)).clamp(0.0, 100.0))),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Nearby {}", self.bench.v2x.nearby_vehicles.len()),
                    Color32::from_gray(180),
                ),
                (
                    format!("OTA {}", if self.bench.telematics.ota_downloading { "downloading" } else if self.bench.telematics.ota_available { "available" } else { "idle" }),
                    if self.bench.telematics.ota_downloading { Color32::YELLOW } else { Color32::GREEN },
                ),
            ],
            Tab::Uds => vec![
                (
                    format!("Session {}", self.bench.uds_ecm.session),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Security {:?}", self.bench.uds_ecm.security),
                    Color32::YELLOW,
                ),
                (
                    format!("UDS log {}", self.uds_log.len()),
                    Color32::from_gray(180),
                ),
            ],
            Tab::EcmLiveData => vec![
                (
                    format!("Mode {:?}", self.hw_cfg.mode),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Detected {}", self.ecm_detected_sas.len()),
                    Color32::from_gray(180),
                ),
                (
                    format!("Feed {}", if self.ecm_live_feed.is_some() { "active" } else { "idle" }),
                    if self.ecm_live_feed.is_some() { Color32::GREEN } else { Color32::YELLOW },
                ),
            ],
            Tab::LeakLab => vec![
                (
                    format!("Circuitos {}", self.bench.leak_rig.circuits.len()),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("Predicoes {}", self.leak_predictions.len()),
                    Color32::YELLOW,
                ),
                (
                    format!("Calibrado {}", if self.leak_calibration_report.is_some() { "sim" } else { "nao" }),
                    if self.leak_calibration_report.is_some() { Color32::GREEN } else { Color32::from_gray(180) },
                ),
            ],
            Tab::Plots => vec![
                (
                    format!("Samples {}", self.pl_rpm.len()),
                    Color32::from_rgb(110, 180, 255),
                ),
                (
                    format!("SPD {:.1} km/h", self.bench.tcm.ground_speed_kmh),
                    Color32::LIGHT_BLUE,
                ),
            ],
        }
    }

    fn render_ux_context_strip(&self, ui: &mut Ui) {
        let items = self.ux_context_items();
        if items.is_empty() {
            return;
        }
        ui.add_space(4.0);
        ui.horizontal_wrapped(|ui| {
            for (text, color) in items {
                egui::Frame::group(ui.style())
                    .fill(Color32::from_gray(20))
                    .inner_margin(Margin::symmetric(8.0, 4.0))
                    .show(ui, |ui| {
                        ui.label(RichText::new(text).size(10.0).color(color));
                    });
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

    fn render_ux_guide(&mut self, ui: &mut Ui) {
        let (title, guide, legend) = match self.tab {
            Tab::Help => (
                "Help",
                "Use esta aba para fluxo rapido de operacao, diagnostico e leitura da Leak Lab.",
                "Comece por Help -> Cluster -> Engine -> Leaks e depois avance para CAN/UDS.",
            ),
            Tab::Cluster => (
                "Cluster",
                "Use ignition -> throttle/brake -> direction to validate baseline vehicle response.",
                "Lamps: RED=critical stop, AMBER=warning, ABS/ESP=assist active.",
            ),
            Tab::CanBus => (
                "CAN Bus",
                "Start in Signals, then Trace, then Network for RCA. Freeze before exporting evidence.",
                "State colors: green=active, yellow=passive, red=bus-off/high error.",
            ),
            Tab::Events => (
                "Events",
                "Filter by severity and subsystem, then correlate timestamp with Faults/UDS actions.",
                "Icon color maps to severity; pause freezes feed for reading.",
            ),
            Tab::EcuNet => (
                "ECU Net",
                "Check node online state and boot stage before deep diagnostics.",
                "Degraded/offline nodes usually correlate with CAN or power faults.",
            ),
            Tab::Engine => (
                "Engine",
                "Validate thermal, pressure and torque limits under controlled load changes.",
                "Watch red/yellow status first, then inspect numeric channels.",
            ),
            Tab::Faults => (
                "Faults",
                "Inject one fault at a time; observe Events/CAN/Engine and clear to compare recovery.",
                "Use CLEAR after each test case to isolate cause/effect.",
            ),
            Tab::Boot => (
                "Boot",
                "Confirm startup sequence and timing before runtime validation.",
                "Blocked stage means readiness is not complete for higher-level tests.",
            ),
            Tab::Implements => (
                "Implements",
                "Operate PTO/hitch/aux valves incrementally and monitor hydraulic limits.",
                "Large jumps in commands can mask root cause of instability.",
            ),
            Tab::Params => (
                "Params",
                "Edit one family of parameters at a time; sync back before changing scenario.",
                "Use Sync from simulation to avoid stale values.",
            ),
            Tab::Sensors => (
                "Sensors",
                "Validate GNSS/IMU/radar/lidar/camera consistency before AD validation.",
                "Prioritize confidence/quality metrics before control metrics.",
            ),
            Tab::Autonomous => (
                "AD",
                "Engage only after sensors and base dynamics are stable; tune ACC/LKA gradually.",
                "TTC/THW and lane confidence are primary safety indicators.",
            ),
            Tab::V2X => (
                "V2X",
                "Evaluate latency/loss first, then cooperative alerts impact on control.",
                "Higher packet loss can invalidate AD cooperative assumptions.",
            ),
            Tab::Uds => (
                "UDS",
                "Follow sequence: session -> security -> read/write/routine -> transfer exit.",
                "Negative response 0x7F indicates rejected service/subfunction/conditions.",
            ),
            Tab::EcmLiveData => (
                "ECM Live",
                "Detect -> Connect -> Retrieve -> Export. Keep mode and SA aligned.",
                "In SIM mode, panels are mock; in LIVE mode, policy gates apply.",
            ),
            Tab::LeakLab => (
                "Leak Lab",
                "Select circuit, apply parameters, then run Scenario or Monte Carlo and review ranking.",
                "Use ASCII legend and temporal map to interpret pressure/risk/flow quickly.",
            ),
            Tab::Plots => (
                "Plots",
                "Use for before/after comparison around faults, AD engagement or UDS actions.",
                "Trend shape matters more than single-point values.",
            ),
        };

        egui::Frame::group(ui.style())
            .fill(Color32::from_gray(16))
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label(
                        RichText::new(format!("UX Guide - {}", title))
                            .size(11.6)
                            .color(Color32::from_rgb(110, 180, 255))
                            .strong(),
                    );
                    ui.separator();
                    ui.checkbox(&mut self.ux_show_guide, "Show details");
                    ui.checkbox(&mut self.ux_compact_mode, "Compact mode");
                    ui.separator();
                    self.render_ux_quick_actions(ui);
                });

                for (color, msg) in self.ux_alerts() {
                    ui.add_space(4.0);
                    ui.label(RichText::new(format!("• {}", msg)).size(10.3).color(color));
                }

                self.render_ux_context_strip(ui);

                if self.ux_show_guide {
                    ui.add_space(4.0);
                    ui.label(RichText::new(format!("How to use: {}", guide)).size(10.2));
                    ui.label(
                        RichText::new(format!("Legend/Interpretation: {}", legend))
                            .size(10.0)
                            .color(Color32::from_gray(175)),
                    );
                }
            });
    }

    fn render_ux_quick_actions(&mut self, ui: &mut Ui) {
        match self.tab {
            Tab::Help => {
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Cluster => {
                if ui.button("Key OFF").clicked() {
                    self.cmds.push(Cmd::KeyOff);
                }
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Events => {
                if ui.button("Clear events").clicked() {
                    self.events.clear();
                }
            }
            Tab::CanBus => {
                if ui.button("Clear CAN views").clicked() {
                    self.sig_map.clear();
                    self.trace_snap.clear();
                }
                if ui.button("Clear CAN injections").clicked() {
                    self.cmds.push(Cmd::CanClearAllInjections);
                }
            }
            Tab::EcuNet => {
                if ui.button("Key OFF").clicked() {
                    self.cmds.push(Cmd::KeyOff);
                }
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Engine => {
                if ui.button("Clear DTCs").clicked() {
                    self.cmds.push(Cmd::ClearDtcs);
                }
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Faults => {
                if ui.button("Inject selected").clicked() {
                    self.cmds.push(Cmd::InjectFault);
                }
                if ui.button("Clear faults").clicked() {
                    self.cmds.push(Cmd::ClearFaults);
                }
            }
            Tab::Boot => {
                if ui.button("Key advance").clicked() {
                    self.cmds.push(Cmd::KeyAdvance);
                }
                if ui.button("Key OFF").clicked() {
                    self.cmds.push(Cmd::KeyOff);
                }
            }
            Tab::Implements => {
                if ui.button("PTO OFF").clicked() {
                    self.cmds.push(Cmd::SetPtoMode(PtoMode::Off));
                }
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Params => {
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Sensors => {
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Autonomous => {
                if ui
                    .button(if self.bench.ad.engaged { "Disengage AD" } else { "Engage AD" })
                    .clicked()
                {
                    self.cmds.push(if self.bench.ad.engaged {
                        Cmd::DisengageAD
                    } else {
                        Cmd::EngageAD
                    });
                }
                if ui.button("Clear path").clicked() {
                    self.cmds.push(Cmd::ClearWaypoints);
                }
            }
            Tab::V2X => {
                if ui
                    .add_enabled(self.bench.telematics.ota_available, Button::new("Start OTA"))
                    .clicked()
                {
                    self.cmds.push(Cmd::StartOta);
                }
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
            Tab::Uds => {
                if ui.button("Clear UDS log").clicked() {
                    self.uds_log.clear();
                }
                if ui.button("Default Session").clicked() {
                    self.uds_input = "10 01".into();
                }
            }
            Tab::EcmLiveData => {
                if ui.button("Clear live snapshot").clicked() {
                    self.ecm_live_snapshot = io::ecm_params::EcmSnapshot::default();
                    self.ecm_live_frames_total = 0;
                    self.ecm_live_last_update_ms = 0;
                }
            }
            Tab::LeakLab => {
                if ui.button("Clear leak outputs").clicked() {
                    self.cmds.push(Cmd::LeakClearScenarioOutputs);
                }
                if ui.button("Reset leak sim").clicked() {
                    self.cmds.push(Cmd::LeakResetSimulation);
                }
            }
            Tab::Plots => {
                if ui.button("Reset simulation").clicked() {
                    self.cmds.push(Cmd::Reset);
                }
            }
        }
    }

    fn tab_ecm_live_data(&mut self, ui: &mut Ui) {
        ui.heading("ECM-Live Data");
        ui.add_space(6.0);
        let live_mode = matches!(self.hw_cfg.mode, io::hw::HwMode::Live);

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {

                if !live_mode {
                    ui.colored_label(
                        Color32::YELLOW,
                        "SIM mode active: showing mock ECM view. Switch to --hw-mode=live for hardware Detect/Connect/Retrieve.",
                    );
                    if self.ecm_detected_sas.is_empty() {
                        self.ecm_detected_sas = vec![0x00];
                    }
                }

        ui.horizontal(|ui| {
            if ui.add_enabled(live_mode, Button::new("Detect")).clicked() {
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

            let can_connect = live_mode && !self.ecm_detected_sas.is_empty();
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

            let can_retrieve = live_mode && self.ecm_connected && self.ecm_live_feed.is_none();
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
                    let name = format!("ecm_live_{}.csv", ts);
                    if let Some(path) = Self::pick_save_path(&name, "csv") {
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
            }
        });

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.label("Detected ECM SA:");
            if self.ecm_detected_sas.is_empty() {
                ui.label("(none)");
            } else {
                let mut selected = self.ecm_selected_idx.min(self.ecm_detected_sas.len() - 1);
                ComboBox::from_id_salt("ecm_live::sa_combo")
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
                Grid::new("ecm_live::dashboard_grid").show(ui, |ui| {
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
        Grid::new("ecm_live::snapshot_grid").show(ui, |ui| {
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

                self.ecm_treeview(ui, !live_mode);
            });
    }

    fn toolbar(&mut self, ui: &mut Ui) {
        ui.push_id("ui::toolbar", |ui| {
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
            let ad_locked = self.bench.ad.engaged;
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
            let mut thr = self.throttle_target;
            let thr_resp = ui.add_enabled(
                !ad_locked,
                Slider::new(&mut thr, 0.0..=1.0)
                    .show_value(false)
                    .trailing_fill(true),
            );
            if ad_locked {
                let _ = thr_resp.on_disabled_hover_text("Throttle manual bloqueado enquanto o AD estiver ativo.");
            } else {
                let thr_changed = (thr - self.throttle_cmd_last).abs() > 0.01;
                let thr_due = self.ticks.saturating_sub(self.throttle_cmd_last_tick) >= 2;
                if thr != self.throttle_target && (thr_changed || thr_due) {
                    self.cmds.push(Cmd::SetThrottle(thr));
                    self.throttle_cmd_last = thr;
                    self.throttle_cmd_last_tick = self.ticks;
                }
            }
            ui.label(
                RichText::new(format!("cmd {:3.0}% / act {:3.0}%", self.throttle_target * 100.0, self.throttle * 100.0))
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
            let mut brk = self.brake_target;
            let brk_resp = ui.add_enabled(
                !ad_locked,
                Slider::new(&mut brk, 0.0..=1.0)
                    .show_value(false)
                    .trailing_fill(true),
            );
            if ad_locked {
                let _ = brk_resp.on_disabled_hover_text("Freio manual bloqueado enquanto o AD estiver ativo.");
            } else {
                let brk_changed = (brk - self.brake_cmd_last).abs() > 0.01;
                let brk_due = self.ticks.saturating_sub(self.brake_cmd_last_tick) >= 2;
                if brk != self.brake_target && (brk_changed || brk_due) {
                    self.cmds.push(Cmd::SetBrake(brk));
                    self.brake_cmd_last = brk;
                    self.brake_cmd_last_tick = self.ticks;
                }
            }
            ui.label(
                RichText::new(format!("cmd {:3.0}% / act {:3.0}%", self.brake_target * 100.0, self.brake * 100.0))
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
            let dir_resp = ui.add_enabled_ui(!ad_locked, |ui| direction_selector(ui, dir_s));
            if ad_locked {
                let _ = dir_resp
                    .response
                    .on_disabled_hover_text("Direcao bloqueada enquanto o AD estiver ativo.");
            }
            if let Some(k) = dir_resp.inner {
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
            let auto_btn = Button::new(RichText::new(if auto { "AUTO✓" } else { "MANUAL" }).size(11.0))
                .fill(if auto {
                    Color32::from_rgb(20, 85, 35)
                } else {
                    Color32::from_gray(35)
                });
            let auto_resp = ui.add_enabled(!ad_locked, auto_btn);
            if ad_locked {
                let _ = auto_resp.on_disabled_hover_text("Modo da transmissao bloqueado enquanto o AD estiver ativo.");
            } else if auto_resp.clicked() {
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
        });
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB CLUSTER
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_help(&mut self, ui: &mut Ui) {
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.heading("Help - Operacao Rapida e Guia de Diagnostico");
                ui.add_space(6.0);

                ui.group(|ui| {
                    ui.label(
                        RichText::new("Fluxo recomendado (conciso)")
                            .size(12.0)
                            .color(Color32::from_rgb(95, 170, 255))
                            .strong(),
                    );
                    ui.label("1) BOOT: confirmar ignicao e readiness.");
                    ui.label("2) CLUSTER/ENGINE: validar rpm, pressao, temperatura e marcha.");
                    ui.label("3) LEAK LAB: aplicar parametros, rodar cenarios/MC e interpretar risco.");
                    ui.label("4) CAN/EVENTS/UDS: fazer RCA e exportar evidencia.");
                    ui.label("5) PLOTS: comparar antes/depois das mudancas.");
                });

                ui.add_space(8.0);
                ui.group(|ui| {
                    ui.label(
                        RichText::new("Leak Lab - O que significa cada bloco")
                            .size(12.0)
                            .color(Color32::from_rgb(95, 170, 255))
                            .strong(),
                    );
                    ui.label("Runtime Circuits: estado atual de cada circuito com alertas e leak L/min.");
                    ui.label("Manual Engineering Input: parametros de pressao e vedacao para calibracao dirigida.");
                    ui.label("Custom Circuit Builder: cria circuito novo para bancada/engenharia.");
                    ui.label("Scenario Ranking: ordena cenarios por risco final e pressao de pico.");
                    ui.label("Timeback/ASCII/Plot: historico temporal de pressao, risco e vazao.");
                });

                ui.add_space(8.0);
                ui.group(|ui| {
                    ui.label(
                        RichText::new("Leak Lab - Envelope correto")
                            .size(12.0)
                            .color(Color32::from_rgb(95, 170, 255))
                            .strong(),
                    );
                    ui.label("Regra obrigatoria: min <= mean <= ideal <= max < rupture");
                    ui.label("Squeeze: tipico 10%~30%, Compression Set: menor tende a maior vida util.");
                    ui.label("Base leak area: influencia vazao de fuga inicial (mm²)." );
                    ui.label("Rupture bar define limite de falha catastrofica e o risco de burst.");
                });

                ui.add_space(8.0);
                ui.group(|ui| {
                    ui.label(
                        RichText::new("Leak Lab - Como operar sem erro")
                            .size(12.0)
                            .color(Color32::from_rgb(95, 170, 255))
                            .strong(),
                    );
                    ui.label("1) Selecione circuito.");
                    ui.label("2) Ajuste parametros e valide (a UI bloqueia aplicacao invalida).");
                    ui.label("3) APPLY MANUAL PARAMETERS.");
                    ui.label("4) RUN SCENARIO PREDICTION ou RUN MONTE CARLO.");
                    ui.label("5) Leia ranking + timeline + plot + alertas.");
                    ui.label("6) Exporte CSV/JSON para rastreabilidade.");
                });

                ui.add_space(8.0);
                ui.group(|ui| {
                    ui.label(
                        RichText::new("Atalhos de diagnostico")
                            .size(12.0)
                            .color(Color32::from_rgb(95, 170, 255))
                            .strong(),
                    );
                    ui.label("- EVENTS: filtre por severidade e subsystem.");
                    ui.label("- CAN: use Freeze antes de exportar snapshot.");
                    ui.label("- UDS: siga Session -> Security -> Service -> Transfer Exit.");
                    ui.label("- ECM LIVE: Detect -> Connect -> Retrieve -> Export.");
                });
            });
    }

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
                self.trace_snap.clear();
            }
            ui.separator();
            ui.label(RichText::new("Filter:").size(11.0).color(Color32::GRAY));
            ui.add_sized(
                [220.0, 20.0],
                TextEdit::singleline(&mut self.can_filter)
                    .hint_text("text | sa:00 | pgn:61444 | id:18F00400"),
            );
            ui.checkbox(&mut self.can_hide_stale, "Hide stale");
            ui.separator();
            ui.label(RichText::new("Rows").size(10.8).color(Color32::from_gray(145)));
            ui.add(Slider::new(&mut self.can_signal_limit, 50..=1200).text("sig"));
            ui.add(Slider::new(&mut self.can_trace_limit, 100..=2000).text("trace"));
            if matches!(self.can_mode, CanMode::Trace) {
                ui.separator();
                ui.label(RichText::new("Trace tick").size(10.8).color(Color32::from_gray(145)));
                ui.add(
                    Slider::new(&mut self.can_trace_update_every_ticks, 1..=30)
                        .text("every")
                        .suffix("t"),
                );
                ui.checkbox(&mut self.can_trace_pause_on_scroll, "Pause on scroll");
                if self.can_trace_pause_ticks_left > 0 {
                    ui.label(
                        RichText::new(format!("paused {}t", self.can_trace_pause_ticks_left))
                            .size(10.8)
                            .color(Color32::YELLOW),
                    );
                }
            }
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
        ui.label(
            RichText::new("Tip: use sa:XX / pgn:NNNN / id:XXXXXXXX for deterministic filtering")
                .size(9.8)
                .color(Color32::from_gray(125)),
        );
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
            ComboBox::from_id_salt("can::target_bus")
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
                let ts = Local::now().format("%Y%m%d_%H%M%S");
                let name = format!("can_network_snapshot_{}.csv", ts);
                if let Some(path) = Self::pick_save_path(&name, "csv") {
                    self.cmds.push(Cmd::CanExportSnapshotCsv(path));
                }
            }
            if ui.button("Export JSON").clicked() {
                let ts = Local::now().format("%Y%m%d_%H%M%S");
                let name = format!("can_network_snapshot_{}.json", ts);
                if let Some(path) = Self::pick_save_path(&name, "json") {
                    self.cmds.push(Cmd::CanExportSnapshotJson(path));
                }
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

        egui::Grid::new("can::net_buses")
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
            egui::Frame::group(cols[0].style())
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

            egui::Frame::group(cols[1].style())
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
        let filt = self.can_filter.clone();
        // Table headers
        egui::Grid::new("can::sig_hdr")
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
            .filter(|((pgn, sa), s)| Self::can_filter_match_signal(&filt, *pgn, *sa, s))
            .filter(|(_, s)| !self.can_hide_stale || s.fresh)
            .collect();
        sigs.sort_by_key(|((pgn, sa), _)| (*sa, *pgn));

        let total = sigs.len();
        ui.label(
            RichText::new(format!(
                "Showing {} / {} signals",
                total.min(self.can_signal_limit),
                total
            ))
            .size(10.2)
            .color(Color32::from_gray(130)),
        );

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .max_height(ui.available_height())
            .show(ui, |ui| {
                for ((pgn, sa), sig) in sigs.iter().take(self.can_signal_limit) {
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
                    ui.horizontal(|ui| {
                        ui.add_sized(
                            [56.0, 18.0],
                            Label::new(
                                RichText::new(format!("{:>5.2}", age))
                                    .size(10.2)
                                    .monospace()
                                    .color(ac),
                            ),
                        );
                        ui.add_sized(
                            [190.0, 18.0],
                            Label::new(
                                RichText::new(format!("{:>5} {}", pgn, sig.pgn_name))
                                    .size(10.2)
                                    .monospace()
                                    .color(Color32::GOLD),
                            ),
                        );
                        ui.add_sized(
                            [125.0, 18.0],
                            Label::new(
                                RichText::new(format!("0x{:02X} {}", sa, sig.sa_name))
                                    .size(10.2)
                                    .monospace()
                                    .color(Color32::from_rgb(120, 200, 255)),
                            ),
                        );
                        ui.add_sized(
                            [76.0, 18.0],
                            Label::new(
                                RichText::new(format!("{:>5.0}ms", sig.period_ms))
                                    .size(10.2)
                                    .monospace()
                                    .color(Color32::from_gray(150)),
                            ),
                        );
                        ui.add_sized(
                            [74.0, 18.0],
                            Label::new(
                                RichText::new(format!("{:>7}", sig.count))
                                    .size(10.2)
                                    .monospace()
                                    .color(Color32::from_gray(130)),
                            ),
                        );
                        ui.label(RichText::new(&vals).size(10.2).color(ac));
                    });
                }
            });
    }

    fn can_trace(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label(RichText::new("Trace View:").size(10.8).color(Color32::GRAY));
            ui.selectable_value(&mut self.can_trace_tree, false, "Table");
            ui.selectable_value(&mut self.can_trace_tree, true, "Tree");
        });
        ui.separator();

        let filtered_rows: Vec<&TraceRow> = self
            .trace_snap
            .iter()
            .filter(|r| Self::can_filter_match_trace(&self.can_filter, r))
            .collect();
        ui.label(
            RichText::new(format!(
                "Rows after filter: {} (render limit {})",
                filtered_rows.len(),
                self.can_trace_limit
            ))
            .size(10.2)
            .color(Color32::from_gray(130)),
        );

        if self.can_trace_tree {
            let mut by_sa: BTreeMap<u8, Vec<TraceRow>> = BTreeMap::new();
            for row in filtered_rows.iter().rev().take(self.can_trace_limit) {
                by_sa.entry(row.2).or_default().push((*row).clone());
            }

            ScrollArea::vertical()
                .auto_shrink([false, false])
                .max_height(ui.available_height())
                .show(ui, |ui| {
                    for (sa, rows) in by_sa {
                        CollapsingHeader::new(format!("Node SA 0x{:02X} ({} frames)", sa, rows.len()))
                            .default_open(sa == 0x00 || sa == 0x03)
                            .show(ui, |ui| {
                                let mut by_pgn: BTreeMap<String, Vec<TraceRow>> = BTreeMap::new();
                                for r in rows {
                                    by_pgn.entry(r.5.clone()).or_default().push(r);
                                }
                                for (pgn, mut frames) in by_pgn {
                                    frames.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
                                    CollapsingHeader::new(format!("{} ({})", pgn, frames.len()))
                                        .default_open(false)
                                        .show(ui, |ui| {
                                            for (ts, raw_id, _sa, dlc, hex, _pgn_sa, decoded) in
                                                frames.iter().rev().take(12).rev()
                                            {
                                                ui.horizontal_wrapped(|ui| {
                                                    ui.label(
                                                        RichText::new(format!("t={:8.3}s", ts))
                                                            .monospace()
                                                            .size(10.0)
                                                            .color(Color32::from_gray(130)),
                                                    );
                                                    ui.label(
                                                        RichText::new(format!("ID={:08X}", raw_id))
                                                            .monospace()
                                                            .size(10.0)
                                                            .color(Color32::LIGHT_BLUE),
                                                    );
                                                    ui.label(
                                                        RichText::new(format!("DLC={}", dlc))
                                                            .monospace()
                                                            .size(10.0)
                                                            .color(Color32::from_gray(140)),
                                                    );
                                                    ui.label(
                                                        RichText::new(hex)
                                                            .monospace()
                                                            .size(10.0)
                                                            .color(Color32::from_gray(200)),
                                                    );
                                                    if !decoded.is_empty() {
                                                        ui.label(
                                                            RichText::new(decoded)
                                                                .size(10.0)
                                                                .color(Color32::from_gray(190)),
                                                        );
                                                    }
                                                });
                                            }
                                        });
                                }
                            });
                    }
                });
            return;
        }

        egui::Grid::new("can::trace_hdr")
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
        // stick_to_bottom only when live; when frozen the user can scroll freely
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .max_height(ui.available_height())
            .stick_to_bottom(!self.can_freeze)
            .show(ui, |ui| {
                for row in filtered_rows.iter().rev().take(self.can_trace_limit).rev() {
                    let (ts, raw_id, sa, dlc, hex, pgn_sa, decoded) = *row;
                    let sc = match *sa {
                        0x00 => Color32::from_rgb(80, 220, 80),
                        0x03 => Color32::from_rgb(0, 210, 210),
                        0x27 => Color32::LIGHT_BLUE,
                        0x1E => Color32::YELLOW,
                        0x0B => Color32::WHITE,
                        _ => Color32::from_gray(175),
                    };
                    ui.horizontal(|ui| {
                        ui.add_sized(
                            [68.0, 18.0],
                            Label::new(
                                RichText::new(format!("{:8.3}", ts))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(110)),
                            ),
                        );
                        ui.add_sized(
                            [84.0, 18.0],
                            Label::new(
                                RichText::new(format!("{:08X}", raw_id))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::LIGHT_BLUE),
                            ),
                        );
                        ui.add_sized(
                            [32.0, 18.0],
                            Label::new(
                                RichText::new(format!("{}", dlc))
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(130)),
                            ),
                        );
                        ui.add_sized(
                            [205.0, 18.0],
                            Label::new(
                                RichText::new(hex)
                                    .size(10.0)
                                    .monospace()
                                    .color(Color32::from_gray(195)),
                            ),
                        );
                        ui.add_sized([180.0, 18.0], Label::new(RichText::new(pgn_sa).size(10.0).color(sc)));
                        ui.label(RichText::new(decoded).size(10.0).color(Color32::from_gray(200)));
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
        egui::Grid::new("events::hdr")
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
                for (idx, ev) in self
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
                    .enumerate()
                {
                    let (icon, col) = match ev.lvl {
                        EventLevel::Debug => ("·", Color32::from_gray(100)),
                        EventLevel::Info => ("ℹ", Color32::LIGHT_BLUE),
                        EventLevel::Ok => ("✓", Color32::GREEN),
                        EventLevel::Warn => ("⚠", Color32::YELLOW),
                        EventLevel::Critical => ("✗", Color32::RED),
                    };
                    ui.push_id(format!("events::row_{}", idx), |ui| {
                        ui.horizontal(|ui| {
                            ui.add_sized(
                                [56.0, 18.0],
                                Label::new(
                                    RichText::new(format!("{:7.2}", ev.ts))
                                        .size(10.5)
                                        .monospace()
                                        .color(Color32::from_gray(115)),
                                ),
                            );
                            ui.add_sized(
                                [20.0, 18.0],
                                Label::new(RichText::new(icon.to_string()).size(11.0).color(col)),
                            );
                            ui.add_sized(
                                [78.0, 18.0],
                                Label::new(
                                    RichText::new(format!("[{:<5}]", ev.source))
                                        .size(10.5)
                                        .monospace()
                                        .color(Color32::from_gray(165)),
                                ),
                            );
                            ui.label(RichText::new(&ev.msg).size(10.5).color(col));
                        });
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
        let online = self
            .bench
            .boot
            .ecus
            .iter()
            .filter(|e| e.is_online())
            .count();
        let total = self.bench.boot.ecus.len();
        let unhealthy_nodes = gw
            .nodes
            .iter()
            .filter(|n| n.tec > 0 || n.rec > 0 || !matches!(n.state, BusState::ErrorActive))
            .count();
        let net_health = if matches!(gw.bus_state, BusState::BusOff) {
            "CRITICAL"
        } else if unhealthy_nodes > 0 || gw.total_errors > 0 {
            "DEGRADED"
        } else {
            "HEALTHY"
        };
        let net_col = match net_health {
            "CRITICAL" => Color32::RED,
            "DEGRADED" => Color32::YELLOW,
            _ => Color32::GREEN,
        };
        ui.label(
            RichText::new(format!(
                "J1939 HS-CAN 500kbps | Bus:{} | Nodes:{}/{} | TotalFrames:{}",
                gw.bus_state,
                online,
                total,
                gw.total_tx
            ))
            .size(12.0)
            .color(Color32::from_gray(185)),
        );
        ui.label(
            RichText::new(format!(
                "Network health: {} | Unhealthy nodes: {} | Total errors: {}",
                net_health, unhealthy_nodes, gw.total_errors
            ))
            .size(11.0)
            .color(net_col),
        );
        ui.separator();

        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                egui::Grid::new("ecu_net::grid")
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
                    RichText::new("CRITICAL NODES (TEC/REC or non-EA state):")
                        .size(11.0)
                        .color(Color32::from_gray(140)),
                );
                for n in gw
                    .nodes
                    .iter()
                    .filter(|n| n.tec > 0 || n.rec > 0 || !matches!(n.state, BusState::ErrorActive))
                    .take(8)
                {
                    ui.label(
                        RichText::new(format!(
                            "SA 0x{:02X}  TEC:{} REC:{}  State:{}",
                            n.source_addr, n.tec, n.rec, n.state
                        ))
                        .size(10.5)
                        .monospace()
                        .color(Color32::YELLOW),
                    );
                }
                ui.separator();
                ui.label(
                    RichText::new("BUS ERROR LOG:")
                        .size(11.0)
                        .color(Color32::from_gray(140)),
                );
                for err in gw.error_log.iter().rev().take(40) {
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
                    egui::Frame::group(cols[0].style())
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
                    egui::Frame::group(cols[1].style())
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
                    egui::Frame::group(cols[2].style())
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
    fn dtc_fault_storage_addr(spn: u32, fmi: u8) -> u32 {
        // Virtualized NVM map for fault records in ECM region.
        0x0802_0000u32 + (spn & 0xFFFF) * 0x20 + ((fmi as u32) * 0x2)
    }

    fn dtc_dm1_payload_hex(amber: bool, red: bool, mil: bool, spn: u32, fmi: u8) -> String {
        let dm1 = auto_breaking::j1939::Builder::dm1(
            0.0,
            amber,
            red,
            mil,
            spn,
            fmi,
            auto_breaking::j1939::addr::ECM_1,
        );
        dm1.data
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ")
    }

    fn fault_selection_hint(selected: auto_breaking::FaultType) -> Option<(u32, u8, &'static str)> {
        match selected {
            auto_breaking::FaultType::HighCoolantTemp => {
                Some((110, 0, "Coolant temp above severe threshold"))
            }
            auto_breaking::FaultType::LowOilPressure => {
                Some((100, 1, "Oil pressure below severe threshold"))
            }
            auto_breaking::FaultType::LowFuelPressure => {
                Some((94, 1, "Fuel delivery pressure below threshold"))
            }
            auto_breaking::FaultType::CriticalDefLevel => {
                Some((3361, 1, "DEF critically low (derate trigger)"))
            }
            auto_breaking::FaultType::HighDpfSoot => {
                Some((3251, 16, "DPF soot above high threshold"))
            }
            _ => None,
        }
    }

    fn tab_faults(&mut self, ui: &mut Ui) {
        let dtc_now = self.bench.ecm.active_dtcs.len();
        let dtc_delta = dtc_now as isize - self.fault_last_dtc_count as isize;
        if dtc_delta != 0 {
            self.fault_feedback = format!(
                "ECM DTC update observed: {} -> {} ({:+})",
                self.fault_last_dtc_count, dtc_now, dtc_delta
            );
            self.fault_last_dtc_count = dtc_now;
        }
        ScrollArea::vertical()
            .id_salt("faults::page_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
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
                            .id_salt("faults::active_dtcs_scroll")
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
                                            let mem_addr = Self::dtc_fault_storage_addr(dtc.spn, dtc.fmi);
                                            let dm1_hex = Self::dtc_dm1_payload_hex(
                                                matches!(dtc.severity, DtcSeverity::Amber),
                                                matches!(dtc.severity, DtcSeverity::Red),
                                                matches!(dtc.severity, DtcSeverity::Mil),
                                                dtc.spn,
                                                dtc.fmi,
                                            );
                                            ui.label(
                                                RichText::new(format!(
                                                    "ECM fault storage addr: 0x{mem_addr:08X} (NVM.FaultStorage)"
                                                ))
                                                .size(9.7)
                                                .monospace()
                                                .color(Color32::from_rgb(140, 210, 255)),
                                            );
                                            ui.label(
                                                RichText::new(
                                                    "Diagnostic request -> ECM SA 0x00: UDS 19 02 FF (ReadDTC by status mask)"
                                                        .to_string(),
                                                )
                                                .size(9.6)
                                                .monospace()
                                                .color(Color32::from_gray(170)),
                                            );
                                            ui.label(
                                                RichText::new(format!(
                                                    "ECM recognition broadcast -> J1939 DM1 PGN 65226 DATA [{}]",
                                                    dm1_hex
                                                ))
                                                .size(9.6)
                                                .monospace()
                                                .color(Color32::from_gray(170)),
                                            );
                                        });
                                }
                            });
                    }
                    ui.separator();
                    ui.label(
                        RichText::new(format!(
                            "Fault pipeline: selected -> inject -> ECM evaluate -> DM1 broadcast | DTC now: {} ({:+})",
                            dtc_now, dtc_delta
                        ))
                        .size(10.0)
                        .color(Color32::from_gray(150)),
                    );
                    if !self.fault_feedback.is_empty() {
                        ui.label(
                            RichText::new(format!("Last action: {}", self.fault_feedback))
                                .size(10.2)
                                .color(Color32::LIGHT_BLUE),
                        );
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
                        .id_salt("faults::selector_scroll")
                        .max_height(340.0)
                        .auto_shrink([false, false])
                        .show(ui, |ui| {
                            for (i, ft) in FAULT_TYPES.iter().enumerate() {
                                ui.push_id(format!("fault::selector::{}", i), |ui| {
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
                                });
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
                    ui.separator();
                    ui.label(
                        RichText::new("ECM DTC RECOGNITION GUIDE")
                            .size(11.0)
                            .color(Color32::from_rgb(110, 180, 255))
                            .strong(),
                    );
                    if let Some((spn, fmi, msg)) =
                        Self::fault_selection_hint(self.bench.selected_fault)
                    {
                        let addr = Self::dtc_fault_storage_addr(spn, fmi);
                        let dm1_hex = Self::dtc_dm1_payload_hex(true, false, false, spn, fmi);
                        ui.label(
                            RichText::new(format!(
                                "Selected fault maps to SPN {spn} FMI {fmi} -> addr 0x{addr:08X}"
                            ))
                            .size(10.2)
                            .monospace()
                            .color(Color32::from_rgb(140, 210, 255)),
                        );
                        ui.label(
                            RichText::new(format!("Trigger condition: {msg}"))
                                .size(10.0)
                                .color(Color32::from_gray(180)),
                        );
                        ui.label(
                            RichText::new("Message to query/ack in diagnostics: UDS 19 02 FF -> ECM SA 0x00")
                                .size(9.8)
                                .monospace()
                                .color(Color32::from_gray(170)),
                        );
                        ui.label(
                            RichText::new(format!(
                                "Expected ECM DM1 payload for this DTC: [{}]",
                                dm1_hex
                            ))
                            .size(9.8)
                            .monospace()
                            .color(Color32::from_gray(170)),
                        );
                    } else {
                        ui.label(
                            RichText::new(
                                "This selected fault is network/power domain and may not map to an ECM SPN/FMI DTC record.",
                            )
                            .size(9.8)
                            .color(Color32::from_gray(155)),
                        );
                    }
                });
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
            let per = ils.len().div_ceil(3);
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
                            RichText::new(format!("{}: {}", il.id, il.description))
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
                        ui.push_id(format!("impl::aux::{}", i), |ui| {
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
                        if imp.isobus_connected {
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
                        self.implement_feedback = if fen {
                            "Front PTO -> OFF".into()
                        } else {
                            "Front PTO -> ON".into()
                        };
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
                            self.implement_feedback =
                                format!("Hitch control mode -> {}", mode_s);
                        }
                    }
                    ui.separator();
                    let hitch_err = (self.hitch_target - hitch_pos).abs();
                    let hitch_col = if hitch_err > 12.0 {
                        Color32::YELLOW
                    } else {
                        Color32::GREEN
                    };
                    ui.label(
                        RichText::new(format!(
                            "Control tracking: Hitch target {:.1}% / actual {:.1}% (err {:.1}%)",
                            self.hitch_target, hitch_pos, hitch_err
                        ))
                        .size(10.2)
                        .color(hitch_col),
                    );
                    if !self.implement_feedback.is_empty() {
                        ui.label(
                            RichText::new(format!("Last command: {}", self.implement_feedback))
                                .size(10.2)
                                .color(Color32::LIGHT_BLUE),
                        );
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
                    egui::Frame::group(cols[0].style())
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
                    egui::Frame::group(cols[1].style())
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
                    egui::Frame::group(cols[2].style())
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
    fn uds_process_sa(&mut self, sa: u8, req: &[u8], ts: f64) -> Vec<u8> {
        if sa == 0x00 {
            self.bench.uds_ecm.process(req, ts)
        } else if sa == 0x03 {
            self.bench.uds_tcm.process(req, ts)
        } else {
            vec![0x7F, req.first().copied().unwrap_or(0x00), 0x11]
        }
    }

    fn uds_send_and_log(&mut self, sa: u8, req: Vec<u8>, ts: f64) -> Vec<u8> {
        let req_hex = req
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ");
        let svc = auto_breaking::uds::UdsServer::service_name(req[0]);
        let resp = self.uds_process_sa(sa, &req, ts);
        let resp_hex = resp
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ");
        let nrc = resp.first().copied() == Some(0x7F);
        self.uds_log.push((
            true,
            format!("[{:.2}s] 0x{:02X} {} -> {}", ts, sa, svc, req_hex),
        ));
        self.uds_log.push((
            false,
            format!(
                "            RESP {}{}",
                resp_hex,
                if nrc { " FAIL" } else { " OK" }
            ),
        ));
        if self.uds_log.len() > 240 {
            self.uds_log.drain(0..60);
        }
        resp
    }

    fn run_uds_flash_pipeline(&mut self, sa: u8, ts: f64) -> Result<String, String> {
        if self.uds_flash_path.trim().is_empty() {
            return Err("selecione um arquivo .bin para flash".into());
        }
        let fw = std::fs::read(&self.uds_flash_path)
            .map_err(|e| format!("falha ao ler firmware: {}", e))?;
        if fw.is_empty() {
            return Err("arquivo de firmware vazio".into());
        }

        // Session + security unlock for programming level.
        let _ = self.uds_send_and_log(sa, vec![0x10, 0x02], ts);
        let seed_resp = self.uds_send_and_log(sa, vec![0x27, 0x05], ts);
        if seed_resp.len() < 6 || seed_resp[0] != 0x67 {
            return Err("seed de seguranca nao recebido".into());
        }
        let seed = ((seed_resp[2] as u32) << 24)
            | ((seed_resp[3] as u32) << 16)
            | ((seed_resp[4] as u32) << 8)
            | seed_resp[5] as u32;
        let key = seed ^ 0xDEADBEEF;
        let unlock_resp = self.uds_send_and_log(
            sa,
            vec![
                0x27,
                0x06,
                ((key >> 24) & 0xFF) as u8,
                ((key >> 16) & 0xFF) as u8,
                ((key >> 8) & 0xFF) as u8,
                (key & 0xFF) as u8,
            ],
            ts,
        );
        if unlock_resp.first().copied() != Some(0x67) {
            return Err("unlock de seguranca falhou".into());
        }

        let _ = self.uds_send_and_log(sa, vec![0x10, 0x03], ts);

        let total_len = fw.len() as u32;
        let req_dl = vec![
            0x34,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            ((total_len >> 24) & 0xFF) as u8,
            ((total_len >> 16) & 0xFF) as u8,
            ((total_len >> 8) & 0xFF) as u8,
            (total_len & 0xFF) as u8,
        ];
        let dl_resp = self.uds_send_and_log(sa, req_dl, ts);
        if dl_resp.first().copied() != Some(0x74) {
            return Err("request download rejeitado".into());
        }

        let mut block: u8 = 1;
        for chunk in fw.chunks(120) {
            let mut req = Vec::with_capacity(chunk.len() + 2);
            req.push(0x36);
            req.push(block);
            req.extend_from_slice(chunk);
            let tr = self.uds_send_and_log(sa, req, ts);
            if tr.first().copied() != Some(0x76) {
                return Err(format!("transfer data falhou no bloco {}", block));
            }
            block = block.wrapping_add(1);
        }

        let end = self.uds_send_and_log(sa, vec![0x37, 0x00], ts);
        if end.first().copied() != Some(0x77) {
            return Err("request transfer exit falhou".into());
        }

        Ok(format!(
            "flash concluido (simulado): {} bytes enviados para SA 0x{:02X}",
            fw.len(),
            sa
        ))
    }

    fn leak_ascii_cad(
        circuit: &LeakCircuit,
        report: &auto_breaking::CircuitResult,
    ) -> (String, Vec<String>) {
        const SECTORS: usize = 16;
        let mut local_pressure = [0.0_f64; SECTORS];
        let mut local_frag = [0.0_f64; SECTORS];
        let mut local_flow = [0.0_f64; SECTORS];

        for i in 0..SECTORS {
            let theta = i as f64 * std::f64::consts::TAU / SECTORS as f64;
            let harmonic = (2.0 * theta).cos() * 0.15 + (theta + 0.4).sin() * 0.07;
            let pressure = (report.current_pressure_bar * (1.0 + harmonic)).max(0.0);
            let pressure_ratio = pressure / circuit.pressure.rupture_bar.max(1e-6);
            let frag = (0.62 * pressure_ratio + 0.38 * (report.rupture_probability_pct / 100.0))
                .clamp(0.0, 1.0);
            let flow = report.leak_lpm * (0.65 + 0.35 * harmonic.max(-0.9));

            local_pressure[i] = pressure;
            local_frag[i] = frag;
            local_flow[i] = flow.max(0.0);
        }

        let mut max_pressure_idx = 0usize;
        let mut max_frag_idx = 0usize;
        for i in 1..SECTORS {
            if local_pressure[i] > local_pressure[max_pressure_idx] {
                max_pressure_idx = i;
            }
            if local_frag[i] > local_frag[max_frag_idx] {
                max_frag_idx = i;
            }
        }

        let width = 37usize;
        let height = 17usize;
        let mut grid = vec![vec![' '; width]; height];
        for (y, row) in grid.iter_mut().enumerate().take(height) {
            for (x, cell) in row.iter_mut().enumerate().take(width) {
                let nx = (x as f64 / (width - 1) as f64 - 0.5) * 2.0;
                let ny = (y as f64 / (height - 1) as f64 - 0.5) * 2.0;
                let rr = ((nx * 1.1).powi(2) + (ny * 0.9).powi(2)).sqrt();
                if (0.45..=0.95).contains(&rr) {
                    let ang = ny.atan2(nx);
                    let mut idx = ((ang + std::f64::consts::PI) / std::f64::consts::TAU
                        * SECTORS as f64) as usize;
                    if idx >= SECTORS {
                        idx = SECTORS - 1;
                    }
                    let frag = local_frag[idx];
                    let mut ch = if frag > 0.85 {
                        'X'
                    } else if frag > 0.70 {
                        '!'
                    } else if frag > 0.50 {
                        '*'
                    } else if frag > 0.30 {
                        ':'
                    } else {
                        '.'
                    };
                    if idx == max_pressure_idx {
                        ch = 'P';
                    }
                    if idx == max_frag_idx {
                        ch = 'T';
                    }
                    *cell = ch;
                }
            }
        }

        let mut cad = String::new();
        cad.push_str("      ENGINEERING ASCII CAD - O-RING / SEAL CROSS-SECTION\n");
        cad.push_str("  legend: T=tear-up point, P=max pressure point, X=fragility critical\n");
        cad.push_str("          !=high fragility, *=moderate, :=watch, .=low\n\n");
        for row in &grid {
            let line: String = row.iter().collect();
            cad.push_str(&line);
            cad.push('\n');
        }

        let mut lines = Vec::new();
        lines.push(format!(
            "pressure point (max): sector {} | {:.2} bar",
            max_pressure_idx, local_pressure[max_pressure_idx]
        ));
        lines.push(format!(
            "tear-up point (max fragility): sector {} | fragility {:.3}",
            max_frag_idx, local_frag[max_frag_idx]
        ));
        lines.push(format!(
            "global leak flow: {:.4} L/min | rupture risk {:.1}% | band {}",
            report.leak_lpm, report.rupture_probability_pct, report.pressure_band
        ));

        let mut ranked: Vec<(usize, f64)> = (0..SECTORS).map(|i| (i, local_frag[i])).collect();
        ranked.sort_by(|a, b| b.1.total_cmp(&a.1));
        for (idx, frag) in ranked.into_iter().take(6) {
            lines.push(format!(
                "fragility point s{:02}: pressure {:.2} bar | fragility {:.3} | flow {:.4} L/min",
                idx, local_pressure[idx], frag, local_flow[idx]
            ));
        }

        (cad, lines)
    }

    fn tab_leak_lab(&mut self, ui: &mut Ui) {
        self.sanitize_leak_manual();
        self.sanitize_leak_custom();
        let manual_validation = self.validate_leak_manual();
        let custom_validation = self.validate_leak_custom();
        let manual_ok = manual_validation.is_ok();
        let custom_ok = custom_validation.is_ok();

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
        let calibration_report = self.leak_calibration_report.clone();
        let selected_circuit = self.bench.leak_rig.circuits.get(self.leak_sel_idx).cloned();
        let selected_report = selected_circuit.as_ref().and_then(|c| {
            reports
                .iter()
                .find(|r| r.name == c.name)
                .cloned()
        });

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
        ui.group(|ui| {
            ui.label(
                RichText::new("Como ler esta aba (direto ao ponto)")
                    .size(10.8)
                    .color(Color32::from_rgb(110, 180, 255))
                    .strong(),
            );
            ui.label("1) Runtime Circuits mostra estado atual (pressao, leak, risco, alerta).");
            ui.label("2) Manual Input ajusta engenharia (envelope/oring/oil). A aplicacao invalida fica bloqueada.");
            ui.label("3) Scenario Ranking ordena pior caso por risco e pico de pressao.");
            ui.label("4) Timeback ASCII/Plot mostra evolucao temporal de pressao-risco-vazao.");
            ui.label("5) Use export CSV/JSON para auditoria e rastreabilidade.");
        });
        ui.horizontal_wrapped(|ui| {
            if ui
                .add(Button::new("Nova rodada (limpar saidas)").fill(Color32::from_rgb(40, 50, 70)))
                .clicked()
            {
                self.cmds.push(Cmd::LeakClearScenarioOutputs);
            }
            if ui
                .add(Button::new("RESET Leak Sim (estado fisico)").fill(Color32::from_rgb(85, 35, 25)))
                .clicked()
            {
                self.cmds.push(Cmd::LeakResetSimulation);
            }
            if ui
                .add_enabled(
                    manual_ok,
                    Button::new("Aplicar + Rodar Cenario").fill(Color32::from_rgb(20, 70, 100)),
                )
                .clicked()
            {
                self.cmds.push(Cmd::LeakApplyAndPredict);
            }
            if ui
                .add_enabled(
                    manual_ok,
                    Button::new("Aplicar + Rodar Monte Carlo").fill(Color32::from_rgb(70, 35, 90)),
                )
                .clicked()
            {
                self.cmds.push(Cmd::LeakApplyAndMonteCarlo);
            }
        });
        ui.separator();
        ui.group(|ui| {
            ui.horizontal_wrapped(|ui| {
                ui.label(
                    RichText::new("TIMEBACK VIEW")
                        .size(10.8)
                        .color(Color32::from_rgb(110, 180, 255))
                        .strong(),
                );
                ui.label("janela(s)");
                ui.add(DragValue::new(&mut self.leak_timeback_window_s).speed(5.0).range(10.0..=7200.0));
                ui.label("stride");
                ui.add(DragValue::new(&mut self.leak_timeback_stride).speed(1.0).range(1..=60));
                ui.checkbox(&mut self.leak_timeback_latest_first, "latest first");
                ui.label(
                    RichText::new(format!("hist: {} pontos", self.leak_temporal_trace.len()))
                        .size(9.8)
                        .color(Color32::from_gray(160)),
                );
            });
            ui.label(
                RichText::new("ASCII timeline ponto a ponto: [pressao][risco][vazao] com janela temporal configuravel")
                    .size(9.6)
                    .color(Color32::from_gray(155)),
            );
        });
        ui.separator();

        let mut do_apply = false;
        let mut do_predict = false;
        let mut do_add_custom = false;
        let mut do_export_report_csv = false;
        let mut do_export_report_json = false;
        let mut do_export_pred_csv = false;
        let mut do_export_pred_json = false;
        let mut do_export_catalog_mat_csv = false;
        let mut do_export_catalog_mat_json = false;
        let mut do_export_catalog_oil_csv = false;
        let mut do_export_catalog_oil_json = false;
        let mut do_pick_calibration_csv = false;
        let mut do_run_calibration = false;
        let mut do_export_calibration_csv = false;
        let mut do_export_calibration_json = false;
        let mut do_monte_carlo = false;
        let mut pending_select: Option<usize> = None;

        ScrollArea::vertical().id_salt("leak::outer_scroll").auto_shrink([false, false]).max_height(ui.available_height()).show(ui, |ui| {
            ui.columns(3, |cols| {
                // Left: circuit list + runtime status
                egui::Frame::group(cols[0].style()).fill(Color32::from_gray(15)).show(&mut cols[0], |ui| {
                    ui.label(RichText::new("RUNTIME CIRCUITS").size(11.5).color(Color32::from_rgb(80,155,255)));
                    ui.separator();
                    for (i, name, app, comp, oil) in &circuits {
                        ui.push_id(format!("leak::circuit_row_{}", i), |ui| {
                            let sel = *i == self.leak_sel_idx;
                            let fill = if sel { Color32::from_rgb(30,60,100) } else { Color32::from_gray(25) };
                            if ui.add(Button::new(RichText::new(format!("{} [{} / {}]", name, comp, oil)).size(10.5)).fill(fill).min_size(Vec2::new(ui.available_width()-8.0,22.0))).clicked() {
                                pending_select = Some(*i);
                            }
                        });
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
                        ui.label(RichText::new(format!(
                            "{} | {} | p={:.1}bar ({}) | leak {:.3} L/min | risk {:.0}%",
                            r.name,
                            r.alert,
                            r.current_pressure_bar,
                            r.pressure_band,
                            r.leak_lpm,
                            r.rupture_probability_pct
                        )).size(10.0).color(c));
                        ui.label(RichText::new(format!(
                            "safe {:.1}-{:.1}bar target {:.1}bar | rupture at {} bar / {} h",
                            r.recommended_hold_min_bar,
                            r.recommended_hold_max_bar,
                            r.recommended_hold_target_bar,
                            r.rupture_pressure_bar.map(|v| format!("{v:.2}")).unwrap_or_else(|| "n/a".into()),
                            r.rupture_elapsed_h.map(|v| format!("{v:.3}")).unwrap_or_else(|| "n/a".into())
                        )).size(9.5).color(Color32::from_gray(160)));
                    }
                    if !alerts.is_empty() {
                        ui.separator();
                        ui.label(RichText::new("ACTIVE ALERTS").size(11.0).color(Color32::RED));
                        for a in &alerts {
                            ui.label(RichText::new(format!("{}: {}", a.circuit_name, a.message)).size(10.0).color(Color32::RED));
                        }
                    }

                    ui.separator();
                    ui.label(RichText::new("3D PHYSICS/LEAK VIEW").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ui.horizontal(|ui| {
                        ui.label("Yaw");
                        ui.add(Slider::new(&mut self.leak_view_yaw_deg, -180.0..=180.0).show_value(true));
                        ui.label("Pitch");
                        ui.add(Slider::new(&mut self.leak_view_pitch_deg, -80.0..=80.0).show_value(true));
                        ui.label("Zoom");
                        ui.add(Slider::new(&mut self.leak_view_zoom, 0.4..=2.5).show_value(true));
                    });
                    let (rect, _resp) = ui.allocate_exact_size(
                        Vec2::new(ui.available_width() - 4.0, 180.0),
                        Sense::hover(),
                    );
                    let p = ui.painter_at(rect);
                    p.rect_filled(rect, 4.0, Color32::from_gray(10));
                    p.rect_stroke(rect, 4.0, Stroke::new(1.0, Color32::from_gray(60)));

                    let yaw = self.leak_view_yaw_deg.to_radians();
                    let pitch = self.leak_view_pitch_deg.to_radians();
                    let zoom = self.leak_view_zoom;
                    let center = rect.center();
                    let scale = 70.0 * zoom;

                    let project = |x: f32, y: f32, z: f32| -> Pos2 {
                        let cy = yaw.cos();
                        let sy = yaw.sin();
                        let cp = pitch.cos();
                        let sp = pitch.sin();

                        let xr = x * cy + z * sy;
                        let zr = -x * sy + z * cy;
                        let yr = y * cp - zr * sp;
                        let zr2 = y * sp + zr * cp + 8.0;
                        let f = scale / zr2.max(0.3);
                        Pos2::new(center.x + xr * f, center.y - yr * f)
                    };

                    let axis = [
                        ([-2.2, 0.0, 0.0], [2.2, 0.0, 0.0], Color32::from_rgb(220, 90, 90)),
                        ([0.0, 0.0, -2.2], [0.0, 0.0, 2.2], Color32::from_rgb(90, 220, 90)),
                        ([0.0, 0.0, 0.0], [0.0, 2.2, 0.0], Color32::from_rgb(90, 140, 255)),
                    ];
                    for (a, b, c) in axis {
                        p.line_segment(
                            [project(a[0], a[1], a[2]), project(b[0], b[1], b[2])],
                            Stroke::new(1.2, c),
                        );
                    }

                    for (i, r) in reports.iter().take(8).enumerate() {
                        let x = -1.8 + i as f32 * 0.5;
                        let h = (r.leak_lpm as f32 * 0.9).clamp(0.08, 1.8);
                        let risk = (r.rupture_probability_pct as f32 / 100.0).clamp(0.0, 1.0);
                        let col = Color32::from_rgb(
                            (90.0 + 150.0 * risk) as u8,
                            (210.0 - 120.0 * risk) as u8,
                            90,
                        );

                        let y0 = 0.0f32;
                        let y1 = h;
                        let z0 = -0.15f32;
                        let z1 = 0.15f32;
                        let x0 = x;
                        let x1 = x + 0.3;

                        let corners = [
                            project(x0, y0, z0),
                            project(x1, y0, z0),
                            project(x1, y1, z0),
                            project(x0, y1, z0),
                            project(x0, y0, z1),
                            project(x1, y0, z1),
                            project(x1, y1, z1),
                            project(x0, y1, z1),
                        ];

                        let edges = [
                            (0, 1), (1, 2), (2, 3), (3, 0),
                            (4, 5), (5, 6), (6, 7), (7, 4),
                            (0, 4), (1, 5), (2, 6), (3, 7),
                        ];
                        for (a, b) in edges {
                            p.line_segment([corners[a], corners[b]], Stroke::new(1.0, col));
                        }
                    }
                });

                // Middle: manual params
                egui::Frame::group(cols[1].style()).fill(Color32::from_gray(15)).show(&mut cols[1], |ui| {
                    ui.label(RichText::new("MANUAL ENGINEERING INPUT").size(11.5).color(Color32::from_rgb(80,155,255)));
                    ui.label(RichText::new("Use this for production-calibrated pressure/oring/oil settings").size(9.8).color(Color32::from_gray(140)));
                    ui.separator();

                    let oil_types = OilType::all();
                    ComboBox::from_id_salt("leak::manual_oil_type")
                        .width(180.0)
                        .selected_text(
                            oil_types
                                .get(self.leak_manual.oil_type_idx)
                                .copied()
                                .unwrap_or(OilType::Custom)
                                .name(),
                        )
                        .show_ui(ui, |ui| {
                            for (i, o) in oil_types.iter().enumerate() {
                                ui.selectable_value(&mut self.leak_manual.oil_type_idx, i, o.name());
                            }
                        });
                    ui.label(RichText::new("Oil Type").size(9.6).color(Color32::from_gray(150)));

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

                    if let Err(msg) = &manual_validation {
                        ui.label(RichText::new(format!("⚠ {}", msg)).size(9.8).color(Color32::YELLOW));
                    }

                    if ui.add_enabled(
                        manual_ok,
                        Button::new(RichText::new("APPLY MANUAL PARAMETERS").size(11.0)).fill(Color32::from_rgb(20,80,30)).min_size(Vec2::new(ui.available_width()-8.0,28.0)),
                    ).clicked() {
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

                    ui.separator();
                    ui.label(RichText::new("O-RING / SEAL ENGINEERING RENDER (ASCII CAD)").size(11.0).color(Color32::from_rgb(80,155,255)));
                    egui::CollapsingHeader::new("ASCII legend guide")
                        .default_open(false)
                        .show(ui, |ui| {
                            ui.label(RichText::new("CAD symbols:").size(9.4).color(Color32::from_gray(180)));
                            ui.label(RichText::new("P = ponto de maior pressao local").size(9.2).color(Color32::from_gray(170)));
                            ui.label(RichText::new("T = ponto de maior fragilidade (tear-up)").size(9.2).color(Color32::from_gray(170)));
                            ui.label(RichText::new("X/!/*/:/. = criticidade decrescente da fragilidade").size(9.2).color(Color32::from_gray(170)));
                            ui.separator();
                            ui.label(RichText::new("Timeline symbols:").size(9.4).color(Color32::from_gray(180)));
                            ui.label(RichText::new("1o char = pressao: ^ alto, : medio, . baixo").size(9.2).color(Color32::from_gray(170)));
                            ui.label(RichText::new("2o char = risco: ! alto, * medio, . baixo").size(9.2).color(Color32::from_gray(170)));
                            ui.label(RichText::new("3o char = vazao: ~ alta, - media, . baixa").size(9.2).color(Color32::from_gray(170)));
                        });
                    if let (Some(c), Some(r)) = (&selected_circuit, &selected_report) {
                        let (cad, points) = Self::leak_ascii_cad(c, r);
                        ui.label(
                            RichText::new("Use scroll horizontal para ver o desenho ASCII completo sem corte.")
                                .size(9.2)
                                .color(Color32::from_gray(150)),
                        );
                        ScrollArea::both()
                            .id_salt("leak::ascii_cad_scroll")
                            .max_height(245.0)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                ui.set_min_width(360.0);
                                ui.label(RichText::new(cad).size(8.8).monospace().color(Color32::from_gray(190)));
                            });
                        for p in points {
                            ui.label(RichText::new(p).size(9.2).color(Color32::from_gray(170)));
                        }
                        ui.label(
                            RichText::new(format!(
                                "{} | material {} | cs {:.2}mm | squeeze {:.1}% | gap {:.3}mm | comp set {:.1}% | shore {:.0}A | life {:.0}h",
                                c.name,
                                c.spec.material.name(),
                                c.spec.cross_section_mm,
                                c.spec.squeeze_pct,
                                c.spec.extrusion_gap_mm,
                                c.spec.compression_set_pct,
                                c.spec.shore_a,
                                c.spec.design_life_hours
                            ))
                            .size(9.4)
                            .color(Color32::from_gray(170)),
                        );

                        ui.separator();
                        ui.label(
                            RichText::new("TEMPORAL ASCII EVOLUTION (pressure/risk/flow)")
                                .size(10.6)
                                .color(Color32::from_rgb(80, 155, 255)),
                        );
                        let t_now = self.bench.elapsed;
                        let t_min = (t_now - self.leak_timeback_window_s).max(0.0);
                        let stride = self.leak_timeback_stride.max(1);
                        let mut tb: Vec<(f64, f64, f64, f64)> = self
                            .leak_temporal_trace
                            .iter()
                            .copied()
                            .filter(|(t, _, _, _)| *t >= t_min)
                            .step_by(stride)
                            .collect();
                        if self.leak_timeback_latest_first {
                            tb.reverse();
                        }

                        ui.label(
                            RichText::new(format!(
                                "window {:.0}s | samples {} | order {}",
                                self.leak_timeback_window_s,
                                tb.len(),
                                if self.leak_timeback_latest_first {
                                    "new->old"
                                } else {
                                    "old->new"
                                }
                            ))
                            .size(9.6)
                            .color(Color32::from_gray(160)),
                        );

                        let mut timeline = String::new();
                        timeline.push_str("legend [P][R][F]: P(^ high, : mid, . low) | R(! high, * mid, . low) | F(~ high, - mid, . low)\n");
                        timeline.push_str("idx | time(s) | ascii\n");
                        for (i, (tt, p, risk, leak)) in tb.iter().enumerate() {
                            let p_ratio = (p / c.pressure.rupture_bar.max(1e-6)).clamp(0.0, 1.0);
                            let r_ratio = (risk / 100.0).clamp(0.0, 1.0);
                            let f_ratio = (leak / 1.5).clamp(0.0, 1.0);
                            let pch = if p_ratio > 0.85 {
                                '^'
                            } else if p_ratio > 0.55 {
                                ':'
                            } else {
                                '.'
                            };
                            let rch = if r_ratio > 0.80 {
                                '!'
                            } else if r_ratio > 0.45 {
                                '*'
                            } else {
                                '.'
                            };
                            let fch = if f_ratio > 0.80 {
                                '~'
                            } else if f_ratio > 0.45 {
                                '-'
                            } else {
                                '.'
                            };
                            timeline.push_str(&format!("{:03} | {:7.2} | {}{}{}\n", i, tt, pch, rch, fch));
                        }
                        ScrollArea::both()
                            .id_salt("leak::timeback_ascii_scroll")
                            .max_height(155.0)
                            .auto_shrink([false, false])
                            .show(ui, |ui| {
                                ui.set_min_width(340.0);
                                ui.label(
                                    RichText::new(timeline)
                                        .size(8.9)
                                        .monospace()
                                        .color(Color32::from_gray(178)),
                                );
                            });

                        let p_points: Vec<[f64; 2]> = tb.iter().map(|(t, p, _, _)| [*t, *p]).collect();
                        let r_points: Vec<[f64; 2]> = tb.iter().map(|(t, _, r, _)| [*t, *r]).collect();
                        let f_points: Vec<[f64; 2]> = tb.iter().map(|(t, _, _, f)| [*t, *f]).collect();
                        ui.label(
                            RichText::new("TIMEBACK VISUAL AID (P/R/F)")
                                .size(10.4)
                                .color(Color32::from_rgb(110, 180, 255)),
                        );
                        Plot::new("leak::timeback_plot")
                            .height(150.0)
                            .allow_drag(false)
                            .allow_zoom(false)
                            .allow_scroll(false)
                            .show_axes([true, true])
                            .show(ui, |plot_ui| {
                                let lp = PlotPoints::from_iter(p_points.iter().copied());
                                let lr = PlotPoints::from_iter(r_points.iter().copied());
                                let lf = PlotPoints::from_iter(f_points.iter().copied());
                                plot_ui.line(
                                    Line::new(lp)
                                        .color(Color32::from_rgb(255, 170, 60))
                                        .name("Pressure bar"),
                                );
                                plot_ui.line(
                                    Line::new(lr)
                                        .color(Color32::from_rgb(255, 80, 80))
                                        .name("Risk %"),
                                );
                                plot_ui.line(
                                    Line::new(lf)
                                        .color(Color32::from_rgb(90, 220, 180))
                                        .name("Leak LPM"),
                                );
                            });
                    } else {
                        ui.label(RichText::new("Select a circuit with runtime data to render O-ring CAD").size(9.8).color(Color32::from_gray(145)));
                    }
                });

                // Right: custom circuit + scenario table
                egui::Frame::group(cols[2].style()).fill(Color32::from_gray(15)).show(&mut cols[2], |ui| {
                    ui.label(RichText::new("CUSTOM CIRCUIT BUILDER").size(11.5).color(Color32::from_rgb(80,155,255)));
                    ui.push_id("leak::custom_name_row", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Name");
                        ui.add_sized([120.0, 20.0], TextEdit::singleline(&mut self.leak_custom.name));
                    });
                    });
                    ui.push_id("leak::custom_app_row", |ui| {
                    ui.horizontal(|ui| {
                        ui.label("App");
                        ui.add_sized([180.0, 20.0], TextEdit::singleline(&mut self.leak_custom.application));
                    });
                    });
                    const COMP_NAMES: [&str; 3] = ["O-ring", "Seal", "A/C Hose"];
                    let oil_types = OilType::all();
                    let materials = OringMaterial::all();
                    ComboBox::from_id_salt("leak::custom_component").selected_text(COMP_NAMES[self.leak_custom.component_idx.min(2)]).show_ui(ui, |ui| {
                        for (i, n) in COMP_NAMES.iter().enumerate() { ui.selectable_value(&mut self.leak_custom.component_idx, i, *n); }
                    });
                    ComboBox::from_id_salt("leak::custom_oil").selected_text(
                        oil_types
                            .get(self.leak_custom.oil_type_idx)
                            .copied()
                            .unwrap_or(OilType::Custom)
                            .name(),
                    ).show_ui(ui, |ui| {
                        for (i, o) in oil_types.iter().enumerate() { ui.selectable_value(&mut self.leak_custom.oil_type_idx, i, o.name()); }
                    });
                    ComboBox::from_id_salt("leak::custom_material").selected_text(
                        materials
                            .get(self.leak_custom.material_idx)
                            .copied()
                            .unwrap_or(OringMaterial::Nbr)
                            .name(),
                    ).show_ui(ui, |ui| {
                        for (i, m) in materials.iter().enumerate() { ui.selectable_value(&mut self.leak_custom.material_idx, i, m.name()); }
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

                    if let Err(msg) = &custom_validation {
                        ui.label(RichText::new(format!("⚠ {}", msg)).size(9.8).color(Color32::YELLOW));
                    }

                    if ui.add_enabled(
                        custom_ok,
                        Button::new(RichText::new("ADD CUSTOM CIRCUIT").size(11.0)).fill(Color32::from_rgb(60,40,10)).min_size(Vec2::new(ui.available_width()-8.0,26.0)),
                    ).clicked() {
                        do_add_custom = true;
                    }

                    ui.separator();
                    ui.label(RichText::new("SCENARIO RANKING").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ScrollArea::vertical().id_salt("leak::scenario_ranking_scroll").max_height(190.0).auto_shrink([false, false]).show(ui, |ui| {
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
                    ui.label(RichText::new("3D SCENARIO SIMULATION VIEW").size(10.6).color(Color32::from_rgb(80,155,255)));
                    let (srect, _sresp) = ui.allocate_exact_size(
                        Vec2::new(ui.available_width() - 4.0, 170.0),
                        Sense::hover(),
                    );
                    let sp = ui.painter_at(srect);
                    sp.rect_filled(srect, 4.0, Color32::from_gray(10));
                    sp.rect_stroke(srect, 4.0, Stroke::new(1.0, Color32::from_gray(60)));

                    let center = srect.center();
                    let yaw = 28.0f32.to_radians();
                    let pitch = 20.0f32.to_radians();
                    let scale = 65.0f32;
                    let project = |x: f32, y: f32, z: f32| -> Pos2 {
                        let cy = yaw.cos();
                        let sy = yaw.sin();
                        let cp = pitch.cos();
                        let spv = pitch.sin();

                        let xr = x * cy + z * sy;
                        let zr = -x * sy + z * cy;
                        let yr = y * cp - zr * spv;
                        let zr2 = y * spv + zr * cp + 8.5;
                        let f = scale / zr2.max(0.35);
                        Pos2::new(center.x + xr * f, center.y - yr * f)
                    };

                    let axis = [
                        ([-2.4, 0.0, 0.0], [2.4, 0.0, 0.0], Color32::from_rgb(220, 90, 90)),
                        ([0.0, 0.0, -2.4], [0.0, 0.0, 2.4], Color32::from_rgb(90, 220, 90)),
                        ([0.0, 0.0, 0.0], [0.0, 2.4, 0.0], Color32::from_rgb(90, 140, 255)),
                    ];
                    for (a, b, c) in axis {
                        sp.line_segment(
                            [project(a[0], a[1], a[2]), project(b[0], b[1], b[2])],
                            Stroke::new(1.1, c),
                        );
                    }

                    for (i, scen) in predictions.iter().take(36).enumerate() {
                        let x = -2.0 + (i % 12) as f32 * 0.36;
                        let z = -1.6 + (i / 12) as f32 * 1.0;
                        let risk = (scen.final_rupture_probability_pct as f32 / 100.0).clamp(0.0, 1.0);
                        let peak = (scen.peak_pressure_bar as f32 / 320.0).clamp(0.0, 1.0);
                        let h = (0.15 + 1.9 * (0.6 * risk + 0.4 * peak)).clamp(0.1, 2.2);
                        let col = Color32::from_rgb(
                            (80.0 + 170.0 * risk) as u8,
                            (210.0 - 140.0 * risk) as u8,
                            (90.0 + 90.0 * (1.0 - peak)) as u8,
                        );

                        let p0 = project(x, 0.0, z);
                        let p1 = project(x, h, z);
                        sp.line_segment([p0, p1], Stroke::new(2.0, col));
                        sp.circle_filled(p1, 2.5, col);
                    }
                    ui.label(
                        RichText::new("3D axes: X=scenario index, Y=risk/pressure amplitude, Z=scenario group")
                            .size(9.0)
                            .color(Color32::from_gray(145)),
                    );
                    ui.separator();
                    ui.label(RichText::new("EXPORT REPORTS").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ui.push_id("leak::export_reports_row", |ui| {
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("CSV Runtime").clicked() { do_export_report_csv = true; }
                        if ui.button("JSON Runtime").clicked() { do_export_report_json = true; }
                        if ui.button("CSV Prediction").clicked() { do_export_pred_csv = true; }
                        if ui.button("JSON Prediction").clicked() { do_export_pred_json = true; }
                    });
                    });
                    ui.label(RichText::new("ENGINEERING CATALOG EXPORT").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ui.push_id("leak::export_catalog_row", |ui| {
                    ui.horizontal_wrapped(|ui| {
                        if ui.button("CSV Materials").clicked() { do_export_catalog_mat_csv = true; }
                        if ui.button("JSON Materials").clicked() { do_export_catalog_mat_json = true; }
                        if ui.button("CSV Oils").clicked() { do_export_catalog_oil_csv = true; }
                        if ui.button("JSON Oils").clicked() { do_export_catalog_oil_json = true; }
                    });
                    });

                    ui.separator();
                    ui.label(RichText::new("CALIBRATION MODE (BENCH CSV)").size(11.0).color(Color32::from_rgb(80,155,255)));
                    ui.push_id("leak::calib_controls", |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("Select CSV").clicked() { do_pick_calibration_csv = true; }
                        let mut run_cal = ui.add_enabled(!self.leak_calibration_csv_path.is_empty(), Button::new("Run Auto Calibration"));
                        if self.leak_calibration_csv_path.is_empty() {
                            run_cal = run_cal.on_disabled_hover_text("Selecione um CSV de bancada antes de calibrar");
                        }
                        if run_cal.clicked() {
                            do_run_calibration = true;
                        }
                    });
                    });
                    if self.leak_calibration_csv_path.is_empty() {
                        ui.label(RichText::new("No CSV selected").size(9.8).color(Color32::from_gray(145)));
                    } else {
                        ui.label(RichText::new(format!("CSV: {}", self.leak_calibration_csv_path)).size(9.6).color(Color32::LIGHT_BLUE));
                    }
                    if let Some(rep) = &calibration_report {
                        ui.label(RichText::new(format!(
                            "Calibrated circuits: {} | samples: {}",
                            rep.calibrated_circuits, rep.total_samples
                        )).size(9.8).color(Color32::YELLOW));
                        for r in rep.circuit_reports.iter().take(5) {
                            ui.label(RichText::new(format!(
                                "{} | RMSE {:.4} LPM | MAPE {:.2}% | rupture acc {:.1}%",
                                r.circuit_name, r.rmse_leak_lpm, r.mape_leak_pct, r.rupture_accuracy_pct
                            )).size(9.4).color(Color32::from_gray(165)));
                        }
                        ui.push_id("leak::export_calibration_row", |ui| {
                        ui.horizontal_wrapped(|ui| {
                            if ui.button("CSV Calibration Report").clicked() { do_export_calibration_csv = true; }
                            if ui.button("JSON Calibration Report").clicked() { do_export_calibration_json = true; }
                        });
                        });
                    }
                });
            });
        });

        if let Some(i) = pending_select {
            self.cmds.push(Cmd::LeakSelectCircuit(i));
        }
        if do_apply && manual_ok {
            self.cmds.push(Cmd::LeakApplyManual);
        }
        if do_predict {
            self.cmds.push(Cmd::LeakPredictScenarios);
        }
        if do_add_custom && custom_ok {
            self.cmds.push(Cmd::LeakAddCustomCircuit);
        }
        if do_export_report_csv {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("leak_report_{}.csv", ts);
            if let Some(path) = Self::pick_save_path(&name, "csv") {
                self.cmds.push(Cmd::LeakExportReportCsv(path));
            }
        }
        if do_export_report_json {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("leak_report_{}.json", ts);
            if let Some(path) = Self::pick_save_path(&name, "json") {
                self.cmds.push(Cmd::LeakExportReportJson(path));
            }
        }
        if do_export_pred_csv {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("leak_predictions_{}.csv", ts);
            if let Some(path) = Self::pick_save_path(&name, "csv") {
                self.cmds.push(Cmd::LeakExportPredCsv(path));
            }
        }
        if do_export_pred_json {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("leak_predictions_{}.json", ts);
            if let Some(path) = Self::pick_save_path(&name, "json") {
                self.cmds.push(Cmd::LeakExportPredJson(path));
            }
        }
        if do_monte_carlo {
            self.cmds.push(Cmd::LeakRunMonteCarlo);
        }
        if do_export_catalog_mat_csv {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("materials_catalog_{}.csv", ts);
            if let Some(path) = Self::pick_save_path(&name, "csv") {
                self.cmds.push(Cmd::LeakExportCatalogMaterialsCsv(path));
            }
        }
        if do_export_catalog_mat_json {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("materials_catalog_{}.json", ts);
            if let Some(path) = Self::pick_save_path(&name, "json") {
                self.cmds.push(Cmd::LeakExportCatalogMaterialsJson(path));
            }
        }
        if do_export_catalog_oil_csv {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("oils_catalog_{}.csv", ts);
            if let Some(path) = Self::pick_save_path(&name, "csv") {
                self.cmds.push(Cmd::LeakExportCatalogOilsCsv(path));
            }
        }
        if do_export_catalog_oil_json {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("oils_catalog_{}.json", ts);
            if let Some(path) = Self::pick_save_path(&name, "json") {
                self.cmds.push(Cmd::LeakExportCatalogOilsJson(path));
            }
        }
        if do_pick_calibration_csv {
            if let Some(path) = Self::pick_open_path("csv") {
                self.leak_calibration_csv_path = path;
                self.leak_note = "CSV de bancada selecionado".into();
            }
        }
        if do_run_calibration {
            self.cmds.push(Cmd::LeakCalibrateFromCsv(
                self.leak_calibration_csv_path.clone(),
            ));
        }
        if do_export_calibration_csv {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("leak_calibration_report_{}.csv", ts);
            if let Some(path) = Self::pick_save_path(&name, "csv") {
                self.cmds.push(Cmd::LeakExportCalibrationCsv(path));
            }
        }
        if do_export_calibration_json {
            let ts = Local::now().format("%Y%m%d_%H%M%S");
            let name = format!("leak_calibration_report_{}.json", ts);
            if let Some(path) = Self::pick_save_path(&name, "json") {
                self.cmds.push(Cmd::LeakExportCalibrationJson(path));
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// TAB SENSORS
// ═════════════════════════════════════════════════════════════════════════════
impl App {
    fn tab_sensors(&self, ui: &mut Ui) {
        let gps_ok = !matches!(
            self.bench.gps.fix_quality,
            auto_breaking::gps::GpsFixQuality::NoFix
        );
        let imu_ok = !self.bench.imu.accel_fault && !self.bench.imu.gyro_fault;
        let radar_ok = self.bench.radar.total_targets() > 0;
        let lidar_ok = self.bench.lidar.points_per_scan > 0;
        let healthy_count = [gps_ok, imu_ok, radar_ok, lidar_ok]
            .into_iter()
            .filter(|v| *v)
            .count();
        let sys_col = if healthy_count >= 4 {
            Color32::GREEN
        } else if healthy_count >= 3 {
            Color32::YELLOW
        } else {
            Color32::RED
        };
        ui.horizontal_wrapped(|ui| {
            ui.label(
                RichText::new(format!("Sensor health {}/4", healthy_count))
                    .size(12.0)
                    .color(sys_col)
                    .strong(),
            );
            for (name, ok) in [
                ("GPS", gps_ok),
                ("IMU", imu_ok),
                ("RADAR", radar_ok),
                ("LIDAR", lidar_ok),
            ] {
                ui.label(
                    RichText::new(format!(
                        "{}:{}",
                        name,
                        if ok { "OK" } else { "CHECK" }
                    ))
                    .size(10.8)
                    .color(if ok { Color32::GREEN } else { Color32::YELLOW }),
                );
            }
        });
        ui.separator();
        ScrollArea::vertical()
            .auto_shrink([false, false])
            .show(ui, |ui| {
                ui.columns(2, |cols| {
                    // GPS
                    egui::Frame::group(cols[0].style())
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
                    egui::Frame::group(cols[1].style())
                        .fill(Color32::from_gray(15))
                        .show(&mut cols[1], |ui| {
                            let imu = &self.bench.imu;
                            ui.label(
                                RichText::new("📐 IMU — Madgwick AHRS 9-DOF")
                                    .size(12.0)
                                    .color(Color32::from_rgb(80, 155, 255)),
                            );
                            ui.columns(3, |c| {
                                egui::Frame::dark_canvas(c[0].style()).show(&mut c[0], |ui| {
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
                                egui::Frame::dark_canvas(c[1].style()).show(&mut c[1], |ui| {
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
                                egui::Frame::dark_canvas(c[2].style()).show(&mut c[2], |ui| {
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
        let mut do_hdw_up = false;
        let mut do_hdw_dn = false;
        let readiness = self.ad_readiness_issues();
        let can_engage_ad = self.bench.engine_running() && !matches!(self.bench.tcm.direction, Direction::Park);

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
                            if !readiness.is_empty() {
                                ui.add_space(4.0);
                                ui.label(
                                    RichText::new("READINESS / WHY IT MAY LOOK INACTIVE")
                                        .size(10.5)
                                        .color(Color32::from_rgb(255, 190, 90))
                                        .strong(),
                                );
                                for item in &readiness {
                                    ui.label(
                                        RichText::new(format!("• {}", item))
                                            .size(10.0)
                                            .color(Color32::from_gray(210)),
                                    );
                                }
                                ui.separator();
                            }
                            let fill = if eng {
                                Color32::from_rgb(100, 20, 20)
                            } else {
                                Color32::from_rgb(20, 80, 20)
                            };
                            let mut engage_resp = ui.add_enabled(
                                eng || can_engage_ad,
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
                            );
                            if !eng && !can_engage_ad {
                                engage_resp = engage_resp.on_disabled_hover_text(
                                    "Para engatar o AD, deixe o motor em RUN e saia de Park.",
                                );
                            }
                            if engage_resp.clicked() {
                                do_engage = true;
                            }
                            ui.label(
                                RichText::new(if lead.is_finite() {
                                    "ACC esta usando alvo frontal e TTC/THW para modular torque/freio."
                                } else {
                                    "Sem alvo frontal: ACC acelera ate a velocidade configurada e LKA depende de lane confidence."
                                })
                                .size(10.0)
                                .color(Color32::from_gray(175)),
                            );
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
                            ui.horizontal(|ui| {
                                ui.label(RichText::new("Headway").size(11.0).color(Color32::GREEN));
                                if ui.small_button("-0.2s").clicked() {
                                    do_hdw_dn = true;
                                }
                                if ui.small_button("+0.2s").clicked() {
                                    do_hdw_up = true;
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
                            if !det_s {
                                ui.label(
                                    RichText::new("LKA em espera: nao ha faixa confiavel detectada.")
                                        .size(10.0)
                                        .color(Color32::YELLOW),
                                );
                            }
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
                                if bsml { "⚠ OBJECT" } else { "clear  " },
                                if bsml {
                                    Color32::YELLOW
                                } else {
                                    Color32::GREEN
                                },
                            );
                            digital_readout(
                                ui,
                                "BSM R",
                                if bsmr { "⚠ OBJECT" } else { "clear  " },
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
                                if det_s { "YES" } else { "NO" },
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
        if do_hdw_up {
            self.cmds.push(Cmd::AccHeadwaySet(acc_hdw + 0.2));
        }
        if do_hdw_dn {
            self.cmds.push(Cmd::AccHeadwaySet(acc_hdw - 0.2));
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
        let v_rng_s = self.v2x_range_ema;
        let v_loss_s = self.v2x_loss_ema;
        let v_lat_s = self.v2x_lat_ema;
        let mut link_score = 100.0;
        link_score -= (v_loss_s * 2.0).clamp(0.0, 60.0);
        link_score -= ((v_lat_s - 10.0) * 1.6).clamp(0.0, 40.0);
        link_score -= ((180.0 - v_rng_s) * 0.15).clamp(0.0, 20.0);
        let link_score = link_score.clamp(0.0, 100.0);
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

        ScrollArea::vertical()
            .id_salt("v2x::page_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
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
                    let link_col = if link_score >= 80.0 {
                        Color32::GREEN
                    } else if link_score >= 60.0 {
                        Color32::YELLOW
                    } else {
                        Color32::RED
                    };
                    ui.label(
                        RichText::new(format!(
                            "Smoothed link: {:.0}% | range {:.0}m | loss {:.1}% | latency {:.1}ms",
                            link_score, v_rng_s, v_loss_s, v_lat_s
                        ))
                        .size(10.6)
                        .color(link_col),
                    );
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
                        .id_salt("v2x::nearby_scroll")
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
                        .id_salt("v2x::events_scroll")
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
        let mut do_pick_flash = false;
        let mut do_flash = false;
        let mut do_clean = false;

        ScrollArea::vertical()
            .id_salt("uds::page_scroll")
            .auto_shrink([false, false])
            .show(ui, |ui| {
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
            ui.label(
                RichText::new("ECM FLASH / UPLOAD WORKFLOW")
                    .size(11.0)
                    .color(Color32::from_rgb(80, 155, 255)),
            );
            ui.horizontal_wrapped(|ui| {
                if ui.button("Select FW (.bin)").clicked() {
                    do_pick_flash = true;
                }
                if ui
                    .add_enabled(!self.uds_flash_path.is_empty(), Button::new("Flash Upload"))
                    .clicked()
                {
                    do_flash = true;
                }
                if ui.button("Clean (ClrDTC + ResetAdap)").clicked() {
                    do_clean = true;
                }
            });
            if self.uds_flash_path.is_empty() {
                ui.label(
                    RichText::new("FW file: none selected")
                        .size(10.0)
                        .color(Color32::from_gray(140)),
                );
            } else {
                ui.label(
                    RichText::new(format!("FW file: {}", self.uds_flash_path))
                        .size(9.5)
                        .color(Color32::LIGHT_BLUE),
                );
            }
            ui.separator();
            ScrollArea::vertical()
                .id_salt("uds::txrx_scroll")
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
                .id_salt("uds::event_log_scroll")
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
            });

        // Execute UDS send AFTER column closure released borrows
        if let Some((bytes, sa)) = send_bytes {
            let ts = elapsed;
            let _ = self.uds_send_and_log(sa, bytes, ts);
        }
        if do_pick_flash {
            if let Some(path) = Self::pick_open_path("bin") {
                self.uds_flash_path = path;
            }
        }
        if do_flash {
            let ts = elapsed;
            if matches!(self.hw_cfg.mode, io::hw::HwMode::Live) {
                if self.uds_flash_path.trim().is_empty() {
                    self.uds_log
                        .push((false, "            flash failed: selecione .bin".into()));
                } else {
                    match std::fs::read(&self.uds_flash_path) {
                        Ok(payload) => {
                            match io::live_runner::live_flash_ecm_firmware(
                                &self.hw_cfg,
                                Some(self.uds_sa),
                                &payload,
                            ) {
                                Ok(s) => self.uds_log.push((
                                    false,
                                    format!(
                                        "            LIVE flash ok: {} bytes / {} blocks / crc32=0x{:08X}",
                                        s.bytes_sent, s.blocks_sent, s.crc32
                                    ),
                                )),
                                Err(e) => self
                                    .uds_log
                                    .push((false, format!("            LIVE flash failed: {}", e))),
                            }
                        }
                        Err(e) => self.uds_log.push((
                            false,
                            format!("            flash failed: leitura firmware: {}", e),
                        )),
                    }
                }
            } else {
                match self.run_uds_flash_pipeline(self.uds_sa, ts) {
                    Ok(msg) => self.uds_log.push((false, format!("            {}", msg))),
                    Err(e) => self
                        .uds_log
                        .push((false, format!("            flash failed: {}", e))),
                }
            }
        }
        if do_clean {
            let ts = elapsed;
            if matches!(self.hw_cfg.mode, io::hw::HwMode::Live) {
                match io::live_runner::live_clean_ecm(&self.hw_cfg, Some(self.uds_sa)) {
                    Ok(()) => self
                        .uds_log
                        .push((false, "            LIVE clean ok".into())),
                    Err(e) => self
                        .uds_log
                        .push((false, format!("            LIVE clean failed: {}", e))),
                }
            } else {
                let _ = self.uds_send_and_log(self.uds_sa, vec![0x14, 0xFF, 0xFF, 0xFF], ts);
                let _ = self.uds_send_and_log(self.uds_sa, vec![0x31, 0x01, 0xDF, 0x04], ts);
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
                &self.pl_hyd,
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
            for (i, (v, lbl, c, y_max)) in ld.iter().enumerate() {
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
                Plot::new(format!("plots::left::{}::{}", i, lbl))
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
            for (i, (v, lbl, c, y_max)) in rd.iter().enumerate() {
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
                Plot::new(format!("plots::right::{}::{}", i, lbl))
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
