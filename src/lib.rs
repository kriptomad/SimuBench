//! Heavy Machinery ECU Simulation Bench — central orchestrator.
//! Instantiates every ECU module, routes J1939/CAN frames through the gateway,
//! and drives the physical simulation (engine, drivetrain, hydraulics, implements).

use std::time::Instant;

// ── Module declarations (one file per real ECU / system) ────────────────────
pub mod boot_sequence;
pub mod can_gateway;
pub mod ecu_abs;
pub mod ecu_bcm;
pub mod ecu_ecm;
pub mod ecu_hcm;
pub mod ecu_icm;
pub mod ecu_tcm;
pub mod implement;
pub mod j1939;
pub mod network_mgmt;
pub mod nvm;
pub mod uds;

// ── Sensor suite (new) ───────────────────────────────────────────────────────
pub mod autonomous;
pub mod camera;
pub mod can_network;
pub mod ecu_vcm;
pub mod gps;
pub mod imu;
pub mod leak_physics;
pub mod lidar;
pub mod observability;
pub mod radar;
pub mod sim_core;
pub mod v2x_telematics;

// Legacy passenger-car modules (kept for reference — not used by heavy bench)
#[allow(dead_code)]
mod adas;
#[allow(dead_code)]
mod can_bus;
#[allow(dead_code)]
mod chassis;
pub mod ecm_heavy;
#[allow(dead_code)]
mod engine;
#[allow(dead_code)]
mod sensors;
#[allow(dead_code)]
mod transmission;
#[allow(dead_code)]
mod vehicle;

// ── Re-exports consumed by main.rs ──────────────────────────────────────────
pub use autonomous::{AutonomousController, FeatureState, LaneInfo, SaeLevel, SafetyStatus};
pub use boot_sequence::{
    BootEvent, BootEventKind, BootSequence, EcuBootRecord, EcuBootStage, IgnitionState,
};
pub use camera::{
    CameraSystem, DetectedObject, DetectedSign, LaneDetection, ObjectClass, SensorFusion,
};
pub use can_gateway::{BusError, BusState, CanGateway, CanNode};
pub use can_network::{BusId, BusState as CanBusState, CanBus, CanErrorKind, CanNetwork, CanSpeed};
pub use ecu_abs::{EcuAbs, EspCondition};
pub use ecu_bcm::{ChargingState, EcuBcm, WiperSpeed};
pub use ecu_ecm::{AftertreatmentState, EcuEcm, FuelMap, GovernorMode};
pub use ecu_hcm::{EcuHcm, HydActuator, PumpMode};
pub use ecu_icm::{EcuIcm, LampColor};
pub use ecu_tcm::{
    AutoShiftMode, ClutchState, Direction, EcuTcm, GearRange, ShiftQuality, TransmissionType,
};
pub use ecu_vcm::{EcuVcm, VehicleMode};
pub use gps::{GpsFixQuality, GpsModule, Satellite};
pub use implement::{HitchMode, ImplementControl, ImplementType, PtoMode, ValveDirection};
pub use imu::Imu;
pub use j1939::addr as SaConstants;
pub use j1939::pgn as PgnConstants;
pub use j1939::{find_pgn, fmi_name, Dtc, DtcSeverity, J1939Bus, J1939Frame};
pub use leak_physics::{
    CircuitComponent, CircuitInput, CircuitResult, LeakAlert, LeakAlertLevel, LeakCircuit,
    LeakPhysicsRig, ManualCircuitParams, OilType, OringMaterial, OringSpec, PressureEnvelope,
    ScenarioPrediction,
};
pub use lidar::{LidarCluster, LidarObjectType, LidarSensor, WorldObstacle};
pub use network_mgmt::{BusNmState, NetworkManager, NmNode, NmState};
pub use nvm::{Adaptations, NvmStore};
pub use observability::{SimMetrics, StructuredEvent};
pub use radar::{RadarObjectClass, RadarSuite, RadarTarget, SimTrafficObj};
pub use uds::{DiagSession, FreezeFrame, SecurityLevel, UdsServer};
pub use v2x_telematics::{ConnectState, TelematicsModule, V2xModule};

// ── Fault injection catalogue ────────────────────────────────────────────────

/// All injectable fault conditions for the fault-simulation panel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultType {
    // Engine faults
    HighCoolantTemp,  // SPN 110 FMI 0 — coolant > 107°C
    LowOilPressure,   // SPN 100 FMI 1 — oil pressure < 80 kPa
    LowFuelPressure,  // SPN  94 FMI 1 — fuel delivery < 150 kPa
    CriticalDefLevel, // SPN 3361 FMI 1 — DEF < 2% → power derate
    HighDpfSoot,      // SPN 3251 FMI 16 — DPF > 80%
    // Transmission faults
    TransClutchOverheat, // clutch temp > 200°C
    TransNeutralFailure, // TCM cannot engage neutral
    // Electrical faults
    LowBatteryVoltage, // battery < 11V
    AlternatorFault,   // alternator < 12V while engine running
    // Hydraulic faults
    HighHydTemp,     // hydraulic fluid > 100°C
    LowHydLevel,     // hydraulic level < 20%
    HydFilterBypass, // filter ΔP > 6 bar
    // CAN faults
    BusOffInjection, // force bus-off condition
    KillEcm,         // power off ECM node
    KillTcm,         // power off TCM node
    // None selected
    None,
}

impl std::fmt::Display for FaultType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            FaultType::HighCoolantTemp => "High Coolant Temp (SPN110 FMI0)",
            FaultType::LowOilPressure => "Low Oil Pressure (SPN100 FMI1)",
            FaultType::LowFuelPressure => "Low Fuel Pressure (SPN94 FMI1)",
            FaultType::CriticalDefLevel => "Critical DEF Level (SPN3361 FMI1)",
            FaultType::HighDpfSoot => "High DPF Soot (SPN3251 FMI16)",
            FaultType::TransClutchOverheat => "Trans Clutch Overheat",
            FaultType::TransNeutralFailure => "Trans Neutral Failure",
            FaultType::LowBatteryVoltage => "Low Battery Voltage",
            FaultType::AlternatorFault => "Alternator Fault",
            FaultType::HighHydTemp => "Hydraulic Fluid High Temp",
            FaultType::LowHydLevel => "Hydraulic Level Low",
            FaultType::HydFilterBypass => "Hyd Filter Bypass Open",
            FaultType::BusOffInjection => "CAN Bus-Off Injection",
            FaultType::KillEcm => "Kill ECM Node",
            FaultType::KillTcm => "Kill TCM Node",
            FaultType::None => "(no fault selected)",
        };
        write!(f, "{}", s)
    }
}

pub const FAULT_TYPES: &[FaultType] = &[
    FaultType::None,
    FaultType::HighCoolantTemp,
    FaultType::LowOilPressure,
    FaultType::LowFuelPressure,
    FaultType::CriticalDefLevel,
    FaultType::HighDpfSoot,
    FaultType::TransClutchOverheat,
    FaultType::TransNeutralFailure,
    FaultType::LowBatteryVoltage,
    FaultType::AlternatorFault,
    FaultType::HighHydTemp,
    FaultType::LowHydLevel,
    FaultType::HydFilterBypass,
    FaultType::BusOffInjection,
    FaultType::KillEcm,
    FaultType::KillTcm,
];

// ── Top-level Heavy Machinery Bench ─────────────────────────────────────────

pub struct HeavyMachinery {
    // ─ CAN infrastructure ─────────────────────────────────────────────────
    pub gateway: CanGateway,
    pub boot: BootSequence,
    pub net_mgmt: NetworkManager, // OSEK NM — coordinated sleep/wake

    // ─ ECU modules (each in its own .rs — mirrors real architecture) ──────
    pub ecm: EcuEcm, // Engine Control Module
    pub tcm: EcuTcm, // Transmission Control Module
    pub bcm: EcuBcm, // Body Control Module
    pub icm: EcuIcm, // Instrument Cluster
    pub hcm: EcuHcm, // Hydraulic Control Module
    pub abs: EcuAbs, // ABS / ESP / TCS

    // ─ Diagnostic services (one UDS server per ECU in real life) ──────────
    pub uds_ecm: UdsServer, // ECM diagnostic endpoint
    pub uds_tcm: UdsServer, // TCM diagnostic endpoint

    // ─ Non-volatile memory (shared EEPROM simulation for ECM) ─────────────
    pub nvm: NvmStore,

    // ─ Physical / implement systems ──────────────────────────────────────
    pub implement: ImplementControl,

    // ─ Simulation time ──────────────────────────────────────────────────
    pub elapsed: f64,

    // ─ Driver inputs ────────────────────────────────────────────────────
    pub throttle_pct: f64,
    pub brake_pct: f64,
    pub hitch_joystick: f64,
    pub loader_lift_cmd: f64,
    pub loader_tilt_cmd: f64,

    // ─ Fault simulation ─────────────────────────────────────────────────
    pub selected_fault: FaultType,
    pub fault_active: bool,

    // ── Sensor suite (autonomous / ADAS) ──────────────────────────────────
    pub gps: GpsModule,
    pub imu: Imu,
    pub radar: RadarSuite,
    pub lidar: LidarSensor,
    pub ad: AutonomousController,
    pub v2x: V2xModule,
    pub telematics: TelematicsModule,
    /// Multi-bus CAN network (HS-CAN powertrain/chassis, MS-CAN body, ISOBUS)
    pub can_net: CanNetwork,
    pub vcm: EcuVcm,
    pub camera: CameraSystem,
    pub fusion: SensorFusion,
    pub leak_rig: LeakPhysicsRig,
    pub leak_reports: Vec<CircuitResult>,
    pub metrics: SimMetrics,
}

impl HeavyMachinery {
    pub fn new() -> Self {
        let mut gw = CanGateway::new();
        gw.register_node(j1939::addr::ECM_1, "ECM #1");
        gw.register_node(j1939::addr::TRANSMISSION, "TCM");
        gw.register_node(j1939::addr::CAB, "BCM/CAB");
        gw.register_node(j1939::addr::INSTRUMENT, "ICM/DASH");
        gw.register_node(j1939::addr::HITCH, "HCM/HITCH");
        gw.register_node(j1939::addr::BRAKES, "ABS/ESP");
        gw.register_node(j1939::addr::ISOBUS_VT, "ISOBUS-VT");
        gw.register_node(j1939::addr::TASK_CTRL, "ISOBUS-TC");
        gw.register_node(j1939::addr::IMPLEMENT, "IMPLEMENT");

        HeavyMachinery {
            gateway: gw,
            boot: BootSequence::new(),
            net_mgmt: NetworkManager::new(),
            ecm: EcuEcm::new(),
            tcm: EcuTcm::new(),
            bcm: EcuBcm::new(),
            icm: EcuIcm::new(),
            hcm: EcuHcm::new(),
            abs: EcuAbs::new(),
            uds_ecm: UdsServer::new("ECM #1", j1939::addr::ECM_1),
            uds_tcm: UdsServer::new("TCM", j1939::addr::TRANSMISSION),
            nvm: NvmStore::new(),
            implement: ImplementControl::new(),
            elapsed: 0.0,
            throttle_pct: 0.0,
            brake_pct: 0.0,
            hitch_joystick: 0.0,
            loader_lift_cmd: 0.0,
            loader_tilt_cmd: 0.0,
            selected_fault: FaultType::None,
            fault_active: false,
            gps: GpsModule::new(),
            imu: Imu::new(),
            radar: RadarSuite::new(),
            lidar: LidarSensor::new_vlp16(),
            ad: AutonomousController::new(),
            v2x: V2xModule::new(),
            telematics: TelematicsModule::new(),
            can_net: CanNetwork::new(),
            vcm: EcuVcm::new(),
            camera: CameraSystem::new(),
            fusion: SensorFusion::new(),
            leak_rig: LeakPhysicsRig::with_default_machine_presets(),
            leak_reports: Vec::new(),
            metrics: SimMetrics::default(),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Master simulation tick — called every 16 ms (≈ 60 Hz).
    pub fn tick(&mut self, dt: f64) {
        let step_started = Instant::now();
        // 1. ── Boot sequence ─────────────────────────────────────────────────
        //    Updates ECU boot state machines, generates address-claim frames.
        let trans_neutral = self.tcm.is_neutral;
        self.boot.engine_running = self.ecm.is_running();
        let boot_frames = self.boot.tick(dt, &mut self.gateway, trans_neutral);
        for f in boot_frames {
            self.gateway.transmit(f);
        }

        // 2. ── Collect received frames from the previous gateway cycle ───────
        //    Each ECU sees frames dispatched in the previous tick (realistic
        //    CAN propagation behaviour).
        let received: Vec<J1939Frame> = self.gateway.dispatched.clone();

        // 3. ── ECM ───────────────────────────────────────────────────────────
        //    Load demand comes from TCM (drivetrain) + PTO shaft.
        let pto_load_nm = self.implement.pto_torque_demand();
        // Simplified drivetrain torque demand: TCM output / gear-ratio feedback
        let drivetrain_load_nm = if self.tcm.direction != Direction::Neutral {
            self.tcm.output_torque_nm * 0.05 // scaled back to engine shaft
        } else {
            0.0
        };
        let total_engine_load = (drivetrain_load_nm + pto_load_nm).clamp(0.0, 2000.0);

        let ecm_alive = self
            .boot
            .ecu_by_sa(j1939::addr::ECM_1)
            .map_or(false, |e| e.is_online());
        let ecm_frames = if ecm_alive
            || matches!(
                self.boot.ignition,
                IgnitionState::Cranking | IgnitionState::Running
            ) {
            let thr = if self.boot.ignition == IgnitionState::Cranking {
                25.0
            } else {
                self.throttle_pct
            };
            self.ecm.tick(thr, total_engine_load, dt)
        } else {
            Vec::new()
        };

        // 4. ── TCM ───────────────────────────────────────────────────────────
        let tcm_alive = self
            .boot
            .ecu_by_sa(j1939::addr::TRANSMISSION)
            .map_or(false, |e| e.is_online());
        let tcm_frames = if tcm_alive {
            self.tcm.tick(
                self.ecm.rpm,
                self.ecm.actual_torque_nm,
                self.throttle_pct,
                self.brake_pct,
                dt,
            )
        } else {
            Vec::new()
        };

        // 5. ── BCM ───────────────────────────────────────────────────────────
        let bcm_frames = self.bcm.tick(self.ecm.alternator_v, dt);

        // 6. ── ICM ───────────────────────────────────────────────────────────
        //    ICM gets the dispatched frames from last cycle and derives gauge
        //    values; it also has direct copies for fields not on CAN yet.
        let icm_frames = self.icm.tick(&received, dt);
        // Push data direct (values that haven't propagated via CAN yet)
        self.icm.engine_rpm = self.ecm.rpm;
        self.icm.vehicle_speed = self.tcm.ground_speed_kmh;
        self.icm.coolant_temp = self.ecm.coolant_temp_c;
        self.icm.oil_pressure_kpa = self.ecm.oil_pressure_kpa;
        self.icm.boost_kpa = self.ecm.boost_pressure_kpa;
        self.icm.fuel_level_pct = self.ecm.fuel_level_pct;
        self.icm.def_level_pct = self.ecm.def_level_pct;
        self.icm.engine_hours = self.ecm.engine_hours;
        self.icm.battery_volts = self.bcm.battery_voltage;
        self.icm.trans_gear = self.tcm.gear_label.clone();
        self.icm.mil_active = self.ecm.mil_active;
        self.icm.amber_active = self.ecm.amber_lamp;
        self.icm.red_active = self.ecm.red_lamp;
        self.icm.protect_active = self.ecm.protect_lamp;
        self.icm.active_dtc_count = self.ecm.active_dtcs.len() as u32;

        // 7. ── HCM ───────────────────────────────────────────────────────────
        let hcm_frames = self.hcm.tick(
            self.ecm.rpm,
            self.hitch_joystick,
            self.loader_lift_cmd,
            self.loader_tilt_cmd,
            dt,
        );
        self.run_leak_physics(dt);
        // Feed hitch cylinder position back to implement controller
        self.implement.hitch_position_pct = self.hcm.hitch_cylinder.position_pct();

        // 8. ── Implement controller ──────────────────────────────────────────
        self.implement
            .update(self.throttle_pct, self.tcm.ground_speed_kmh, dt);
        let impl_frames = self.implement.generate_j1939_frames(self.elapsed);

        // 9. ── ABS / ESP / TCS ───────────────────────────────────────────────
        let (tcs_cut, abs_frames) = self.abs.tick(
            self.tcm.ground_speed_kmh,
            self.brake_pct,
            self.throttle_pct,
            self.tcm.clutch_slip_pct, // use as steering proxy
            self.tcm.ground_speed_kmh * 0.01,
            0.0,
            dt,
        );
        let _effective_throttle = self.throttle_pct * (1.0 - tcs_cut);

        // 10. ── UDS diagnostic server ticks ───────────────────────────────────
        self.uds_ecm.tick(dt);
        self.uds_tcm.tick(dt);

        // 11. ── NVM periodic flush ────────────────────────────────────────────
        self.nvm.tick(self.elapsed, dt);
        // Sync engine hours to NVM every second
        if (self.elapsed % 60.0) < dt {
            self.nvm.write_f64("engine_hours", self.ecm.engine_hours);
            self.nvm
                .write_f64("odometer_km", self.tcm.total_distance_km + 12450.0);
        }

        // 12. ── Network Management ────────────────────────────────────────────
        let powered: Vec<u8> = self
            .boot
            .ecus
            .iter()
            .filter(|e| e.is_online())
            .map(|e| e.sa)
            .collect();
        self.net_mgmt.tick(&powered, dt);

        // 13. ── Submit all frames to the CAN gateway ──────────────────────────
        for f in ecm_frames {
            self.gateway.transmit(f);
        }
        for f in tcm_frames {
            self.gateway.transmit(f);
        }
        for f in bcm_frames {
            self.gateway.transmit(f);
        }
        for f in icm_frames {
            self.gateway.transmit(f);
        }
        for f in hcm_frames {
            self.gateway.transmit(f);
        }
        for f in impl_frames {
            self.gateway.transmit(f);
        }
        for f in abs_frames {
            self.gateway.transmit(f);
        }

        // 14. ── Gateway arbitrates and dispatches all frames ──────────────────
        self.gateway.tick(dt);

        // 15. ── Update gateway node records from boot state ───────────────────
        for ecu in &self.boot.ecus {
            if ecu.is_online() {
                self.gateway.set_node_online(ecu.sa);
            }
        }

        // 15b.── Multi-bus CAN network — route frames through proper buses ─────
        for f in self.gateway.dispatched.clone() {
            self.can_net.transmit(f.clone());
        }
        self.can_net.tick(dt);
        for ecu in &self.boot.ecus {
            if ecu.is_online() {
                self.can_net.set_online(ecu.sa);
            }
        }

        // 16. ── Sensor suite update ───────────────────────────────────────────
        let spd_ms = self.tcm.ground_speed_kmh / 3.6;

        // GPS
        self.gps
            .update(self.tcm.ground_speed_kmh, self.vehicle_heading(), dt);

        // IMU
        self.imu.update(
            self.vehicle_accel_ms2(),
            0.0,
            self.tcm.ground_speed_kmh * 0.01,
            self.vehicle_heading(),
            0.0,
            self.ecm.rpm,
            dt,
        );

        // RADAR — build traffic objects from implement traffic simulation
        let traffic = self.build_radar_traffic();
        self.radar.update(spd_ms, &traffic, dt);

        // LIDAR — build world obstacles and scan
        let world = self.build_lidar_world();
        self.lidar.update(&world, self.imu.pitch_deg, dt);

        // Camera + Sensor Fusion
        self.camera.update(
            self.tcm.ground_speed_kmh,
            self.vehicle_heading(),
            self.radar.ttc_front,
            self.elapsed,
            dt,
        );
        self.fusion
            .fuse(&self.camera, &self.radar.front_center.targets, spd_ms, dt);

        // AD controller
        let lane_off = self.imu.lateral_g * 0.2; // rough lane offset from lateral G
        let lane_conf = if self.tcm.ground_speed_kmh > 5.0 {
            0.92
        } else {
            0.0
        };
        let fused_ttc = self.fusion.ttc_critical.min(self.radar.ttc_front);
        let fused_lead = self.fusion.lead_dist_m.min(self.radar.closest_front_m);
        let ttc = fused_ttc;
        let lead_spd = if self.radar.front_center.closest_inpath().is_some() {
            (self.tcm.ground_speed_kmh - 5.0).max(0.0) / 3.6
        } else {
            0.0
        };
        let (ad_thr, ad_brk, ad_str) = self.ad.tick(
            self.tcm.ground_speed_kmh,
            self.throttle_pct / 100.0,
            self.brake_pct / 100.0,
            fused_lead,
            lead_spd,
            lane_off,
            0.0,
            lane_conf,
            ttc,
            self.radar.bsm_left,
            self.radar.bsm_right,
            dt,
        );
        // AD overrides apply back to vehicle (if engaged)
        let _ = (ad_thr, ad_brk, ad_str); // commands available for main.rs to use

        // V2X
        self.v2x.update(
            self.gps.latitude_deg,
            self.gps.longitude_deg,
            spd_ms,
            self.vehicle_heading(),
            self.imu.longitudinal_g * 9.81,
            self.imu.lateral_g * 9.81,
            self.abs.abs_system_active,
            self.elapsed,
            dt,
        );

        // Telematics
        self.telematics.update(
            self.gps.latitude_deg,
            self.gps.longitude_deg,
            self.tcm.ground_speed_kmh,
            self.ecm.engine_hours,
            self.ecm.active_dtcs.len(),
            self.elapsed,
            dt,
        );

        // VCM
        let abs_torque_limit_nm = if self.abs.abs_system_active || self.abs.tcs_system_active {
            260.0
        } else {
            1050.0
        };
        let ad_torque_limit_nm = if fused_ttc < 1.2 {
            0.0
        } else if fused_ttc < 2.0 {
            220.0
        } else if fused_ttc < 3.5 {
            520.0
        } else if fused_lead < 12.0 {
            680.0
        } else {
            1050.0
        };
        let vcm_frames = self.vcm.tick(
            self.elapsed,
            self.ecm.rpm,
            self.tcm.ground_speed_kmh,
            self.ecm.actual_torque_nm,
            self.hcm.hydraulic_power_kw,
            self.implement.pto_torque_demand(),
            self.bcm.total_load_amps,
            abs_torque_limit_nm,
            ad_torque_limit_nm,
            self.ecm.active_dtcs.len() as u32,
            &self.gateway.dispatched.clone(),
            dt,
        );
        for f in vcm_frames {
            self.gateway.transmit(f);
        }

        // ADAS/fusion/energy status frames on the powertrain network via VCM SA.
        let lead_distance = self.fusion.lead_dist_m.min(self.radar.closest_front_m);
        let critical_ttc = self.fusion.ttc_critical.min(self.radar.ttc_front);
        let in_path_hazard = self
            .fusion
            .objects
            .iter()
            .any(|o| o.in_path && o.ttc_s < 3.0);
        let speed_limit = self.camera.active_speed_limit.unwrap_or(0) as f64;
        let adas = j1939::Builder::adas1(
            self.elapsed,
            self.ad.lka_state == FeatureState::Active,
            self.ad.acc_state == FeatureState::Active,
            self.ad.aeb_state == FeatureState::Active,
            lead_distance,
            critical_ttc,
            speed_limit,
            j1939::addr::HEADWAY,
        );
        let fus = j1939::Builder::fus1(
            self.elapsed,
            self.fusion.objects.len(),
            critical_ttc,
            self.fusion
                .objects
                .iter()
                .map(|o| o.confidence)
                .fold(0.0, f64::max),
            in_path_hazard,
            j1939::addr::HEADWAY,
        );
        let engy = j1939::Builder::engy1(
            self.elapsed,
            self.bcm.battery_voltage,
            self.bcm.total_load_amps,
            self.bcm.total_load_amps * self.bcm.battery_voltage / 1000.0,
            self.hcm.hydraulic_power_kw,
            j1939::addr::HEADWAY,
        );
        self.gateway.transmit(adas);
        self.gateway.transmit(fus);
        self.gateway.transmit(engy);

        // 17. ── Active fault injection ────────────────────────────────────────
        if self.fault_active {
            self.apply_fault();
        }

        self.elapsed += dt;
        self.metrics
            .on_step(step_started.elapsed().as_secs_f64() * 1000.0);

        if (self.metrics.steps_completed % 120) == 0 {
            observability::log_structured(&StructuredEvent {
                timestamp: self.elapsed,
                level: "INFO",
                module: "sim.tick",
                correlation_id: self.metrics.steps_completed,
                event: "tick_metrics",
                details: format!(
                    "loop_duration_ms={:.3},steps_completed={},error_count={},replay_failures={}",
                    self.metrics.loop_duration_ms,
                    self.metrics.steps_completed,
                    self.metrics.error_count,
                    self.metrics.replay_failures
                ),
            });
        }
    }

    fn vehicle_heading(&self) -> f64 {
        self.gps.course_deg
    }
    fn vehicle_accel_ms2(&self) -> f64 {
        self.ecm.acceleration_ms2()
    }

    fn run_leak_physics(&mut self, dt: f64) {
        let hvac_duty = if self.bcm.hvac_on { 0.82 } else { 0.18 };
        let ac_high_bar = if self.bcm.hvac_on {
            (6.5 + (self.ecm.rpm / 2200.0).clamp(0.0, 1.4) * 9.5
                + (self.ecm.ambient_temp_c - 20.0).max(0.0) * 0.12)
                .clamp(4.0, 32.0)
        } else {
            5.5
        };

        self.leak_rig.set_runtime_pressure_state(
            "HYD_MAIN",
            self.hcm.system_pressure_bar,
            self.hcm.system_pressure_bar,
        );
        self.leak_rig.set_runtime_pressure_state(
            "ENG_OIL",
            (self.ecm.oil_pressure_kpa / 100.0).max(0.0),
            (self.ecm.oil_pressure_kpa / 100.0).max(0.0),
        );
        self.leak_rig
            .set_runtime_pressure_state("AC_HIGH", ac_high_bar, ac_high_bar);

        let hyd_rho = self.leak_rig.oil_density_for("HYD_MAIN");
        let eng_rho = self.leak_rig.oil_density_for("ENG_OIL");
        let ac_rho = self.leak_rig.oil_density_for("AC_HIGH");

        let inputs = [
            (
                "HYD_MAIN",
                CircuitInput {
                    pressure_bar: self.hcm.system_pressure_bar,
                    delta_p_bar: (self.hcm.system_pressure_bar - self.hcm.return_pres_bar).max(0.0),
                    temp_c: self.hcm.fluid_temp_c,
                    cycles_per_s: (self.hcm.total_flow_demand_lpm / 45.0).clamp(0.0, 5.0),
                    duty_01: (self.hcm.total_flow_demand_lpm / self.hcm.pump_flow_lpm.max(1.0))
                        .clamp(0.0, 1.0),
                    fluid_density_kg_m3: hyd_rho,
                },
            ),
            (
                "ENG_OIL",
                CircuitInput {
                    pressure_bar: (self.ecm.oil_pressure_kpa / 100.0).max(0.0),
                    delta_p_bar: (self.ecm.oil_pressure_kpa / 100.0).max(0.0),
                    temp_c: self.ecm.oil_temp_c,
                    cycles_per_s: (self.ecm.rpm / 60.0).clamp(0.0, 60.0),
                    duty_01: (self.ecm.active_throttle / 100.0).clamp(0.0, 1.0),
                    fluid_density_kg_m3: eng_rho,
                },
            ),
            (
                "AC_HIGH",
                CircuitInput {
                    pressure_bar: ac_high_bar,
                    delta_p_bar: (ac_high_bar - 2.0).max(0.0),
                    temp_c: self.bcm.cab_temp_c + 8.0,
                    cycles_per_s: (self.ecm.rpm / 120.0).clamp(0.0, 30.0),
                    duty_01: hvac_duty,
                    fluid_density_kg_m3: ac_rho,
                },
            ),
        ];

        let reports = self.leak_rig.step(&inputs, dt);
        self.leak_reports = reports.clone();

        for rep in &reports {
            let leaked_l = rep.leak_lpm * dt / 60.0;
            match rep.name.as_str() {
                "HYD_MAIN" => {
                    let pressure_drop_bar = leaked_l * 85.0;
                    self.hcm.system_pressure_bar =
                        (self.hcm.system_pressure_bar - pressure_drop_bar).max(0.0);
                    self.hcm.fluid_level_pct =
                        (self.hcm.fluid_level_pct - leaked_l / 90.0 * 100.0).clamp(0.0, 100.0);
                    if rep.alert >= LeakAlertLevel::Warning {
                        self.hcm.alarm_low_pressure = true;
                    }
                }
                "ENG_OIL" => {
                    let pressure_scale = (1.0 - (rep.leak_lpm / 12.0)).clamp(0.12, 1.0);
                    self.ecm.oil_pressure_kpa *= pressure_scale;
                    self.ecm.oil_level_pct =
                        (self.ecm.oil_level_pct - leaked_l / 28.0 * 100.0).clamp(0.0, 100.0);
                    if rep.alert >= LeakAlertLevel::Warning && self.ecm.rpm > 600.0 {
                        self.ecm.amber_lamp = true;
                    }
                    if rep.alert >= LeakAlertLevel::Critical && self.ecm.rpm > 600.0 {
                        self.ecm.red_lamp = true;
                    }
                }
                "AC_HIGH" => {
                    if self.bcm.hvac_on {
                        let cooling_loss = (rep.leak_lpm / 1.5).clamp(0.0, 1.0);
                        self.bcm.cab_temp_c += (0.22 + cooling_loss * 0.55) * dt;
                    }
                }
                _ => {}
            }
        }
    }

    fn build_radar_traffic(&self) -> Vec<SimTrafficObj> {
        self.implement_traffic()
    }

    fn implement_traffic(&self) -> Vec<SimTrafficObj> {
        use radar::RadarObjectClass;
        let t = self.elapsed;
        let lead_dist = 50.0 + (t * 0.1).sin() * 30.0;
        vec![
            SimTrafficObj {
                id: 0,
                distance_m: lead_dist.max(5.0),
                lateral_offset_m: 0.0,
                speed_ms: 10.0 + (t * 0.05).sin() * 5.0,
                rcs_dbsm: 12.0,
                object_type: RadarObjectClass::Car,
            },
            SimTrafficObj {
                id: 1,
                distance_m: 25.0 + (t * 0.07).sin() * 10.0,
                lateral_offset_m: 3.5,
                speed_ms: 8.0,
                rcs_dbsm: 8.0,
                object_type: RadarObjectClass::Car,
            },
            SimTrafficObj {
                id: 2,
                distance_m: 80.0,
                lateral_offset_m: -0.5,
                speed_ms: 15.0,
                rcs_dbsm: 22.0,
                object_type: RadarObjectClass::Truck,
            },
        ]
    }

    fn build_lidar_world(&self) -> Vec<WorldObstacle> {
        let t = self.elapsed;
        vec![
            WorldObstacle::new_vehicle(50.0 + (t * 0.1).sin() * 20.0, 0.0, 10.0),
            WorldObstacle::new_vehicle(25.0, 3.5, 8.0),
            WorldObstacle::new_barrier(30.0, -8.0, 10.0),
            WorldObstacle::new_pedestrian(12.0, 2.0),
        ]
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn apply_fault(&mut self) {
        match self.selected_fault {
            FaultType::HighCoolantTemp => {
                self.ecm.coolant_temp_c = 115.0;
            }
            FaultType::LowOilPressure => {
                self.ecm.oil_pressure_kpa = 40.0;
            }
            FaultType::LowFuelPressure => {
                self.ecm.fuel_pressure_kpa = 50.0;
            }
            FaultType::CriticalDefLevel => {
                self.ecm.def_level_pct = 1.0;
            }
            FaultType::HighDpfSoot => {
                self.ecm.dpf_soot_pct = 95.0;
            }
            FaultType::TransClutchOverheat => {
                self.tcm.clutch_temp_c = 250.0;
            }
            FaultType::TransNeutralFailure => { /* TCM cannot leave neutral */ }
            FaultType::LowBatteryVoltage => {
                self.bcm.battery_voltage = 10.8;
            }
            FaultType::AlternatorFault => {
                self.ecm.alternator_v = 11.0;
            }
            FaultType::HighHydTemp => {
                self.hcm.fluid_temp_c = 115.0;
            }
            FaultType::LowHydLevel => {
                self.hcm.fluid_level_pct = 10.0;
            }
            FaultType::HydFilterBypass => {
                self.hcm.filter_dp_bar = 8.0;
            }
            FaultType::BusOffInjection => {
                self.gateway.inject_bus_off = true;
                self.fault_active = false;
            }
            FaultType::KillEcm => {
                self.gateway.kill_node(j1939::addr::ECM_1);
                self.fault_active = false;
            }
            FaultType::KillTcm => {
                self.gateway.kill_node(j1939::addr::TRANSMISSION);
                self.fault_active = false;
            }
            FaultType::None => {}
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Activate the selected fault
    pub fn inject_fault(&mut self) {
        self.fault_active = true;
    }

    /// Clear all faults and restore modules
    pub fn clear_faults(&mut self) {
        self.fault_active = false;
        self.ecm.clear_dtcs();
        self.gateway.revive_node(j1939::addr::ECM_1);
        self.gateway.revive_node(j1939::addr::TRANSMISSION);
        self.leak_rig.reset();
        self.leak_reports.clear();
    }

    /// Advance ignition key one position
    pub fn key_advance(&mut self) {
        self.boot.key_advance();
    }

    /// Turn key fully off — powers down all ECUs
    pub fn key_off(&mut self) {
        self.boot.key_off();
        self.throttle_pct = 0.0;
        self.brake_pct = 0.0;
    }

    /// Hard reset — rebuilds entire bench from scratch
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn apply_leak_manual_params(
        &mut self,
        circuit_name: &str,
        params: ManualCircuitParams,
    ) -> bool {
        self.leak_rig.apply_manual_params(circuit_name, params)
    }

    pub fn add_custom_leak_circuit(&mut self, circuit: LeakCircuit) {
        self.leak_rig.add_circuit(circuit);
    }

    pub fn predict_leak_scenarios(&self, horizon_s: f64, dt: f64) -> Vec<ScenarioPrediction> {
        self.leak_rig.predict_scenarios(horizon_s, dt)
    }

    pub fn export_leak_report_json<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> std::io::Result<()> {
        self.leak_rig.export_last_results_json(path)
    }

    pub fn export_leak_report_csv<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> std::io::Result<()> {
        self.leak_rig.export_last_results_csv(path)
    }

    pub fn export_leak_predictions_json<P: AsRef<std::path::Path>>(
        &self,
        path: P,
        predictions: &[ScenarioPrediction],
    ) -> std::io::Result<()> {
        self.leak_rig.export_predictions_json(path, predictions)
    }

    pub fn export_leak_predictions_csv<P: AsRef<std::path::Path>>(
        &self,
        path: P,
        predictions: &[ScenarioPrediction],
    ) -> std::io::Result<()> {
        self.leak_rig.export_predictions_csv(path, predictions)
    }

    pub fn monte_carlo_leak_predictions(
        &self,
        runs: usize,
        horizon_s: f64,
        dt: f64,
        variation_pct: f64,
    ) -> Vec<ScenarioPrediction> {
        self.leak_rig
            .run_monte_carlo(runs, horizon_s, dt, variation_pct)
    }

    pub fn export_can_snapshot_json<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> std::io::Result<()> {
        self.can_net.export_snapshot_json(path)
    }

    pub fn export_can_snapshot_csv<P: AsRef<std::path::Path>>(
        &self,
        path: P,
    ) -> std::io::Result<()> {
        self.can_net.export_snapshot_csv(path)
    }

    // ─ Convenience accessors ─────────────────────────────────────────────────
    pub fn ignition(&self) -> IgnitionState {
        self.boot.ignition
    }
    pub fn engine_running(&self) -> bool {
        self.ecm.is_running()
    }
    pub fn all_ecus_online(&self) -> bool {
        self.boot.all_critical_online
    }
}
