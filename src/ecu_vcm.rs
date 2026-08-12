//! Vehicle Control Module (VCM) / Gateway ECU — SA 0x14 (headway controller position).
//!
//! The VCM is the central coordinator in modern heavy machinery:
//!   • Acts as CAN gateway between HS-CAN powertrain, MS-CAN body, and ISOBUS
//!   • Coordinates ADAS features across domains
//!   • Implements vehicle-level safety functions (ISO 26262 ASIL-B)
//!   • Manages power distribution and load shedding
//!   • Provides torque coordination between ECM and braking systems
//!   • Hosts the proprietary J1939 working set master for ISOBUS
//!   • Manages creep/crawl protection, rollaway prevention

use crate::j1939::{addr, pgn, J1939Frame};

pub const VCM_SA: u8 = addr::HEADWAY; // 0x20

// ── Vehicle operating mode ────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VehicleMode {
    Parked,          // engine off, no motion
    Idle,            // engine running, no drive torque
    FieldWork,       // operating in field (low speed, implements active)
    RoadTransport,   // high speed, no implement engagement
    TransportBraked, // decelerating on road
    Emergency,       // emergency state (e-stop or fault)
    Limp,            // degraded mode (lost critical ECU communication)
}

impl std::fmt::Display for VehicleMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            VehicleMode::Parked => write!(f, "PARKED        "),
            VehicleMode::Idle => write!(f, "IDLE          "),
            VehicleMode::FieldWork => write!(f, "FIELD WORK    "),
            VehicleMode::RoadTransport => write!(f, "ROAD TRANSPORT"),
            VehicleMode::TransportBraked => write!(f, "BRAKING       "),
            VehicleMode::Emergency => write!(f, "EMERGENCY!    "),
            VehicleMode::Limp => write!(f, "LIMP HOME     "),
        }
    }
}

// ── Power distribution ────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct PowerDistribution {
    pub total_load_kw: f64,
    pub engine_available_kw: f64,
    pub drivetrain_demand_kw: f64,
    pub hydraulics_demand_kw: f64,
    pub pto_demand_kw: f64,
    pub electrical_demand_kw: f64,
    pub overload: bool,
    pub load_shed_active: bool,
    pub load_shed_reason: Option<&'static str>,
}

impl Default for PowerDistribution {
    fn default() -> Self {
        Self::new()
    }
}

impl PowerDistribution {
    pub fn new() -> Self {
        PowerDistribution {
            total_load_kw: 0.0,
            engine_available_kw: 220.0,
            drivetrain_demand_kw: 0.0,
            hydraulics_demand_kw: 0.0,
            pto_demand_kw: 0.0,
            electrical_demand_kw: 3.0,
            overload: false,
            load_shed_active: false,
            load_shed_reason: None,
        }
    }
}

// ── Torque coordination ───────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct TorqueCoordination {
    /// Max torque allowed from ECM (limited by VCM for safety)
    pub max_engine_torque_nm: f64,
    /// Torque requested by TCM (for shift quality)
    pub tcm_torque_request_nm: f64,
    /// Torque requested by ABS/TCS (for slip control)
    pub abs_torque_limit_nm: f64,
    /// Torque requested by AD system
    pub ad_torque_request_nm: f64,
    /// Final commanded torque (minimum of all limits)
    pub commanded_torque_nm: f64,
    /// Reason for any torque reduction
    pub limiting_source: &'static str,
}

impl Default for TorqueCoordination {
    fn default() -> Self {
        Self::new()
    }
}

impl TorqueCoordination {
    pub fn new() -> Self {
        TorqueCoordination {
            max_engine_torque_nm: 1050.0,
            tcm_torque_request_nm: 0.0,
            abs_torque_limit_nm: 1050.0,
            ad_torque_request_nm: 1050.0,
            commanded_torque_nm: 1050.0,
            limiting_source: "none",
        }
    }

    pub fn compute_final(&mut self) {
        let limit = [
            (self.max_engine_torque_nm, "VCM-max"),
            (self.abs_torque_limit_nm, "ABS/TCS"),
            (self.ad_torque_request_nm, "AD-ctrl"),
        ];
        let (min_torque, reason) = limit
            .iter()
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .copied()
            .unwrap_or((1050.0, "none"));
        self.commanded_torque_nm = min_torque;
        self.limiting_source = reason;
    }
}

// ── Communication matrix health ───────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct CommHealth {
    pub ecm_alive: bool,
    pub tcm_alive: bool,
    pub abs_alive: bool,
    pub bcm_alive: bool,
    pub icm_alive: bool,
    pub hcm_alive: bool,
    pub ecm_timeout_count: u32,
    pub tcm_timeout_count: u32,
    pub abs_timeout_count: u32,
    pub last_ecm_ts: f64,
    pub last_tcm_ts: f64,
    pub last_abs_ts: f64,
    pub last_hcm_ts: f64,
}

impl Default for CommHealth {
    fn default() -> Self {
        Self::new()
    }
}

impl CommHealth {
    pub fn new() -> Self {
        CommHealth {
            ecm_alive: false,
            tcm_alive: false,
            abs_alive: false,
            bcm_alive: false,
            icm_alive: false,
            hcm_alive: false,
            ecm_timeout_count: 0,
            tcm_timeout_count: 0,
            abs_timeout_count: 0,
            last_ecm_ts: 0.0,
            last_tcm_ts: 0.0,
            last_abs_ts: 0.0,
            last_hcm_ts: 0.0,
        }
    }

    /// Check for message timeouts — real VCM monitors each critical ECU
    pub fn check_timeouts(&mut self, elapsed: f64) {
        const ECM_TIMEOUT: f64 = 0.050; // EEC1 at 10ms → 50ms timeout
        const TCM_TIMEOUT: f64 = 0.100; // ETC1 at 20ms → 100ms timeout
        const ABS_TIMEOUT: f64 = 0.200; // EBC1 at 20ms → 200ms timeout
        const HCM_TIMEOUT: f64 = 0.500; // HITCH at 100ms → 500ms timeout

        let was_ecm = self.ecm_alive;
        self.ecm_alive = (elapsed - self.last_ecm_ts) < ECM_TIMEOUT;
        if was_ecm && !self.ecm_alive {
            self.ecm_timeout_count += 1;
        }

        let was_tcm = self.tcm_alive;
        self.tcm_alive = (elapsed - self.last_tcm_ts) < TCM_TIMEOUT;
        if was_tcm && !self.tcm_alive {
            self.tcm_timeout_count += 1;
        }

        let was_abs = self.abs_alive;
        self.abs_alive = (elapsed - self.last_abs_ts) < ABS_TIMEOUT;
        if was_abs && !self.abs_alive {
            self.abs_timeout_count += 1;
        }

        self.hcm_alive = (elapsed - self.last_hcm_ts) < HCM_TIMEOUT;
    }

    pub fn update_from_frame(&mut self, frame: &J1939Frame) {
        let ts = frame.timestamp;
        match frame.sa {
            s if s == addr::ECM_1 => self.last_ecm_ts = ts,
            s if s == addr::TRANSMISSION => self.last_tcm_ts = ts,
            s if s == addr::BRAKES => self.last_abs_ts = ts,
            s if s == addr::HITCH => self.last_hcm_ts = ts,
            _ => {}
        }
    }
}

// ── VCM ──────────────────────────────────────────────────────────────────────
pub struct EcuVcm {
    pub sa: u8,
    pub mode: VehicleMode,
    pub power: PowerDistribution,
    pub torque_coord: TorqueCoordination,
    pub comm_health: CommHealth,

    // ─ Safety functions ───────────────────────────────────────────────────────
    pub rollaway_prevention: bool,  // prevents motion without driver
    pub overspeed_protection: bool, // limits speed to rated maximum
    pub pto_interlock: bool,        // prevents unsafe PTO engagement
    pub max_speed_kmh: f64,         // speed limit from VCM (e.g. 50 km/h road limit)
    pub speed_limited: bool,

    // ─ Fuel-cut coordination ──────────────────────────────────────────────────
    pub fuel_cut_active: bool,
    pub fuel_cut_reason: Option<&'static str>,

    // ─ Diagnostic aggregation ────────────────────────────────────────────────
    pub total_active_dtcs: u32,
    pub critical_fault: bool,

    // ─ J1939 TX timers ───────────────────────────────────────────────────────
    t_status: f64,
    t_ws_mst: f64,
    t_dm13: f64,
}

impl Default for EcuVcm {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuVcm {
    pub fn new() -> Self {
        EcuVcm {
            sa: VCM_SA,
            mode: VehicleMode::Parked,
            power: PowerDistribution::new(),
            torque_coord: TorqueCoordination::new(),
            comm_health: CommHealth::new(),
            rollaway_prevention: true,
            overspeed_protection: true,
            pto_interlock: true,
            max_speed_kmh: 50.0,
            speed_limited: false,
            fuel_cut_active: false,
            fuel_cut_reason: None,
            total_active_dtcs: 0,
            critical_fault: false,
            t_status: 0.0,
            t_ws_mst: 0.0,
            t_dm13: 0.0,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn tick(
        &mut self,
        elapsed: f64,
        engine_rpm: f64,
        ground_speed_kmh: f64,
        engine_torque_nm: f64,
        hydraulic_power_kw: f64,
        pto_torque_nm: f64,
        bcm_load_a: f64,
        abs_torque_limit: f64,
        ad_torque_limit: f64,
        dtc_count: u32,
        received_frames: &[J1939Frame],
        dt: f64,
    ) -> Vec<J1939Frame> {
        // ─ Update comms health ────────────────────────────────────────────────
        for f in received_frames {
            self.comm_health.update_from_frame(f);
        }
        self.comm_health.check_timeouts(elapsed);

        // ─ Determine vehicle operating mode ───────────────────────────────────
        self.mode = if engine_rpm < 400.0 {
            VehicleMode::Parked
        } else if ground_speed_kmh > 15.0 {
            VehicleMode::RoadTransport
        } else if ground_speed_kmh > 0.5 {
            VehicleMode::FieldWork
        } else {
            VehicleMode::Idle
        };

        // ─ Limp home if critical ECU lost ────────────────────────────────────
        if engine_rpm > 400.0 && !self.comm_health.ecm_alive {
            self.mode = VehicleMode::Limp;
        }

        // ─ Power distribution ─────────────────────────────────────────────────
        self.power.drivetrain_demand_kw =
            engine_torque_nm * engine_rpm * std::f64::consts::PI / 30000.0;
        self.power.hydraulics_demand_kw = hydraulic_power_kw;
        self.power.pto_demand_kw = pto_torque_nm * engine_rpm * std::f64::consts::PI / 30000.0;
        self.power.electrical_demand_kw = bcm_load_a * 14.0 / 1000.0;
        self.power.total_load_kw = self.power.drivetrain_demand_kw
            + self.power.hydraulics_demand_kw
            + self.power.pto_demand_kw
            + self.power.electrical_demand_kw;
        self.power.overload = self.power.total_load_kw > 220.0 * 1.05;

        if self.power.overload {
            self.power.load_shed_active = true;
            self.power.load_shed_reason = Some("Total load > 220 kW rated");
            // Priority: drivetrain > hydraulics > PTO > electrical
            self.power.pto_demand_kw =
                (220.0 * 0.9 - self.power.drivetrain_demand_kw - self.power.hydraulics_demand_kw)
                    .max(0.0);
        } else {
            self.power.load_shed_active = false;
            self.power.load_shed_reason = None;
        }

        // ─ Torque coordination ────────────────────────────────────────────────
        self.torque_coord.abs_torque_limit_nm = abs_torque_limit;
        self.torque_coord.ad_torque_request_nm = ad_torque_limit;
        self.torque_coord.max_engine_torque_nm = if self.mode == VehicleMode::Limp {
            1050.0 * 0.5
        } else {
            1050.0
        };
        self.torque_coord.compute_final();

        // ─ Speed limiting ─────────────────────────────────────────────────────
        self.speed_limited = ground_speed_kmh > self.max_speed_kmh;
        if self.speed_limited {
            self.fuel_cut_active = true;
            self.fuel_cut_reason = Some("Speed limit exceeded");
        } else {
            self.fuel_cut_active = false;
            self.fuel_cut_reason = None;
        }

        // ─ Fault aggregation ─────────────────────────────────────────────────
        self.total_active_dtcs = dtc_count;
        self.critical_fault = !self.comm_health.ecm_alive && engine_rpm > 400.0;

        // ─ J1939 periodic output ──────────────────────────────────────────────
        self.t_status += dt;
        self.t_ws_mst += dt;
        self.t_dm13 += dt;
        let mut frames: Vec<J1939Frame> = Vec::new();

        if self.t_status >= 0.100 {
            self.t_status = 0.0;
            frames.push(self.build_vcm_status(elapsed));
        }
        if self.t_ws_mst >= 0.100 {
            self.t_ws_mst = 0.0;
            frames.push(self.build_ws_master(elapsed));
        }
        if self.t_dm13 >= 5.000 {
            self.t_dm13 = 0.0;
            frames.push(self.build_dm13(elapsed));
        }
        frames
    }

    // ─ Frame builders ─────────────────────────────────────────────────────────

    /// VCM proprietary status frame (Prop-A, 100ms)
    fn build_vcm_status(&self, ts: f64) -> J1939Frame {
        let mut data = [0u8; 8];
        data[0] = self.mode as u8;
        data[1] = (self.power.total_load_kw / 220.0 * 250.0) as u8;
        data[2] = self.comm_health.ecm_alive as u8
            | ((self.comm_health.tcm_alive as u8) << 1)
            | ((self.comm_health.abs_alive as u8) << 2)
            | ((self.comm_health.hcm_alive as u8) << 3);
        data[3] = if self.critical_fault { 0x01 } else { 0x00 };
        data[4] = (self.total_active_dtcs.min(255)) as u8;
        data[5] = (self.torque_coord.commanded_torque_nm / 1050.0 * 250.0) as u8;
        data[6] = self.fuel_cut_active as u8;
        data[7] = self.power.load_shed_active as u8;
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(7, pgn::PROP_A, self.sa, 0xFF),
            &data,
        )
    }

    /// ISOBUS Working Set Master announcement (100ms)
    fn build_ws_master(&self, ts: f64) -> J1939Frame {
        let data = [0x00u8; 8]; // member count = 0 (VCM is the only member)
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(7, pgn::WS_MST, self.sa, 0xFF),
            &data,
        )
    }

    /// DM13 — Stop/Start Broadcast (NM coordination)
    fn build_dm13(&self, ts: f64) -> J1939Frame {
        let mut data = [0xFFu8; 8];
        // All bits: 0b11 = don't care, 0b01 = stop broadcast (sleep)
        data[0] = 0xFF;
        data[1] = 0xFF;
        J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(7, pgn::DM13, self.sa, addr::BROADCAST),
            &data,
        )
    }
}
