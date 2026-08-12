//! V2X — Vehicle-to-Everything communication (DSRC 5.9GHz + C-V2X LTE-V).
//! Telematics — Fleet management, remote diagnostics, OTA update.

use std::collections::VecDeque;
use std::f64::consts::PI;

// ═════════════════════════════════════════════════════════════════════════════
// V2X Module
// ═════════════════════════════════════════════════════════════════════════════

/// SAE J2735 Basic Safety Message
#[derive(Debug, Clone)]
pub struct BasicSafetyMessage {
    pub msg_count: u8,
    pub vehicle_id: u32,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub elevation_m: f64,
    pub speed_ms: f64,
    pub heading_deg: f64,
    pub accel_long: f64, // m/s²
    pub accel_lat: f64,
    pub accel_vert: f64,
    pub yaw_rate: f64,
    pub brake_status: BrakeStatus,
    pub size_length_m: f64,
    pub size_width_m: f64,
    pub timestamp_ms: u64,
}

#[derive(Debug, Clone, Copy)]
pub struct BrakeStatus {
    pub brake_applied: bool,
    pub traction_control: bool,
    pub abs_active: bool,
    pub stability: bool,
    pub aux_brakes: bool,
}

/// SAE J2735 Signal Phase and Timing (SPaT) from infrastructure
#[derive(Debug, Clone)]
pub struct SignalPhaseAndTiming {
    pub intersection_id: u32,
    pub movement_id: u8,
    pub phase_state: TrafficPhase,
    pub time_to_change_s: f64,
    pub distance_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrafficPhase {
    Green,
    Yellow,
    Red,
    Unknown,
}

impl std::fmt::Display for TrafficPhase {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrafficPhase::Green => "GREEN",
            TrafficPhase::Yellow => "YELLOW",
            TrafficPhase::Red => "RED  ",
            TrafficPhase::Unknown => "UNKNWN",
        }
        .fmt(f)
    }
}

/// Work Zone Alert (roadwork ahead)
#[derive(Debug, Clone)]
pub struct WorkZoneAlert {
    pub zone_id: u32,
    pub distance_m: f64,
    pub speed_limit_kmh: f64,
    pub description: String,
}

/// Emergency Vehicle Alert
#[derive(Debug, Clone)]
pub struct EmergencyAlert {
    pub vehicle_id: u32,
    pub vehicle_type: EmergencyType,
    pub distance_m: f64,
    pub bearing_deg: f64,
    pub approaching: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum EmergencyType {
    Ambulance,
    FireTruck,
    Police,
}

pub struct V2xModule {
    // ─ Own BSM ───────────────────────────────────────────────────────────────
    pub bsm_tx_count: u64,
    pub bsm_tx_rate_hz: f64, // 10 Hz per J2735
    pub dsrc_active: bool,   // DSRC 5.9GHz
    pub cv2x_active: bool,   // Cellular V2X

    // ─ Received messages ─────────────────────────────────────────────────────
    pub nearby_vehicles: Vec<BasicSafetyMessage>,
    pub spat_messages: Vec<SignalPhaseAndTiming>,
    pub work_zones: Vec<WorkZoneAlert>,
    pub emergency_alerts: Vec<EmergencyAlert>,

    // ─ Statistics ────────────────────────────────────────────────────────────
    pub range_m: f64, // effective communication range
    pub packet_loss_pct: f64,
    pub latency_ms: f64,
    pub rx_count: u64,

    // ─ Alerts ────────────────────────────────────────────────────────────────
    pub forward_collision_alert: bool,
    pub intersection_alert: bool,
    pub emergency_vehicle_alert: bool,
    pub road_hazard_alert: bool,

    // Internal
    bsm_timer: f64,
    sim_timer: f64,
    msg_count: u8,
    noise_t: f64,
}

impl Default for V2xModule {
    fn default() -> Self {
        Self::new()
    }
}

impl V2xModule {
    pub fn new() -> Self {
        V2xModule {
            bsm_tx_count: 0,
            bsm_tx_rate_hz: 10.0,
            dsrc_active: true,
            cv2x_active: true,
            nearby_vehicles: Vec::new(),
            spat_messages: Vec::new(),
            work_zones: Vec::new(),
            emergency_alerts: Vec::new(),
            range_m: 300.0,
            packet_loss_pct: 2.0,
            latency_ms: 15.0,
            rx_count: 0,
            forward_collision_alert: false,
            intersection_alert: false,
            emergency_vehicle_alert: false,
            road_hazard_alert: false,
            bsm_timer: 0.0,
            sim_timer: 0.0,
            msg_count: 0,
            noise_t: 0.0,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        lat: f64,
        lon: f64,
        speed_ms: f64,
        heading: f64,
        _accel_lon: f64,
        _accel_lat: f64,
        _abs_active: bool,
        elapsed: f64,
        dt: f64,
    ) {
        self.noise_t += dt;
        self.bsm_timer += dt;
        self.sim_timer += dt;
        let n = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5 % 1.0 - 0.5) * 2.0;
        let nt = self.noise_t;

        // ─ Transmit BSM at 10 Hz ──────────────────────────────────────────────
        if self.bsm_timer >= 1.0 / self.bsm_tx_rate_hz {
            self.bsm_timer = 0.0;
            self.bsm_tx_count += 1;
            self.msg_count = self.msg_count.wrapping_add(1);
        }

        // ─ Simulate receiving BSMs from nearby vehicles ───────────────────────
        self.nearby_vehicles.clear();
        // 3-5 vehicles within range
        let n_vehicles = 3 + (n(nt * 0.1).abs() * 2.0) as usize;
        for i in 0..n_vehicles {
            let dist = 30.0 + i as f64 * 40.0 + n(nt + i as f64) * 15.0;
            let az = (i as f64 * 60.0 + n(nt * 0.3 + i as f64) * 10.0) % 360.0;
            self.nearby_vehicles.push(BasicSafetyMessage {
                msg_count: self.msg_count.wrapping_add(i as u8),
                vehicle_id: 0xA000 + i as u32,
                lat_deg: lat + (az.to_radians().cos() * dist / 111320.0),
                lon_deg: lon + (az.to_radians().sin() * dist / 111320.0),
                elevation_m: 0.0,
                speed_ms: 15.0 + n(nt + i as f64 * 2.0) * 5.0,
                heading_deg: (heading + n(nt + i as f64) * 15.0 + 360.0) % 360.0,
                accel_long: n(nt + i as f64 * 3.0) * 1.0,
                accel_lat: n(nt + i as f64 * 4.0) * 0.5,
                accel_vert: 0.0,
                yaw_rate: n(nt + i as f64 * 5.0) * 0.1,
                brake_status: BrakeStatus {
                    brake_applied: false,
                    traction_control: false,
                    abs_active: false,
                    stability: false,
                    aux_brakes: false,
                },
                size_length_m: 4.8,
                size_width_m: 1.9,
                timestamp_ms: (elapsed * 1000.0) as u64,
            });
            self.rx_count += 1;
        }

        // ─ Simulate SPaT messages from intersections ──────────────────────────
        self.spat_messages.clear();
        let spat_dist = 150.0 + (nt * 0.02).sin() * 50.0;
        let phase_cycle = elapsed % 90.0;
        let (phase, ttc) = if phase_cycle < 45.0 {
            (TrafficPhase::Green, 45.0 - phase_cycle)
        } else if phase_cycle < 50.0 {
            (TrafficPhase::Yellow, 50.0 - phase_cycle)
        } else {
            (TrafficPhase::Red, 90.0 - phase_cycle)
        };
        self.spat_messages.push(SignalPhaseAndTiming {
            intersection_id: 0x1234,
            movement_id: 1,
            phase_state: phase,
            time_to_change_s: ttc,
            distance_m: spat_dist,
        });

        // ─ Work zone simulation ───────────────────────────────────────────────
        if self.work_zones.is_empty() {
            self.work_zones.push(WorkZoneAlert {
                zone_id: 0x5678,
                distance_m: 800.0,
                speed_limit_kmh: 40.0,
                description: "Road work — 800m ahead".into(),
            });
        }
        // Work zone gets closer as we drive
        for wz in &mut self.work_zones {
            wz.distance_m = (wz.distance_m - speed_ms * dt).max(50.0);
        }

        // ─ Emergency vehicle simulation (periodic) ───────────────────────────
        self.emergency_alerts.clear();
        if (elapsed % 30.0) < 5.0 {
            self.emergency_alerts.push(EmergencyAlert {
                vehicle_id: 0xE001,
                vehicle_type: EmergencyType::Ambulance,
                distance_m: 200.0 - speed_ms * (elapsed % 5.0),
                bearing_deg: 180.0,
                approaching: true,
            });
        }

        // ─ Generate alerts ────────────────────────────────────────────────────
        self.forward_collision_alert = self.nearby_vehicles.iter().any(|v| {
            v.speed_ms < speed_ms * 0.5 && {
                let dx = v.lat_deg - lat;
                let dy = v.lon_deg - lon;
                let dist = (dx * dx + dy * dy).sqrt() * 111320.0;
                dist < 30.0
            }
        });
        self.intersection_alert = self
            .spat_messages
            .iter()
            .any(|s| s.distance_m < 80.0 && s.phase_state == TrafficPhase::Red);
        self.emergency_vehicle_alert = !self.emergency_alerts.is_empty();
        self.road_hazard_alert = self.work_zones.iter().any(|w| w.distance_m < 100.0);

        // ─ Link quality simulation ────────────────────────────────────────────
        self.packet_loss_pct = (5.0 * n(nt * 0.5).abs()).min(20.0);
        self.latency_ms = 12.0 + n(nt * 0.8).abs() * 8.0;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Telematics Module (Fleet Management / Remote Diagnostics)
// ═════════════════════════════════════════════════════════════════════════════

#[derive(Debug, Clone)]
pub struct TelematicsEvent {
    pub timestamp: f64,
    pub event_type: TelEventType,
    pub description: String,
    pub sent: bool,
    pub ack: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum TelEventType {
    DtcAlert,
    PeriodicReport,
    GeofenceAlert,
    SpeedAlert,
    EngineOn,
    EngineOff,
    MaintenanceAlert,
    CrashDetect,
    OtaUpdateAvail,
    OtaUpdateComplete,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ConnectState {
    Offline,
    Connecting,
    Connected4G,
    Connected5G,
}

impl std::fmt::Display for ConnectState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConnectState::Offline => write!(f, "OFFLINE   "),
            ConnectState::Connecting => write!(f, "CONNECTING"),
            ConnectState::Connected4G => write!(f, "4G LTE    "),
            ConnectState::Connected5G => write!(f, "5G NR     "),
        }
    }
}

pub struct TelematicsModule {
    // ─ Connectivity ──────────────────────────────────────────────────────────
    pub connect_state: ConnectState,
    pub signal_bars: u8, // 0-5
    pub rsrp_dbm: f64,   // Reference Signal Received Power
    pub rsrq_db: f64,    // Reference Signal Received Quality
    pub ping_ms: f64,
    pub download_kbps: f64,
    pub upload_kbps: f64,
    pub data_used_mb: f64,

    // ─ Fleet server ──────────────────────────────────────────────────────────
    pub fleet_server: String,
    pub vehicle_id: String,
    pub fleet_unit_id: u32,
    pub last_sync_s: f64,
    pub report_interval_s: f64,

    // ─ Remote commands received ───────────────────────────────────────────────
    pub pending_commands: Vec<RemoteCommand>,

    // ─ OTA Update ────────────────────────────────────────────────────────────
    pub ota_available: bool,
    pub ota_version: String,
    pub ota_size_mb: f64,
    pub ota_progress_pct: f64,
    pub ota_downloading: bool,

    // ─ Geofencing ────────────────────────────────────────────────────────────
    pub geofences: Vec<Geofence>,
    pub inside_geofence: Option<u32>,

    // ─ Event log ─────────────────────────────────────────────────────────────
    pub events: VecDeque<TelematicsEvent>,
    pub total_events_sent: u64,

    // ─ Timers ────────────────────────────────────────────────────────────────
    report_timer: f64,
    connect_timer: f64,
    noise_t: f64,
}

#[derive(Debug, Clone)]
pub struct RemoteCommand {
    pub id: u32,
    pub cmd_type: RemoteCmdType,
    pub params: String,
    pub received_ts: f64,
    pub executed: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum RemoteCmdType {
    RequestDtcs,
    ClearDtcs,
    UpdateFirmware,
    SetSpeedLimit,
    Immobilise,
    Unlock,
    RequestLocation,
}

impl std::fmt::Display for RemoteCmdType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RemoteCmdType::RequestDtcs => write!(f, "REQUEST_DTC  "),
            RemoteCmdType::ClearDtcs => write!(f, "CLEAR_DTC    "),
            RemoteCmdType::UpdateFirmware => write!(f, "OTA_UPDATE   "),
            RemoteCmdType::SetSpeedLimit => write!(f, "SET_SPD_LIM  "),
            RemoteCmdType::Immobilise => write!(f, "IMMOBILISE   "),
            RemoteCmdType::Unlock => write!(f, "UNLOCK       "),
            RemoteCmdType::RequestLocation => write!(f, "REQ_LOCATION "),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Geofence {
    pub id: u32,
    pub name: String,
    pub center_lat: f64,
    pub center_lon: f64,
    pub radius_m: f64,
    pub alert_type: GeofenceAlert,
}

#[derive(Debug, Clone, Copy)]
pub enum GeofenceAlert {
    Entry,
    Exit,
    Both,
}

impl Default for TelematicsModule {
    fn default() -> Self {
        Self::new()
    }
}

impl TelematicsModule {
    pub fn new() -> Self {
        let geofences = vec![
            Geofence {
                id: 1,
                name: "HQ Depot".into(),
                center_lat: -22.810,
                center_lon: -47.062,
                radius_m: 500.0,
                alert_type: GeofenceAlert::Both,
            },
            Geofence {
                id: 2,
                name: "Field Zone A".into(),
                center_lat: -22.850,
                center_lon: -47.090,
                radius_m: 1000.0,
                alert_type: GeofenceAlert::Entry,
            },
        ];
        TelematicsModule {
            connect_state: ConnectState::Connected4G,
            signal_bars: 4,
            rsrp_dbm: -85.0,
            rsrq_db: -9.0,
            ping_ms: 28.0,
            download_kbps: 15000.0,
            upload_kbps: 5000.0,
            data_used_mb: 0.0,
            fleet_server: "fleet.example.com:8443".into(),
            vehicle_id: "CAT-320-001".into(),
            fleet_unit_id: 1001,
            last_sync_s: 0.0,
            report_interval_s: 30.0,
            pending_commands: Vec::new(),
            ota_available: false,
            ota_version: String::new(),
            ota_size_mb: 0.0,
            ota_progress_pct: 0.0,
            ota_downloading: false,
            geofences,
            inside_geofence: None,
            events: VecDeque::new(),
            total_events_sent: 0,
            report_timer: 0.0,
            connect_timer: 0.0,
            noise_t: 0.0,
        }
    }

    #[allow(clippy::too_many_arguments)]
    pub fn update(
        &mut self,
        lat: f64,
        lon: f64,
        speed_kmh: f64,
        engine_hours: f64,
        active_dtcs: usize,
        elapsed: f64,
        dt: f64,
    ) {
        self.noise_t += dt;
        self.report_timer += dt;
        self.connect_timer += dt;
        let n = |s: f64| ((s * 127.1 + 311.7).sin() * 43758.5 % 1.0 - 0.5) * 2.0;
        let nt = self.noise_t;

        // ─ Signal quality simulation ──────────────────────────────────────────
        self.rsrp_dbm = -82.0 + n(nt * 0.3) * 8.0;
        self.rsrq_db = -9.0 + n(nt * 0.5) * 3.0;
        self.signal_bars = match self.rsrp_dbm as i32 {
            p if p > -70 => 5,
            p if p > -80 => 4,
            p if p > -90 => 3,
            p if p > -100 => 2,
            _ => 1,
        };
        self.ping_ms = 25.0 + n(nt * 0.7).abs() * 20.0;
        self.connect_state = if self.rsrp_dbm > -105.0 {
            if (nt * 0.01).sin() > 0.0 {
                ConnectState::Connected5G
            } else {
                ConnectState::Connected4G
            }
        } else {
            ConnectState::Connecting
        };

        // ─ Periodic report ────────────────────────────────────────────────────
        if self.report_timer >= self.report_interval_s {
            self.report_timer = 0.0;
            self.last_sync_s = elapsed;
            self.data_used_mb += 0.05 + speed_kmh * 0.0001;
            let ev = TelematicsEvent {
                timestamp: elapsed,
                sent: true,
                ack: true,
                event_type: TelEventType::PeriodicReport,
                description: format!(
                    "Pos:{:.4},{:.4} Spd:{:.0}km/h Hrs:{:.1}h DTCs:{}",
                    lat, lon, speed_kmh, engine_hours, active_dtcs
                ),
            };
            self.push_event(ev);
            self.total_events_sent += 1;
        }

        // ─ DTC alert ──────────────────────────────────────────────────────────
        if active_dtcs > 0
            && self
                .events
                .iter()
                .all(|e| !matches!(e.event_type, TelEventType::DtcAlert))
        {
            let ev = TelematicsEvent {
                timestamp: elapsed,
                sent: true,
                ack: false,
                event_type: TelEventType::DtcAlert,
                description: format!("{} active DTC(s) — ECM", active_dtcs),
            };
            self.push_event(ev);
        }

        // ─ Speed alert ────────────────────────────────────────────────────────
        if speed_kmh > 80.0
            && self
                .events
                .iter()
                .rev()
                .take(5)
                .all(|e| !matches!(e.event_type, TelEventType::SpeedAlert))
        {
            let ev = TelematicsEvent {
                timestamp: elapsed,
                sent: true,
                ack: true,
                event_type: TelEventType::SpeedAlert,
                description: format!("Speed limit exceeded: {:.0} km/h", speed_kmh),
            };
            self.push_event(ev);
        }

        // ─ Simulate incoming remote commands ─────────────────────────────────
        if self.connect_timer > 45.0 && self.pending_commands.is_empty() {
            self.connect_timer = 0.0;
            self.pending_commands.push(RemoteCommand {
                id: (elapsed as u32),
                cmd_type: RemoteCmdType::RequestDtcs,
                params: "{}".into(),
                received_ts: elapsed,
                executed: false,
            });
        }

        // ─ Geofence check ─────────────────────────────────────────────────────
        self.inside_geofence = None;
        for gf in &self.geofences {
            let dx = (lat - gf.center_lat) * 111320.0;
            let dy = (lon - gf.center_lon) * 111320.0 * (lat * PI / 180.0).cos();
            if (dx * dx + dy * dy).sqrt() < gf.radius_m {
                self.inside_geofence = Some(gf.id);
                break;
            }
        }

        // ─ OTA simulation ────────────────────────────────────────────────────
        if elapsed > 60.0 && !self.ota_available && !self.ota_downloading {
            self.ota_available = true;
            self.ota_version = "SW_01.24.001".into();
            self.ota_size_mb = 48.5;
        }
        if self.ota_downloading {
            self.ota_progress_pct = (self.ota_progress_pct + dt * 0.5).min(100.0);
            self.data_used_mb += 0.5 * dt;
            if self.ota_progress_pct >= 100.0 {
                self.ota_downloading = false;
                self.ota_available = false;
                let ev = TelematicsEvent {
                    timestamp: elapsed,
                    sent: true,
                    ack: true,
                    event_type: TelEventType::OtaUpdateComplete,
                    description: format!("OTA {} installed", self.ota_version),
                };
                self.push_event(ev);
            }
        }
    }

    fn push_event(&mut self, ev: TelematicsEvent) {
        self.events.push_front(ev);
        if self.events.len() > 100 {
            self.events.pop_back();
        }
    }

    pub fn start_ota(&mut self) {
        if self.ota_available {
            self.ota_downloading = true;
            self.ota_progress_pct = 0.0;
        }
    }
}
