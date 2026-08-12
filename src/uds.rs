//! UDS — ISO 14229-1 Unified Diagnostic Services
//!
//! UDS is the standard diagnostic protocol used by ALL modern ECUs (automotive
//! and off-highway). Every OEM tool (CAT ET, CNH EST, AGCO, Deere Service
//! Advisor) sends UDS frames wrapped in J1939 or ISO-TP transport.
//!
//! Services implemented:
//!   0x10 DiagnosticSessionControl   — switch between default/extended/programming
//!   0x11 ECUReset                    — hard reset, soft reset, key-off/on
//!   0x14 ClearDiagnosticInformation  — erase DTCs
//!   0x19 ReadDTCInformation          — query active/stored/all DTCs
//!   0x22 ReadDataByIdentifier        — read live parameters by 2-byte DID
//!   0x27 SecurityAccess              — seed/key authentication for programming
//!   0x2E WriteDataByIdentifier       — write calibration/config parameters
//!   0x31 RoutineControl              — start/stop routines (DPF regen, etc.)
//!   0x3E TesterPresent               — keep session alive (sent every ≤5 s)
//!   0x34 RequestDownload             — begin firmware/cal download
//!   0x36 TransferData                — stream data blocks
//!   0x37 RequestTransferExit         — finish download
//!
//! Transport: J1939 TP (BAM or CMDT) or ISO-TP (ISO 15765-2) depending on bus.
//! Max payload without transport: 7 bytes (with J1939 TP: up to 1785 bytes).

use std::collections::VecDeque;

// ── Diagnostic Session ────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DiagSession {
    /// Default: DTC reading, live data (always available, no authentication)
    Default,
    /// Extended: clear DTCs, write parameters, actuator tests (auth required)
    Extended,
    /// Programming: firmware/calibration download (most restrictive)
    Programming,
}

impl std::fmt::Display for DiagSession {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DiagSession::Default => write!(f, "DEFAULT    "),
            DiagSession::Extended => write!(f, "EXTENDED   "),
            DiagSession::Programming => write!(f, "PROGRAMMING"),
        }
    }
}

// ── Security Level ────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum SecurityLevel {
    /// Locked — no programming access
    Locked,
    /// Level 1 (0x01/0x02) — extended diagnostic access
    Level1,
    /// Level 3 (0x05/0x06) — ECU programming / calibration write
    Level3,
    /// Level 9 (0x11/0x12) — Factory / supplier level (erases all data)
    Level9,
}

// ── UDS NRC (Negative Response Code) ─────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Nrc {
    GeneralReject = 0x10,
    ServiceNotSupported = 0x11,
    SubFunctionNotSupported = 0x12,
    IncorrectMessageLength = 0x13,
    ResponseTooLong = 0x14,
    BusyRepeatRequest = 0x21,
    ConditionsNotCorrect = 0x22,
    RequestSequenceError = 0x24,
    RequestOutOfRange = 0x31,
    SecurityAccessDenied = 0x33,
    InvalidKey = 0x35,
    ExceededNumberOfAttempts = 0x36,
    RequiredTimeDelayNotExpired = 0x37,
    UploadDownloadNotAccepted = 0x70,
    TransferDataSuspended = 0x71,
    GeneralProgrammingFailure = 0x72,
    RequestCorrectlyReceivedButResponsePending = 0x78,
    ServiceNotSupportedInActiveSession = 0x7F,
}

// ── DID (Data Identifier) registry ───────────────────────────────────────────
// Standard DIDs used across most ECUs

pub mod did {
    // Identification
    pub const VIN: u16 = 0xF190; // Vehicle Identification Number
    pub const ECU_SERIAL: u16 = 0xF18C; // ECU Serial Number
    pub const SW_VERSION: u16 = 0xF189; // Software Version Number
    pub const HW_VERSION: u16 = 0xF191; // Hardware Version Number
    pub const MANUF_DATE: u16 = 0xF18B; // Manufacturing Date
    pub const PROGRAM_DATE: u16 = 0xF19E; // ECU Programming Date
    pub const CALIBRATION_ID: u16 = 0xF180; // Active Calibration ID
    pub const FINGERPRINT: u16 = 0xF184; // Programming fingerprint

    // Live engine data (0xDD00-0xDDFF common OEM range)
    pub const ENGINE_RPM: u16 = 0xDD01;
    pub const ENGINE_LOAD: u16 = 0xDD02;
    pub const COOLANT_TEMP: u16 = 0xDD03;
    pub const OIL_PRESSURE: u16 = 0xDD04;
    pub const BOOST_PRESSURE: u16 = 0xDD05;
    pub const FUEL_RATE: u16 = 0xDD06;
    pub const ENGINE_TORQUE: u16 = 0xDD07;
    pub const DPF_SOOT: u16 = 0xDD08;
    pub const DEF_LEVEL: u16 = 0xDD09;
    pub const SCR_EFFICIENCY: u16 = 0xDD0A;
    pub const EXHAUST_TEMP: u16 = 0xDD0B;
    pub const RAIL_PRESSURE: u16 = 0xDD0C;
    pub const ENGINE_HOURS: u16 = 0xDD0D;
    pub const BATTERY_VOLTAGE: u16 = 0xDD0E;
    pub const VEHICLE_SPEED: u16 = 0xDD0F;

    // Calibration parameters (writeable in Extended session with security)
    pub const IDLE_RPM_CAL: u16 = 0xCA01; // Idle speed setpoint
    pub const RATED_RPM_CAL: u16 = 0xCA02; // Rated speed limit
    pub const MAX_TORQUE_CAL: u16 = 0xCA03; // Max torque limit
    pub const GOVERNOR_MODE_CAL: u16 = 0xCA04; // Governor mode select
    pub const DPF_REGEN_THRESHOLD: u16 = 0xCA05; // DPF regen trigger %
    pub const DEF_WARNING_LEVEL: u16 = 0xCA06; // DEF warn threshold %
    pub const FUEL_MAP_SELECT: u16 = 0xCA07; // Fuel map (0=Std,1=Eco,2=Pwr)
    pub const PTO_MAX_RPM: u16 = 0xCA08; // PTO speed limit
    pub const SERVICE_INTERVAL_H: u16 = 0xCA09; // Service interval hours

    // Routine IDs
    pub const ROUTINE_DPF_REGEN: u16 = 0xDF01; // Trigger active DPF regen
    pub const ROUTINE_INJECTOR_TEST: u16 = 0xDF02; // Cylinder balance test
    pub const ROUTINE_ACTUATOR_TEST: u16 = 0xDF03; // VGT/EGR actuator test
    pub const ROUTINE_RESET_ADAP: u16 = 0xDF04; // Reset learned adaptations
}

// ── Freeze Frame — snapshot of all parameters when a DTC sets ────────────────

#[derive(Debug, Clone)]
pub struct FreezeFrame {
    pub dtc_spn: u32,
    pub dtc_fmi: u8,
    pub timestamp_h: f64, // engine hours when set
    pub engine_rpm: f64,
    pub engine_load_pct: f64,
    pub vehicle_speed: f64,
    pub coolant_temp_c: f64,
    pub oil_pressure_kpa: f64,
    pub fuel_rate_lph: f64,
    pub boost_kpa: f64,
    pub battery_v: f64,
    pub throttle_pct: f64,
    pub ambient_temp_c: f64,
}

// ── Download state machine ────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DownloadState {
    Idle,
    Requested,    // 0x34 received, size/address known
    Transferring, // 0x36 blocks flowing
    Complete,     // 0x37 received, waiting for ECU to validate/flash
}

// ── UDS Server ────────────────────────────────────────────────────────────────

pub struct UdsServer {
    pub node_name: &'static str,
    pub sa: u8,

    // ─ Session & security ────────────────────────────────────────────────────
    pub session: DiagSession,
    pub security: SecurityLevel,
    session_timer: f64,    // seconds since last tester activity
    security_seed: u32,    // current seed (changes each request)
    security_attempts: u8, // failed attempts counter (lockout after 3)
    lockout_timer: f64,

    // ─ DTC storage (persists key cycles in real ECU via NVM) ─────────────────
    pub active_dtcs: Vec<crate::j1939::Dtc>,
    pub stored_dtcs: Vec<crate::j1939::Dtc>,
    pub freeze_frames: Vec<FreezeFrame>,

    // ─ Calibration parameters ────────────────────────────────────────────────
    pub idle_rpm_cal: f64,
    pub rated_rpm_cal: f64,
    pub max_torque_cal: f64,
    pub dpf_regen_threshold: f64,
    pub def_warning_level: f64,
    pub fuel_map_select: u8,
    pub service_interval_h: f64,
    pub pto_max_rpm: f64,

    // ─ Identification strings ────────────────────────────────────────────────
    pub vin: String,
    pub sw_version: String,
    pub hw_version: String,
    pub cal_id: String,
    pub ecu_serial: String,

    // ─ Download session ──────────────────────────────────────────────────────
    pub download_state: DownloadState,
    download_address: u32,
    download_expected_len: u32,
    download_received_len: u32,
    download_block_num: u8,

    // ─ Routine states ────────────────────────────────────────────────────────
    pub dpf_regen_routine_active: bool,
    pub injector_test_active: bool,

    // ─ Event log ─────────────────────────────────────────────────────────────
    pub event_log: VecDeque<UdsEvent>,
}

#[derive(Debug, Clone)]
pub struct UdsEvent {
    pub timestamp: f64,
    pub service: u8,
    pub sub_func: Option<u8>,
    pub did: Option<u16>,
    pub result: UdsEventResult,
    pub detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UdsEventResult {
    Positive,
    Negative(u8),
    SecurityFail,
}

impl UdsServer {
    pub fn new(node_name: &'static str, sa: u8) -> Self {
        UdsServer {
            node_name,
            sa,
            session: DiagSession::Default,
            security: SecurityLevel::Locked,
            session_timer: 0.0,
            security_seed: 0xA5A5A5A5,
            security_attempts: 0,
            lockout_timer: 0.0,
            active_dtcs: Vec::new(),
            stored_dtcs: Vec::new(),
            freeze_frames: Vec::new(),
            idle_rpm_cal: 800.0,
            rated_rpm_cal: 2200.0,
            max_torque_cal: 1050.0,
            dpf_regen_threshold: 75.0,
            def_warning_level: 10.0,
            fuel_map_select: 0,
            service_interval_h: 500.0,
            pto_max_rpm: 1000.0,
            vin: "1HD1KEM16FB123456".into(),
            sw_version: "SW_01.23.004".into(),
            hw_version: "HW_02.00".into(),
            cal_id: "CAL_TIER4_V3.1".into(),
            ecu_serial: "ECU20240811001".into(),
            download_state: DownloadState::Idle,
            download_address: 0,
            download_expected_len: 0,
            download_received_len: 0,
            download_block_num: 0,
            dpf_regen_routine_active: false,
            injector_test_active: false,
            event_log: VecDeque::new(),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Tick: advance session timer, auto-expire Extended/Programming sessions.
    pub fn tick(&mut self, dt: f64) {
        self.session_timer += dt;

        // Extended session times out after 5 s without TesterPresent
        if self.session != DiagSession::Default && self.session_timer > 5.0 {
            self.session = DiagSession::Default;
            self.security = SecurityLevel::Locked;
        }

        // Lockout timer for security (penalty 10 s after 3 failed attempts)
        if self.lockout_timer > 0.0 {
            self.lockout_timer -= dt;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Process a raw UDS request payload. Returns response payload (positive or negative).
    pub fn process(&mut self, request: &[u8], elapsed: f64) -> Vec<u8> {
        if request.is_empty() {
            return self.nrc(0x00, Nrc::IncorrectMessageLength);
        }
        let service = request[0];

        

        match service {
            0x10 => self.svc_session_control(request, elapsed),
            0x11 => self.svc_ecu_reset(request, elapsed),
            0x14 => self.svc_clear_dtc(request, elapsed),
            0x19 => self.svc_read_dtc(request, elapsed),
            0x22 => self.svc_read_data_by_id(request, elapsed),
            0x27 => self.svc_security_access(request, elapsed),
            0x2E => self.svc_write_data_by_id(request, elapsed),
            0x31 => self.svc_routine_control(request, elapsed),
            0x34 => self.svc_request_download(request, elapsed),
            0x36 => self.svc_transfer_data(request, elapsed),
            0x37 => self.svc_request_transfer_exit(request, elapsed),
            0x3E => self.svc_tester_present(request, elapsed),
            _ => self.nrc(service, Nrc::ServiceNotSupported),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x10 — Diagnostic Session Control
    fn svc_session_control(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 2 {
            return self.nrc(0x10, Nrc::IncorrectMessageLength);
        }
        let sub = req[1];
        let new_session = match sub {
            0x01 => DiagSession::Default,
            0x02 => DiagSession::Extended,
            0x03 => DiagSession::Programming,
            _ => return self.nrc(0x10, Nrc::SubFunctionNotSupported),
        };
        // Programming session only allowed if already in Extended + security unlocked
        if new_session == DiagSession::Programming && self.security < SecurityLevel::Level3 {
            return self.nrc(0x10, Nrc::SecurityAccessDenied);
        }
        self.session = new_session;
        self.session_timer = 0.0;
        self.log(
            ts,
            0x10,
            Some(sub),
            None,
            UdsEventResult::Positive,
            format!("→ {:?}", self.session),
        );
        vec![0x50, sub, 0x00, 0x19, 0x01, 0xF4] // P2=25ms, P2*=500ms
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x11 — ECU Reset
    fn svc_ecu_reset(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 2 {
            return self.nrc(0x11, Nrc::IncorrectMessageLength);
        }
        if self.session == DiagSession::Default {
            return self.nrc(0x11, Nrc::ServiceNotSupportedInActiveSession);
        }
        let sub = req[1];
        match sub {
            0x01 => self.log(
                ts,
                0x11,
                Some(sub),
                None,
                UdsEventResult::Positive,
                "Hard Reset".into(),
            ),
            0x02 => self.log(
                ts,
                0x11,
                Some(sub),
                None,
                UdsEventResult::Positive,
                "Key-Off/On Reset".into(),
            ),
            0x03 => self.log(
                ts,
                0x11,
                Some(sub),
                None,
                UdsEventResult::Positive,
                "Soft Reset".into(),
            ),
            _ => return self.nrc(0x11, Nrc::SubFunctionNotSupported),
        }
        self.session = DiagSession::Default;
        self.security = SecurityLevel::Locked;
        vec![0x51, sub]
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x14 — Clear Diagnostic Information
    fn svc_clear_dtc(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 4 {
            return self.nrc(0x14, Nrc::IncorrectMessageLength);
        }
        if self.session == DiagSession::Default {
            return self.nrc(0x14, Nrc::ServiceNotSupportedInActiveSession);
        }
        let group = ((req[1] as u32) << 16) | ((req[2] as u32) << 8) | (req[3] as u32);
        match group {
            0xFFFFFF => {
                // clear all groups
                self.active_dtcs.clear();
                self.stored_dtcs.clear();
                self.freeze_frames.clear();
            }
            spn => {
                self.active_dtcs.retain(|d| d.spn != spn);
                self.stored_dtcs.retain(|d| d.spn != spn);
            }
        }
        self.log(
            ts,
            0x14,
            None,
            None,
            UdsEventResult::Positive,
            format!("Cleared group 0x{:06X}", group),
        );
        vec![0x54]
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x19 — Read DTC Information
    fn svc_read_dtc(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 2 {
            return self.nrc(0x19, Nrc::IncorrectMessageLength);
        }
        let sub = req[1];
        let mut resp = vec![0x59, sub];

        match sub {
            // 0x01 — Report number of DTCs by status mask
            0x01 => {
                let mask = if req.len() >= 3 { req[2] } else { 0xFF };
                let count = self.active_dtcs.len() as u16;
                resp.push(mask); // status availability mask
                resp.push(0x01); // DTC format (ISO 14229 format 1)
                resp.push((count >> 8) as u8);
                resp.push((count & 0xFF) as u8);
            }
            // 0x02 — Report DTC by status mask (active)
            0x02 => {
                let mask = if req.len() >= 3 { req[2] } else { 0xFF };
                resp.push(mask);
                for dtc in &self.active_dtcs {
                    // 3 bytes DTC + 1 byte status
                    let dtc_bytes = dtc_to_3bytes(dtc.spn, dtc.fmi);
                    resp.extend_from_slice(&dtc_bytes);
                    resp.push(0x08); // status: confirmed + test failed
                }
            }
            // 0x0A — Report all supported DTCs
            0x0A => {
                resp.push(0xFF); // all status bits supported
                for dtc in self.active_dtcs.iter().chain(self.stored_dtcs.iter()) {
                    let dtc_bytes = dtc_to_3bytes(dtc.spn, dtc.fmi);
                    resp.extend_from_slice(&dtc_bytes);
                    resp.push(if dtc.active { 0x08 } else { 0x00 });
                }
            }
            // 0x04 — Report DTC snapshot (freeze frame) by DTC number
            0x04 => {
                if req.len() >= 5 {
                    let spn = ((req[2] as u32) << 16) | ((req[3] as u32) << 8) | (req[4] as u32);
                    if let Some(ff) = self
                        .freeze_frames
                        .iter()
                        .find(|f| dtc_to_3bytes(f.dtc_spn, f.dtc_fmi) == dtc_to_3bytes(spn, 0))
                    {
                        let dtc_b = dtc_to_3bytes(ff.dtc_spn, ff.dtc_fmi);
                        resp.extend_from_slice(&dtc_b);
                        resp.push(0x01); // record number
                                         // Append snapshot data as DID 0xDD01-0xDD0F
                        let rpm_raw = (ff.engine_rpm / 0.125) as u16;
                        resp.push(0xDD);
                        resp.push(0x01);
                        resp.push((rpm_raw >> 8) as u8);
                        resp.push((rpm_raw & 0xFF) as u8);
                        resp.push(0xDD);
                        resp.push(0x03);
                        resp.push((ff.coolant_temp_c + 40.0) as u8);
                    }
                }
            }
            _ => return self.nrc(0x19, Nrc::SubFunctionNotSupported),
        }
        self.log(
            ts,
            0x19,
            Some(sub),
            None,
            UdsEventResult::Positive,
            format!("{} DTCs", self.active_dtcs.len()),
        );
        resp
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x22 — Read Data By Identifier
    fn svc_read_data_by_id(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 3 {
            return self.nrc(0x22, Nrc::IncorrectMessageLength);
        }
        let did = ((req[1] as u16) << 8) | req[2] as u16;
        let mut resp = vec![0x62, req[1], req[2]];

        match did {
            d if d == did::VIN => resp.extend_from_slice(self.vin.as_bytes()),
            d if d == did::SW_VERSION => resp.extend_from_slice(self.sw_version.as_bytes()),
            d if d == did::HW_VERSION => resp.extend_from_slice(self.hw_version.as_bytes()),
            d if d == did::CALIBRATION_ID => resp.extend_from_slice(self.cal_id.as_bytes()),
            d if d == did::ECU_SERIAL => resp.extend_from_slice(self.ecu_serial.as_bytes()),
            d if d == did::IDLE_RPM_CAL => {
                let v = (self.idle_rpm_cal / 0.125) as u16;
                resp.push((v >> 8) as u8);
                resp.push((v & 0xFF) as u8);
            }
            d if d == did::RATED_RPM_CAL => {
                let v = (self.rated_rpm_cal / 0.125) as u16;
                resp.push((v >> 8) as u8);
                resp.push((v & 0xFF) as u8);
            }
            d if d == did::MAX_TORQUE_CAL => {
                let v = self.max_torque_cal as u16;
                resp.push((v >> 8) as u8);
                resp.push((v & 0xFF) as u8);
            }
            d if d == did::DPF_REGEN_THRESHOLD => resp.push((self.dpf_regen_threshold / 0.4) as u8),
            d if d == did::DEF_WARNING_LEVEL => resp.push((self.def_warning_level / 0.4) as u8),
            d if d == did::FUEL_MAP_SELECT => resp.push(self.fuel_map_select),
            d if d == did::SERVICE_INTERVAL_H => {
                let v = (self.service_interval_h / 0.05) as u32;
                resp.extend_from_slice(&v.to_be_bytes());
            }
            _ => return self.nrc(0x22, Nrc::RequestOutOfRange),
        }
        self.log(
            ts,
            0x22,
            None,
            Some(did),
            UdsEventResult::Positive,
            format!("DID 0x{:04X}", did),
        );
        resp
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x27 — Security Access (seed/key challenge-response)
    fn svc_security_access(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 2 {
            return self.nrc(0x27, Nrc::IncorrectMessageLength);
        }
        if self.lockout_timer > 0.0 {
            return self.nrc(0x27, Nrc::RequiredTimeDelayNotExpired);
        }

        let sub = req[1];
        match sub {
            // Odd sub-functions = seed request
            sub if sub % 2 == 1 => {
                // Generate new seed (deterministic for simulation)
                self.security_seed =
                    (self.security_seed.wrapping_mul(1664525)).wrapping_add(1013904223);
                let seed_bytes = self.security_seed.to_be_bytes();
                let mut resp = vec![0x67, sub];
                resp.extend_from_slice(&seed_bytes);
                self.log(
                    ts,
                    0x27,
                    Some(sub),
                    None,
                    UdsEventResult::Positive,
                    format!("Seed 0x{:08X}", self.security_seed),
                );
                resp
            }
            // Even sub-functions = key send
            sub if sub % 2 == 0 => {
                if req.len() < 6 {
                    return self.nrc(0x27, Nrc::IncorrectMessageLength);
                }
                let key = ((req[2] as u32) << 24)
                    | ((req[3] as u32) << 16)
                    | ((req[4] as u32) << 8)
                    | (req[5] as u32);
                // Algorithm: XOR seed with constant (simplified — real uses AES/SHA)
                let expected_key = self.security_seed ^ 0xDEADBEEF;
                if key == expected_key {
                    self.security = match sub {
                        0x02 => SecurityLevel::Level1,
                        0x06 => SecurityLevel::Level3,
                        0x12 => SecurityLevel::Level9,
                        _ => SecurityLevel::Level1,
                    };
                    self.security_attempts = 0;
                    self.log(
                        ts,
                        0x27,
                        Some(sub),
                        None,
                        UdsEventResult::Positive,
                        format!("Unlocked {:?}", self.security),
                    );
                    vec![0x67, sub]
                } else {
                    self.security_attempts += 1;
                    if self.security_attempts >= 3 {
                        self.lockout_timer = 10.0; // 10-second lockout
                        self.log(
                            ts,
                            0x27,
                            Some(sub),
                            None,
                            UdsEventResult::SecurityFail,
                            "LOCKED OUT after 3 failures".into(),
                        );
                        return self.nrc(0x27, Nrc::ExceededNumberOfAttempts);
                    }
                    self.log(
                        ts,
                        0x27,
                        Some(sub),
                        None,
                        UdsEventResult::SecurityFail,
                        format!("Wrong key — attempt {}/3", self.security_attempts),
                    );
                    self.nrc(0x27, Nrc::InvalidKey)
                }
            }
            _ => self.nrc(0x27, Nrc::SubFunctionNotSupported),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x2E — Write Data By Identifier
    fn svc_write_data_by_id(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 3 {
            return self.nrc(0x2E, Nrc::IncorrectMessageLength);
        }
        if self.session != DiagSession::Extended || self.security < SecurityLevel::Level1 {
            return self.nrc(0x2E, Nrc::SecurityAccessDenied);
        }
        let did = ((req[1] as u16) << 8) | req[2] as u16;
        let data = &req[3..];

        match did {
            d if d == did::IDLE_RPM_CAL && data.len() >= 2 => {
                self.idle_rpm_cal = (((data[0] as u16) << 8) | data[1] as u16) as f64 * 0.125;
            }
            d if d == did::RATED_RPM_CAL && data.len() >= 2 => {
                self.rated_rpm_cal = (((data[0] as u16) << 8) | data[1] as u16) as f64 * 0.125;
            }
            d if d == did::MAX_TORQUE_CAL && data.len() >= 2 => {
                self.max_torque_cal = (((data[0] as u16) << 8) | data[1] as u16) as f64;
            }
            d if d == did::DPF_REGEN_THRESHOLD && !data.is_empty() => {
                self.dpf_regen_threshold = data[0] as f64 * 0.4;
            }
            d if d == did::DEF_WARNING_LEVEL && !data.is_empty() => {
                self.def_warning_level = data[0] as f64 * 0.4;
            }
            d if d == did::FUEL_MAP_SELECT && !data.is_empty() => {
                self.fuel_map_select = data[0].min(2);
            }
            _ => return self.nrc(0x2E, Nrc::RequestOutOfRange),
        }
        self.log(
            ts,
            0x2E,
            None,
            Some(did),
            UdsEventResult::Positive,
            format!("DID 0x{:04X} written", did),
        );
        vec![0x6E, req[1], req[2]]
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x31 — Routine Control
    fn svc_routine_control(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if req.len() < 4 {
            return self.nrc(0x31, Nrc::IncorrectMessageLength);
        }
        if self.session == DiagSession::Default {
            return self.nrc(0x31, Nrc::ServiceNotSupportedInActiveSession);
        }
        let sub = req[1]; // 0x01=start, 0x02=stop, 0x03=request result
        let rid_hi = req[2];
        let rid_lo = req[3];
        let rid: u16 = ((rid_hi as u16) << 8) | rid_lo as u16;

        match rid {
            r if r == did::ROUTINE_DPF_REGEN => match sub {
                0x01 => {
                    self.dpf_regen_routine_active = true;
                }
                0x02 => {
                    self.dpf_regen_routine_active = false;
                }
                _ => {}
            },
            r if r == did::ROUTINE_INJECTOR_TEST => match sub {
                0x01 => {
                    self.injector_test_active = true;
                }
                0x02 => {
                    self.injector_test_active = false;
                }
                _ => {}
            },
            _ => return self.nrc(0x31, Nrc::RequestOutOfRange),
        }
        self.log(
            ts,
            0x31,
            Some(sub),
            Some(rid),
            UdsEventResult::Positive,
            format!("Routine 0x{:04X} sub={}", rid, sub),
        );
        vec![0x71, sub, rid_hi, rid_lo, 0x00]
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Service 0x34 — Request Download
    fn svc_request_download(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if self.security < SecurityLevel::Level3 {
            return self.nrc(0x34, Nrc::SecurityAccessDenied);
        }
        if req.len() < 8 {
            return self.nrc(0x34, Nrc::IncorrectMessageLength);
        }
        self.download_address = ((req[2] as u32) << 24)
            | ((req[3] as u32) << 16)
            | ((req[4] as u32) << 8)
            | req[5] as u32;
        self.download_expected_len =
            ((req[4] as u32) << 24) | ((req[5] as u32) << 16) | ((req[6] as u32) << 8) | req[7] as u32;
        self.download_received_len = 0;
        self.download_block_num = 1;
        self.download_state = DownloadState::Requested;
        self.log(
            ts,
            0x34,
            None,
            None,
            UdsEventResult::Positive,
            format!("Download @0x{:08X}", self.download_address),
        );
        vec![0x74, 0x20, 0x00, 0x81] // maxBlockLen = 0x0081 = 129 bytes
    }

    // Service 0x36 — Transfer Data
    fn svc_transfer_data(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        if self.download_state != DownloadState::Requested
            && self.download_state != DownloadState::Transferring
        {
            return self.nrc(0x36, Nrc::RequestSequenceError);
        }
        if req.len() < 2 {
            return self.nrc(0x36, Nrc::IncorrectMessageLength);
        }
        let block_num = req[1];
        if block_num != self.download_block_num {
            return self.nrc(0x36, Nrc::RequestSequenceError);
        }
        self.download_received_len += (req.len() - 2) as u32;
        self.download_block_num = self.download_block_num.wrapping_add(1);
        self.download_state = DownloadState::Transferring;
        self.log(
            ts,
            0x36,
            None,
            None,
            UdsEventResult::Positive,
            format!("Block {} ({} bytes)", block_num, req.len() - 2),
        );
        vec![0x76, block_num]
    }

    // Service 0x37 — Request Transfer Exit
    fn svc_request_transfer_exit(&mut self, _req: &[u8], ts: f64) -> Vec<u8> {
        if self.download_state != DownloadState::Transferring {
            return self.nrc(0x37, Nrc::RequestSequenceError);
        }
        self.download_state = DownloadState::Complete;
        self.log(
            ts,
            0x37,
            None,
            None,
            UdsEventResult::Positive,
            format!("{} bytes transferred", self.download_received_len),
        );
        vec![0x77]
    }

    // Service 0x3E — Tester Present (keep-alive)
    fn svc_tester_present(&mut self, req: &[u8], ts: f64) -> Vec<u8> {
        self.session_timer = 0.0;
        let sub = if req.len() >= 2 { req[1] } else { 0x00 };
        // sub=0x80 = "zero sub function" (suppress positive response)
        if sub == 0x80 {
            return Vec::new();
        }
        self.log(
            ts,
            0x3E,
            Some(sub),
            None,
            UdsEventResult::Positive,
            "Keep-alive".into(),
        );
        vec![0x7E, sub]
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Called by ECM when a new DTC becomes active — stores freeze frame.
    pub fn record_dtc_set(&mut self, spn: u32, fmi: u8, ff: FreezeFrame) {
        use crate::j1939::{Dtc, DtcSeverity};
        if !self
            .active_dtcs
            .iter()
            .any(|d| d.spn == spn && d.fmi == fmi)
        {
            self.active_dtcs.push(Dtc {
                spn,
                fmi,
                count: 1,
                active: true,
                desc: "via UDS record",
                severity: DtcSeverity::Amber,
            });
            self.freeze_frames.push(ff);
            // Move to stored too (persists after clear)
            self.stored_dtcs.push(Dtc {
                spn,
                fmi,
                count: 1,
                active: false,
                desc: "stored",
                severity: DtcSeverity::Amber,
            });
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    fn nrc(&self, service: u8, code: Nrc) -> Vec<u8> {
        vec![0x7F, service, code as u8]
    }

    fn log(
        &mut self,
        ts: f64,
        svc: u8,
        sf: Option<u8>,
        did: Option<u16>,
        res: UdsEventResult,
        detail: String,
    ) {
        self.event_log.push_front(UdsEvent {
            timestamp: ts,
            service: svc,
            sub_func: sf,
            did,
            result: res,
            detail,
        });
        if self.event_log.len() > 50 {
            self.event_log.pop_back();
        }
    }

    pub fn service_name(svc: u8) -> &'static str {
        match svc {
            0x10 => "DiagSessCtrl",
            0x11 => "ECUReset",
            0x14 => "ClearDTC",
            0x19 => "ReadDTC",
            0x22 => "RdDataByID",
            0x27 => "SecurityAccess",
            0x2E => "WrDataByID",
            0x31 => "RoutineCtrl",
            0x34 => "ReqDownload",
            0x36 => "TransferData",
            0x37 => "XferExit",
            0x3E => "TesterPresent",
            0x7F => "NRC",
            _ => "Unknown",
        }
    }
}

// Helper: pack SPN+FMI into J1939 DTC 3-byte format
fn dtc_to_3bytes(spn: u32, fmi: u8) -> [u8; 3] {
    [
        (spn & 0xFF) as u8,
        ((spn >> 8) & 0xFF) as u8,
        ((((spn >> 16) & 0x7) as u8) | ((fmi & 0x1F) << 3)),
    ]
}
