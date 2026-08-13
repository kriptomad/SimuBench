use std::collections::BTreeSet;
use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::Write;
use std::net::{TcpStream, ToSocketAddrs};
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::sync::OnceLock;
use std::thread;
use std::time::{Duration, Instant};

use super::ecm_params::{j1939_pgn, merge_snapshot, EcmSnapshot, PGN_EEC1, PGN_EFL_P1, PGN_ET1};
use super::hw::{CanFrame, Frame, HwConfig, HwError, HwMode};
use super::replay::{append_record, frame_to_record, JsonlRecord};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

const DID_VIN: u16 = 0xF190;
const DID_SW_VERSION: u16 = 0xF189;
const DID_HW_VERSION: u16 = 0xF191;
const DID_CALIBRATION_ID: u16 = 0xF180;
const DID_ECU_SERIAL: u16 = 0xF18C;
const DID_FINGERPRINT: u16 = 0xF184;
const DID_BATTERY_VOLTAGE: u16 = 0xDD0E;
const MIN_FLASH_SUPPLY_V: f64 = 11.8;

static FLASH_AUDIT_CHAIN: OnceLock<Mutex<[u8; 32]>> = OnceLock::new();

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DetectResult {
    pub source_addresses: Vec<u8>,
}

#[derive(Clone)]
pub struct LiveFeed {
    stop: Arc<AtomicBool>,
    alive: Arc<AtomicBool>,
    latest_snapshot: Arc<Mutex<EcmSnapshot>>,
    history: Arc<Mutex<VecDeque<SnapshotPoint>>>,
    last_error: Arc<Mutex<Option<String>>>,
    last_update_ms: Arc<AtomicU64>,
    frames_total: Arc<AtomicU64>,
}

#[derive(Debug, Clone)]
pub struct SnapshotPoint {
    pub ts_ms: u64,
    pub engine_speed_rpm: Option<f64>,
    pub accel_pedal_pct: Option<f64>,
    pub coolant_temp_c: Option<f64>,
    pub fuel_temp_c: Option<f64>,
    pub oil_pressure_kpa: Option<f64>,
}

impl LiveFeed {
    pub fn latest_snapshot(&self) -> EcmSnapshot {
        match self.latest_snapshot.lock() {
            Ok(s) => s.clone(),
            Err(poisoned) => poisoned.into_inner().clone(),
        }
    }

    pub fn last_update_ms(&self) -> u64 {
        self.last_update_ms.load(Ordering::Relaxed)
    }

    pub fn frames_total(&self) -> u64 {
        self.frames_total.load(Ordering::Relaxed)
    }

    pub fn recent_points(&self, limit: usize) -> Vec<SnapshotPoint> {
        let hist = match self.history.lock() {
            Ok(h) => h,
            Err(poisoned) => poisoned.into_inner(),
        };
        let count = hist.len().min(limit);
        hist.iter().skip(hist.len() - count).cloned().collect()
    }

    pub fn is_alive(&self) -> bool {
        self.alive.load(Ordering::Relaxed)
    }

    pub fn last_error(&self) -> Option<String> {
        match self.last_error.lock() {
            Ok(e) => e.clone(),
            Err(poisoned) => poisoned.into_inner().clone(),
        }
    }

    pub fn export_csv<P: AsRef<Path>>(&self, path: P) -> Result<(), HwError> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)
                .map_err(|e| HwError::Unknown(format!("create csv dir failed: {e}")))?;
        }

        let hist = self
            .history
            .lock()
            .map_err(|_| HwError::Unknown("history lock poisoned".to_string()))?;

        let mut f = File::create(path)
            .map_err(|e| HwError::Unknown(format!("create csv file failed: {e}")))?;
        writeln!(
            f,
            "ts_ms,engine_speed_rpm,accel_pedal_pct,coolant_temp_c,fuel_temp_c,oil_pressure_kpa"
        )
        .map_err(|e| HwError::Unknown(format!("write csv header failed: {e}")))?;

        for p in hist.iter() {
            writeln!(
                f,
                "{},{},{},{},{},{}",
                p.ts_ms,
                optf(p.engine_speed_rpm),
                optf(p.accel_pedal_pct),
                optf(p.coolant_temp_c),
                optf(p.fuel_temp_c),
                optf(p.oil_pressure_kpa)
            )
            .map_err(|e| HwError::Unknown(format!("write csv row failed: {e}")))?;
        }
        Ok(())
    }

    pub fn stop(&self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

pub fn detect_ecms(cfg: &HwConfig, timeout: Duration) -> Result<DetectResult, HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "Detect requires --hw-mode=live".to_string(),
        ));
    }
    check_live_channel_policy(cfg)?;

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;

    let start = Instant::now();
    let mut found = BTreeSet::new();

    while start.elapsed() < timeout {
        match adapter.read_frame() {
            Ok(Frame::Can(cf)) => {
                let pgn = j1939_pgn(cf.id);
                let sa = (cf.id & 0xFF) as u8;
                if matches!(pgn, PGN_EEC1 | PGN_ET1 | PGN_EFL_P1 | 60928) {
                    found.insert(sa);
                }
            }
            Ok(Frame::Serial(_)) => {
                // For serial-only links, one logical ECM channel is considered detected.
                found.insert(0);
            }
            Err(HwError::Timeout) => {}
            Err(e) => {
                adapter.close()?;
                return Err(e);
            }
        }
    }

    adapter.close()?;
    Ok(DetectResult {
        source_addresses: found.into_iter().collect(),
    })
}

pub fn connect_ecm(cfg: &HwConfig, target_sa: Option<u8>) -> Result<(), HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "Connect requires --hw-mode=live".to_string(),
        ));
    }
    check_live_channel_policy(cfg)?;

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;

    // For CAN links, send standard J1939 PGN requests as an integration handshake.
    if cfg.can_interface.is_some() {
        let dst = target_sa.unwrap_or(0xFF);
        let reqs = [60928_u32, PGN_EEC1, PGN_ET1, PGN_EFL_P1];
        for pgn in reqs {
            let mut data = [0u8; 8];
            data[0] = (pgn & 0xFF) as u8;
            data[1] = ((pgn >> 8) & 0xFF) as u8;
            data[2] = ((pgn >> 16) & 0xFF) as u8;
            let req_id = 0x18EA0000 | ((dst as u32) << 8) | 0xF9; // Request PGN from tool SA=0xF9
            let req = Frame::Can(CanFrame {
                id: req_id,
                dlc: 8,
                data,
                len: 8,
                timestamp_ms: Some(now_ms()),
            });
            if let Err(e) = adapter.send_frame(req) {
                adapter.close()?;
                return Err(e);
            }
        }

        // Authentication/identification proxy: require at least one expected PGN
        // from the requested source (or any source when no target SA is provided).
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut got_response = false;
        while Instant::now() < deadline {
            match adapter.read_frame() {
                Ok(Frame::Can(cf)) => {
                    let pgn = j1939_pgn(cf.id);
                    let sa = (cf.id & 0xFF) as u8;
                    let valid_pgn = matches!(pgn, 60928 | PGN_EEC1 | PGN_ET1 | PGN_EFL_P1);
                    let valid_sa = target_sa.is_none() || Some(sa) == target_sa;
                    if valid_pgn && valid_sa {
                        got_response = true;
                        break;
                    }
                }
                Ok(Frame::Serial(_)) => {}
                Err(HwError::Timeout) => {}
                Err(e) => {
                    adapter.close()?;
                    return Err(e);
                }
            }
        }
        if !got_response {
            adapter.close()?;
            return Err(HwError::Timeout);
        }
    }

    let identity = read_ecu_identity(&mut *adapter, cfg, target_sa)?;
    if !cfg.allow_untrusted_ecu && identity.fingerprint.is_none() {
        adapter.close()?;
        return Err(HwError::Unknown(
            "connect rejected: missing fingerprint DID 0xF184 on trusted profile".to_string(),
        ));
    }

    adapter.close()?;
    Ok(())
}

pub fn start_retrieve_data(cfg: HwConfig, target_sa: Option<u8>) -> Result<LiveFeed, HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "Retrieve requires --hw-mode=live".to_string(),
        ));
    }
    check_live_channel_policy(&cfg)?;

    let stop = Arc::new(AtomicBool::new(false));
    let alive = Arc::new(AtomicBool::new(true));
    let latest_snapshot = Arc::new(Mutex::new(EcmSnapshot::default()));
    let history = Arc::new(Mutex::new(VecDeque::<SnapshotPoint>::new()));
    let last_error = Arc::new(Mutex::new(None));
    let last_update_ms = Arc::new(AtomicU64::new(0));
    let frames_total = Arc::new(AtomicU64::new(0));

    let feed = LiveFeed {
        stop: Arc::clone(&stop),
        alive: Arc::clone(&alive),
        latest_snapshot: Arc::clone(&latest_snapshot),
        history: Arc::clone(&history),
        last_error: Arc::clone(&last_error),
        last_update_ms: Arc::clone(&last_update_ms),
        frames_total: Arc::clone(&frames_total),
    };

    thread::spawn(move || {
        let mut adapter = match super::hw::open_real_adapter(&cfg) {
            Ok(a) => a,
            Err(e) => {
                if let Ok(mut err) = last_error.lock() {
                    *err = Some(format!("open adapter failed: {e}"));
                }
                alive.store(false, Ordering::Relaxed);
                eprintln!("[ecm-live] open adapter failed: {e}");
                return;
            }
        };
        if let Err(e) = adapter.init(&cfg) {
            if let Ok(mut err) = last_error.lock() {
                *err = Some(format!("init adapter failed: {e}"));
            }
            alive.store(false, Ordering::Relaxed);
            eprintln!("[ecm-live] init adapter failed: {e}");
            return;
        }

        let rx_log = cfg.log_dir.join("ecm_rx.jsonl");
        let snapshot_log = cfg.log_dir.join("ecm_snapshot.jsonl");
        let mut snapshot = EcmSnapshot::default();

        while !stop.load(Ordering::Relaxed) {
            match adapter.read_frame() {
                Ok(frame) => {
                    frames_total.fetch_add(1, Ordering::Relaxed);
                    let rec = frame_to_record(&frame, "rx", None, Some(cfg.dry_run));
                    let _ = append_record(&rx_log, &rec);

                    if let Frame::Can(cf) = &frame {
                        let sa = (cf.id & 0xFF) as u8;
                        if target_sa.is_some() && Some(sa) != target_sa {
                            continue;
                        }

                        snapshot = merge_snapshot(snapshot, cf);
                        if let Ok(mut s) = latest_snapshot.lock() {
                            *s = snapshot.clone();
                        }

                        if let Ok(mut hist) = history.lock() {
                            hist.push_back(SnapshotPoint {
                                ts_ms: cf.timestamp_ms.unwrap_or_else(now_ms),
                                engine_speed_rpm: snapshot.engine_speed_rpm,
                                accel_pedal_pct: snapshot.accel_pedal_pct,
                                coolant_temp_c: snapshot.coolant_temp_c,
                                fuel_temp_c: snapshot.fuel_temp_c,
                                oil_pressure_kpa: snapshot.oil_pressure_kpa,
                            });
                            if hist.len() > 5000 {
                                let _ = hist.pop_front();
                            }
                        }

                        let ts = cf.timestamp_ms.unwrap_or_else(now_ms);
                        last_update_ms.store(ts, Ordering::Relaxed);

                        let snap_line = JsonlRecord {
                            ts,
                            transport: "can".to_string(),
                            dir: "snapshot".to_string(),
                            id: Some(cf.id),
                            dlc: Some(cf.dlc),
                            data: Some(
                                serde_json::to_string(&snapshot)
                                    .unwrap_or_else(|_| "{}".to_string()),
                            ),
                            raw_hex: None,
                            allowed: None,
                            dry_run: Some(cfg.dry_run),
                        };
                        let _ = append_record(&snapshot_log, &snap_line);
                    }
                }
                Err(HwError::Timeout) => {
                    thread::sleep(Duration::from_millis(20));
                }
                Err(e) => {
                    if let Ok(mut err) = last_error.lock() {
                        *err = Some(format!("read error: {e}"));
                    }
                    eprintln!("[ecm-live] read error: {e}");
                    thread::sleep(Duration::from_millis(150));
                }
            }
        }

        alive.store(false, Ordering::Relaxed);
        let _ = adapter.close();
    });

    Ok(feed)
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn optf(v: Option<f64>) -> String {
    match v {
        Some(x) => format!("{x:.4}"),
        None => String::new(),
    }
}

#[derive(Debug, Clone)]
pub struct FlashSummary {
    pub bytes_sent: usize,
    pub blocks_sent: usize,
    pub crc32: u32,
    pub supply_voltage_v: f64,
    pub vin: String,
    pub sw_version: String,
    pub hw_version: String,
    pub calibration_id: String,
    pub ecu_serial: String,
    pub transport_diagnostics: FlashTransportDiagnostics,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlashTransportDiagnostics {
    pub security_seed_positive: bool,
    pub security_unlock_positive: bool,
    pub request_download_positive: bool,
    pub transfer_data_blocks_attempted: usize,
    pub transfer_data_blocks_acked: usize,
    pub request_transfer_exit_positive: bool,
    pub fc_blocksize_seen: Option<u8>,
    pub fc_stmin_seen_ms: Option<u64>,
    pub wait_frame_count: u32,
    pub sequence_error_count: u32,
    pub flowcontrol_timeout_count: u32,
}

#[derive(Debug, Clone, Default)]
struct BridgeDiagAggregate {
    wait_frame_count: u32,
    sequence_error_count: u32,
    flowcontrol_timeout_count: u32,
    fc_blocksize_seen: Option<u8>,
    fc_stmin_seen_ms: Option<u64>,
}

#[derive(Debug, Deserialize)]
struct BridgeDiagLine {
    ts_ms: u64,
    wait_frame_count_delta: Option<u32>,
    sequence_error_count_delta: Option<u32>,
    flowcontrol_timeout_count_delta: Option<u32>,
    fc_blocksize: Option<u8>,
    fc_stmin_ms: Option<u64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EcuIdentity {
    pub vin: String,
    pub sw_version: String,
    pub hw_version: String,
    pub calibration_id: String,
    pub ecu_serial: String,
    pub fingerprint: Option<String>,
}

#[derive(Debug, Clone)]
struct FlashPreflight {
    supply_voltage_v: f64,
    identity: EcuIdentity,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FlashPreflightReport {
    pub supply_voltage_v: f64,
    pub identity: EcuIdentity,
    pub trusted_fingerprint: bool,
}

#[derive(Debug, Clone, Copy, Serialize)]
pub enum FlashAuditStatus {
    Ok,
    Failed,
}

#[derive(Debug, Serialize)]
struct FlashAuditEvent<'a> {
    ts_ms: u64,
    stage: &'a str,
    status: FlashAuditStatus,
    detail: String,
    target_sa: Option<u8>,
    dry_run: bool,
    prev_hash: String,
    event_hash: String,
}

fn hex_lower(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

fn append_flash_audit(
    cfg: &HwConfig,
    stage: &str,
    status: FlashAuditStatus,
    detail: impl Into<String>,
    target_sa: Option<u8>,
) {
    let chain = FLASH_AUDIT_CHAIN.get_or_init(|| Mutex::new([0u8; 32]));
    let mut chain_guard = match chain.lock() {
        Ok(g) => g,
        Err(poisoned) => poisoned.into_inner(),
    };
    let prev_hash = *chain_guard;
    let detail_str = detail.into();
    let canonical = format!(
        "{}|{}|{:?}|{}|{:?}|{}",
        now_ms(),
        stage,
        status,
        detail_str,
        target_sa,
        cfg.dry_run
    );
    let mut hasher = Sha256::new();
    hasher.update(prev_hash);
    hasher.update(canonical.as_bytes());
    let event_hash_arr: [u8; 32] = hasher.finalize().into();

    let event = FlashAuditEvent {
        ts_ms: now_ms(),
        stage,
        status,
        detail: detail_str,
        target_sa,
        dry_run: cfg.dry_run,
        prev_hash: hex_lower(&prev_hash),
        event_hash: hex_lower(&event_hash_arr),
    };
    let serialized = serde_json::to_string(&event).unwrap_or_else(|_| {
        format!(
            "{{\"ts_ms\":{},\"stage\":\"{}\",\"status\":\"Failed\",\"detail\":\"audit_serialize_failed\",\"target_sa\":{},\"dry_run\":{}}}",
            event.ts_ms,
            stage,
            target_sa
                .map(|sa| sa.to_string())
                .unwrap_or_else(|| "null".to_string()),
            cfg.dry_run
        )
    });

    let rec = JsonlRecord {
        ts: now_ms(),
        transport: "audit".to_string(),
        dir: "flash-stage".to_string(),
        id: None,
        dlc: None,
        data: Some(serialized),
        raw_hex: None,
        allowed: Some(cfg.write_effectively_enabled()),
        dry_run: Some(cfg.dry_run),
    };
    let _ = append_record(&cfg.log_dir.join("ecm_flash_audit.jsonl"), &rec);
    *chain_guard = event_hash_arr;
}

fn ensure_positive_response(
    rsp: &[u8],
    req_sid: u8,
    stage: &str,
    cfg: &HwConfig,
    target_sa: Option<u8>,
) -> Result<(), HwError> {
    if rsp.is_empty() {
        append_flash_audit(
            cfg,
            stage,
            FlashAuditStatus::Failed,
            "empty UDS response",
            target_sa,
        );
        return Err(HwError::Unknown(format!(
            "{stage}: empty UDS response for service 0x{req_sid:02X}"
        )));
    }

    if rsp[0] == 0x7F {
        let nrc = rsp.get(2).copied().unwrap_or(0x00);
        let sid = rsp.get(1).copied().unwrap_or(0x00);
        append_flash_audit(
            cfg,
            stage,
            FlashAuditStatus::Failed,
            format!("negative response SID=0x{sid:02X} NRC=0x{nrc:02X}"),
            target_sa,
        );
        return Err(HwError::Unknown(format!(
            "{stage}: UDS negative response SID=0x{sid:02X}, NRC=0x{nrc:02X}"
        )));
    }

    let expected = req_sid.wrapping_add(0x40);
    if rsp[0] != expected {
        append_flash_audit(
            cfg,
            stage,
            FlashAuditStatus::Failed,
            format!(
                "unexpected positive response SID=0x{:02X}, expected=0x{expected:02X}",
                rsp[0]
            ),
            target_sa,
        );
        return Err(HwError::Unknown(format!(
            "{stage}: unexpected response SID=0x{:02X}, expected=0x{expected:02X}",
            rsp[0]
        )));
    }

    append_flash_audit(
        cfg,
        stage,
        FlashAuditStatus::Ok,
        format!("positive response SID=0x{:02X}", rsp[0]),
        target_sa,
    );
    Ok(())
}

fn read_did_ascii(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: Option<u8>,
    did: u16,
    stage: &str,
) -> Result<String, HwError> {
    let req = [0x22, (did >> 8) as u8, (did & 0xFF) as u8];
    let rsp = send_uds_and_wait(
        adapter,
        cfg,
        target_sa,
        &req,
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&rsp, 0x22, stage, cfg, target_sa)?;
    if rsp.len() < 3 {
        return Err(HwError::ParseError {
            cause: format!("{stage}: short DID response"),
            raw_data: format!("len={}", rsp.len()),
        });
    }
    let rsp_did = ((rsp[1] as u16) << 8) | rsp[2] as u16;
    if rsp_did != did {
        return Err(HwError::ParseError {
            cause: format!("{stage}: DID echo mismatch"),
            raw_data: format!("expected=0x{did:04X}, got=0x{rsp_did:04X}"),
        });
    }
    let payload = &rsp[3..];
    if payload.is_empty() {
        return Err(HwError::ParseError {
            cause: format!("{stage}: empty DID payload"),
            raw_data: format!("did=0x{did:04X}"),
        });
    }
    let text = String::from_utf8(payload.to_vec()).map_err(|_| HwError::ParseError {
        cause: format!("{stage}: non-utf8 DID payload"),
        raw_data: format!("did=0x{did:04X}, bytes={}", hex_lower(payload)),
    })?;
    Ok(text.trim().to_string())
}

fn read_did_battery_voltage(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: Option<u8>,
) -> Result<f64, HwError> {
    let req = [0x22, (DID_BATTERY_VOLTAGE >> 8) as u8, (DID_BATTERY_VOLTAGE & 0xFF) as u8];
    let rsp = send_uds_and_wait(
        adapter,
        cfg,
        target_sa,
        &req,
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&rsp, 0x22, "preflight::battery_did", cfg, target_sa)?;
    if rsp.len() < 5 {
        return Err(HwError::ParseError {
            cause: "preflight::battery_did short payload".to_string(),
            raw_data: format!("len={}", rsp.len()),
        });
    }
    let raw = u16::from_be_bytes([rsp[3], rsp[4]]);
    Ok((raw as f64) * 0.05)
}

fn decode_battery_voltage_from_can(cf: &CanFrame) -> Option<f64> {
    let pgn = j1939_pgn(cf.id);
    if !matches!(pgn, 65271 | 65272) {
        return None;
    }
    let p = can_payload(cf);
    if p.len() < 2 {
        return None;
    }
    let raw = u16::from_le_bytes([p[0], p[1]]);
    if raw == 0xFFFF {
        return None;
    }
    Some((raw as f64) * 0.05)
}

fn sample_supply_voltage_from_bus(
    adapter: &mut dyn super::hw::HardwareInterface,
    timeout: Duration,
) -> Result<Option<f64>, HwError> {
    let deadline = Instant::now() + timeout;
    let mut max_v = f64::NEG_INFINITY;
    while Instant::now() < deadline {
        match adapter.read_frame() {
            Ok(Frame::Can(cf)) => {
                if let Some(v) = decode_battery_voltage_from_can(&cf) {
                    max_v = max_v.max(v);
                }
            }
            Ok(Frame::Serial(_)) => {}
            Err(HwError::Timeout) => {}
            Err(e) => return Err(e),
        }
    }
    if max_v.is_finite() {
        Ok(Some(max_v))
    } else {
        Ok(None)
    }
}

fn read_ecu_identity(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: Option<u8>,
) -> Result<EcuIdentity, HwError> {
    let vin = read_did_ascii(adapter, cfg, target_sa, DID_VIN, "id::vin")?;
    let sw = read_did_ascii(adapter, cfg, target_sa, DID_SW_VERSION, "id::sw")?;
    let hw = read_did_ascii(adapter, cfg, target_sa, DID_HW_VERSION, "id::hw")?;
    let cal = read_did_ascii(adapter, cfg, target_sa, DID_CALIBRATION_ID, "id::cal")?;
    let serial = read_did_ascii(adapter, cfg, target_sa, DID_ECU_SERIAL, "id::serial")?;
    let fingerprint = read_did_ascii(
        adapter,
        cfg,
        target_sa,
        DID_FINGERPRINT,
        "id::fingerprint",
    )
    .ok();

    Ok(EcuIdentity {
        vin,
        sw_version: sw,
        hw_version: hw,
        calibration_id: cal,
        ecu_serial: serial,
        fingerprint,
    })
}

fn run_flash_preflight(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: Option<u8>,
) -> Result<FlashPreflight, HwError> {
    let supply_voltage = match sample_supply_voltage_from_bus(adapter, Duration::from_millis(500))? {
        Some(v) => v,
        None => read_did_battery_voltage(adapter, cfg, target_sa)?,
    };
    if supply_voltage < MIN_FLASH_SUPPLY_V {
        return Err(HwError::Unknown(format!(
            "preflight failed: supply voltage {:.2}V below minimum {:.2}V",
            supply_voltage, MIN_FLASH_SUPPLY_V
        )));
    }

    let identity = read_ecu_identity(adapter, cfg, target_sa)?;
    if !cfg.allow_untrusted_ecu && identity.fingerprint.is_none() {
        return Err(HwError::Unknown(
            "preflight failed: ECU fingerprint DID 0xF184 not available; use trusted ECU profile or --allow-untrusted-ecu only in controlled bench".to_string(),
        ));
    }

    Ok(FlashPreflight {
        supply_voltage_v: supply_voltage,
        identity,
    })
}

pub fn live_read_ecm_identity(cfg: &HwConfig, target_sa: Option<u8>) -> Result<EcuIdentity, HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "identity read requires --hw-mode=live".to_string(),
        ));
    }
    check_live_channel_policy(cfg)?;

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;
    let identity = read_ecu_identity(&mut *adapter, cfg, target_sa)?;
    adapter.close()?;
    Ok(identity)
}

pub fn live_preflight_ecm(
    cfg: &HwConfig,
    target_sa: Option<u8>,
) -> Result<FlashPreflightReport, HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "preflight requires --hw-mode=live".to_string(),
        ));
    }
    check_live_channel_policy(cfg)?;

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;
    let preflight = run_flash_preflight(&mut *adapter, cfg, target_sa)?;
    adapter.close()?;

    Ok(FlashPreflightReport {
        supply_voltage_v: preflight.supply_voltage_v,
        trusted_fingerprint: preflight.identity.fingerprint.is_some(),
        identity: preflight.identity,
    })
}

fn j1939_uds_req_id(dst_sa: u8) -> u32 {
    0x18DA0000 | ((dst_sa as u32) << 8) | 0xF9
}

fn is_uds_response_id(target_sa: Option<u8>, id: u32) -> bool {
    let pf = ((id >> 16) & 0xFF) as u8;
    let ps = ((id >> 8) & 0xFF) as u8;
    let sa = (id & 0xFF) as u8;
    pf == 0xDA && ps == 0xF9 && target_sa.is_none_or(|t| t == sa)
}

fn can_payload(cf: &CanFrame) -> &[u8] {
    &cf.data[..cf.len.min(8)]
}

fn send_frame_retry(
    adapter: &mut dyn super::hw::HardwareInterface,
    frame: Frame,
    retry_count: u8,
    backoff_ms: u64,
) -> Result<(), HwError> {
    let max_tries = retry_count.saturating_add(1);
    let mut try_idx = 0u8;
    loop {
        match adapter.send_frame(frame.clone()) {
            Ok(()) => return Ok(()),
            Err(HwError::Timeout)
            | Err(HwError::RateLimited)
            | Err(HwError::TransceiverError)
            | Err(HwError::BusOff)
                if try_idx + 1 < max_tries =>
            {
                try_idx = try_idx.saturating_add(1);
                thread::sleep(Duration::from_millis(backoff_ms.max(1)));
            }
            Err(e) => return Err(e),
        }
    }
}

fn send_can_sf(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    dst_sa: u8,
    payload: &[u8],
) -> Result<(), HwError> {
    if payload.len() > 7 {
        return Err(HwError::Unknown(
            "CAN single-frame payload exceeds 7 bytes".to_string(),
        ));
    }

    let mut data = [0u8; 8];
    data[0] = payload.len() as u8;
    if !payload.is_empty() {
        data[1..(1 + payload.len())].copy_from_slice(payload);
    }

    let frame = Frame::Can(CanFrame {
        id: j1939_uds_req_id(dst_sa),
        dlc: 8,
        data,
        len: 8,
        timestamp_ms: Some(now_ms()),
    });
    send_frame_retry(
        adapter,
        frame,
        cfg.uds_retry_count,
        cfg.write_retry_backoff_ms,
    )
}

fn wait_for_fc(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: u8,
    timeout: Duration,
) -> Result<(u8, u64), HwError> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match adapter.read_frame() {
            Ok(Frame::Can(cf)) => {
                if !is_uds_response_id(Some(target_sa), cf.id) {
                    continue;
                }
                let p = can_payload(&cf);
                if p.len() < 3 {
                    continue;
                }
                let pci_type = (p[0] >> 4) & 0x0F;
                if pci_type != 0x3 {
                    continue;
                }
                let fs = p[0] & 0x0F;
                if fs == 0x1 {
                    return Err(HwError::RateLimited);
                }
                if fs == 0x2 {
                    return Err(HwError::Timeout);
                }
                let bs = p[1];
                let st_min = p[2] as u64;
                return Ok((bs, st_min.max(cfg.uds_st_min_ms)));
            }
            Ok(_) => {}
            Err(HwError::Timeout) => {}
            Err(e) => return Err(e),
        }
    }
    Err(HwError::Timeout)
}

fn send_can_multiframe(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    dst_sa: u8,
    payload: &[u8],
) -> Result<(), HwError> {
    if payload.len() <= 7 {
        return send_can_sf(adapter, cfg, dst_sa, payload);
    }
    if payload.len() > 4095 {
        return Err(HwError::Unknown(
            "CAN ISO-TP payload too large (>4095 bytes)".to_string(),
        ));
    }

    let total_len = payload.len();
    let mut ff = [0u8; 8];
    ff[0] = 0x10 | (((total_len >> 8) & 0x0F) as u8);
    ff[1] = (total_len & 0xFF) as u8;
    ff[2..8].copy_from_slice(&payload[..6]);

    let ff_frame = Frame::Can(CanFrame {
        id: j1939_uds_req_id(dst_sa),
        dlc: 8,
        data: ff,
        len: 8,
        timestamp_ms: Some(now_ms()),
    });
    send_frame_retry(
        adapter,
        ff_frame,
        cfg.uds_retry_count,
        cfg.write_retry_backoff_ms,
    )?;

    let (mut bs, mut st_min) = wait_for_fc(
        adapter,
        cfg,
        dst_sa,
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    if cfg.uds_block_size > 0 {
        bs = cfg.uds_block_size;
    }
    st_min = st_min.max(cfg.uds_st_min_ms);

    let mut idx = 6usize;
    let mut sn = 1u8;
    let mut sent_in_block = 0u8;

    while idx < payload.len() {
        let end = (idx + 7).min(payload.len());
        let mut data = [0u8; 8];
        data[0] = 0x20 | (sn & 0x0F);
        let count = end - idx;
        data[1..(1 + count)].copy_from_slice(&payload[idx..end]);

        let cf = Frame::Can(CanFrame {
            id: j1939_uds_req_id(dst_sa),
            dlc: 8,
            data,
            len: 8,
            timestamp_ms: Some(now_ms()),
        });
        send_frame_retry(adapter, cf, cfg.uds_retry_count, cfg.write_retry_backoff_ms)?;

        idx = end;
        sn = (sn + 1) & 0x0F;
        sent_in_block = sent_in_block.saturating_add(1);

        if st_min > 0 {
            thread::sleep(Duration::from_millis(st_min));
        }

        if bs > 0 && sent_in_block >= bs && idx < payload.len() {
            let (new_bs, new_st) = wait_for_fc(
                adapter,
                cfg,
                dst_sa,
                Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
            )?;
            sent_in_block = 0;
            if cfg.uds_block_size == 0 {
                bs = new_bs;
            }
            st_min = new_st.max(cfg.uds_st_min_ms);
        }
    }

    Ok(())
}

fn send_fc_cts(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: u8,
) -> Result<(), HwError> {
    let mut data = [0u8; 8];
    data[0] = 0x30;
    data[1] = cfg.uds_block_size;
    data[2] = cfg.uds_st_min_ms.min(0x7F) as u8;

    let frame = Frame::Can(CanFrame {
        id: j1939_uds_req_id(target_sa),
        dlc: 8,
        data,
        len: 8,
        timestamp_ms: Some(now_ms()),
    });
    send_frame_retry(
        adapter,
        frame,
        cfg.uds_retry_count,
        cfg.write_retry_backoff_ms,
    )
}

fn recv_can_uds_response(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: Option<u8>,
    timeout: Duration,
) -> Result<Vec<u8>, HwError> {
    let mut deadline = Instant::now() + timeout;
    let mut last_keepalive = Instant::now();

    loop {
        if Instant::now() >= deadline {
            return Err(HwError::Timeout);
        }

        match adapter.read_frame() {
            Ok(Frame::Can(cf)) => {
                if !is_uds_response_id(target_sa, cf.id) {
                    continue;
                }
                let p = can_payload(&cf);
                if p.is_empty() {
                    continue;
                }

                let pci_type = (p[0] >> 4) & 0x0F;
                if pci_type == 0x0 {
                    let l = (p[0] & 0x0F) as usize;
                    if p.len() < 1 + l {
                        continue;
                    }
                    let payload = p[1..(1 + l)].to_vec();
                    if payload.len() >= 3 && payload[0] == 0x7F && payload[2] == 0x78 {
                        deadline = Instant::now()
                            + Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1));
                        continue;
                    }
                    return Ok(payload);
                }

                if pci_type == 0x1 {
                    if p.len() < 2 {
                        continue;
                    }
                    let total_len = (((p[0] as usize) & 0x0F) << 8) | (p[1] as usize);
                    if total_len == 0 {
                        continue;
                    }

                    let src_sa = (cf.id & 0xFF) as u8;
                    send_fc_cts(adapter, cfg, src_sa)?;

                    let mut out = Vec::with_capacity(total_len);
                    out.extend_from_slice(&p[2..p.len().min(8)]);
                    let mut expected_sn = 1u8;

                    while out.len() < total_len {
                        match adapter.read_frame() {
                            Ok(Frame::Can(seg)) => {
                                if !is_uds_response_id(target_sa, seg.id) {
                                    continue;
                                }
                                let sp = can_payload(&seg);
                                if sp.is_empty() {
                                    continue;
                                }
                                let t = (sp[0] >> 4) & 0x0F;
                                if t != 0x2 {
                                    continue;
                                }
                                let sn_cf = sp[0] & 0x0F;
                                if sn_cf != expected_sn {
                                    return Err(HwError::ParseError {
                                        cause: "ISO-TP CF sequence mismatch".to_string(),
                                        raw_data: format!(
                                            "expected_sn={expected_sn}, got_sn={sn_cf}"
                                        ),
                                    });
                                }
                                expected_sn = (expected_sn + 1) & 0x0F;
                                out.extend_from_slice(&sp[1..]);
                            }
                            Ok(_) => {}
                            Err(HwError::Timeout) => {}
                            Err(e) => return Err(e),
                        }

                        if Instant::now() >= deadline {
                            return Err(HwError::Timeout);
                        }
                    }

                    out.truncate(total_len);
                    if out.len() >= 3 && out[0] == 0x7F && out[2] == 0x78 {
                        deadline = Instant::now()
                            + Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1));
                        continue;
                    }
                    return Ok(out);
                }
            }
            Ok(Frame::Serial(_)) => {}
            Err(HwError::Timeout) => {
                if cfg.keep_channels_alive
                    && cfg.can_interface.is_some()
                    && last_keepalive.elapsed()
                        >= Duration::from_millis(cfg.j1939_idle_guard_ms.max(250))
                {
                    if let Some(sa) = target_sa {
                        let _ = send_can_sf(adapter, cfg, sa, &[0x3E, 0x00]);
                    }
                    last_keepalive = Instant::now();
                }
            }
            Err(e) => return Err(e),
        }
    }
}

fn send_uds_and_wait(
    adapter: &mut dyn super::hw::HardwareInterface,
    cfg: &HwConfig,
    target_sa: Option<u8>,
    req: &[u8],
    timeout: Duration,
) -> Result<Vec<u8>, HwError> {
    if cfg.can_interface.is_some() {
        let dst_sa = target_sa.unwrap_or(0x00);
        send_can_multiframe(adapter, cfg, dst_sa, req)?;
        return recv_can_uds_response(adapter, cfg, target_sa, timeout);
    }

    let frame = Frame::Serial(super::hw::SerialFrame {
        bytes: req.to_vec(),
        protocol_hint: Some("uds-raw".into()),
        timestamp_ms: Some(now_ms()),
    });
    send_frame_retry(
        adapter,
        frame,
        cfg.uds_retry_count,
        cfg.write_retry_backoff_ms,
    )?;

    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match adapter.read_frame() {
            Ok(Frame::Serial(sf)) => {
                if sf
                    .protocol_hint
                    .as_deref()
                    .is_some_and(|h| !h.eq_ignore_ascii_case("uds-raw"))
                {
                    continue;
                }
                if sf.bytes.is_empty() {
                    continue;
                }
                if sf.bytes.len() >= 3 && sf.bytes[0] == 0x7F && sf.bytes[2] == 0x78 {
                    continue;
                }
                return Ok(sf.bytes);
            }
            Ok(Frame::Can(_)) => {}
            Err(HwError::Timeout) => {}
            Err(e) => return Err(e),
        }
    }
    Err(HwError::Timeout)
}

fn check_live_channel_policy(cfg: &HwConfig) -> Result<(), HwError> {
    if cfg.ethernet_probe.is_some() && cfg.can_interface.is_none() {
        return Err(HwError::Unknown(
            "ethernet/internet checks require CAN/J1939 active (--can-if) to keep channel continuity"
                .to_string(),
        ));
    }

    if let Some(target) = &cfg.ethernet_probe {
        let mut addrs = target
            .to_socket_addrs()
            .map_err(|e| HwError::Unknown(format!("invalid --eth-probe target: {e}")))?;
        let addr = addrs
            .next()
            .ok_or_else(|| HwError::Unknown("--eth-probe resolved no socket address".to_string()))?;
        TcpStream::connect_timeout(&addr, Duration::from_millis(350)).map_err(|e| {
            HwError::Unknown(format!(
                "ethernet/internet check failed ({target}): {e}; preserving J1939 stability"
            ))
        })?;
    }

    Ok(())
}

pub fn live_clean_ecm(cfg: &HwConfig, target_sa: Option<u8>) -> Result<(), HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "clean requires --hw-mode=live".to_string(),
        ));
    }
    if !cfg.write_effectively_enabled() {
        return Err(HwError::WriteBlockedAllowlist);
    }
    check_live_channel_policy(cfg)?;

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;

    let session_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x10, 0x02],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&session_rsp, 0x10, "clean::session", cfg, target_sa)?;

    let clear_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x14, 0xFF, 0xFF, 0xFF],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&clear_rsp, 0x14, "clean::clear_dtc", cfg, target_sa)?;

    let routine_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x31, 0x01, 0xDF, 0x04],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&routine_rsp, 0x31, "clean::routine", cfg, target_sa)?;

    adapter.close()?;
    Ok(())
}

pub fn live_flash_ecm_firmware(
    cfg: &HwConfig,
    target_sa: Option<u8>,
    firmware: &[u8],
) -> Result<FlashSummary, HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "flash requires --hw-mode=live".to_string(),
        ));
    }
    if firmware.is_empty() {
        return Err(HwError::Unknown("firmware payload is empty".to_string()));
    }
    if !cfg.write_effectively_enabled() {
        return Err(HwError::WriteBlockedAllowlist);
    }
    check_live_channel_policy(cfg)?;
    let flash_start_ms = now_ms();

    append_flash_audit(
        cfg,
        "flash::start",
        FlashAuditStatus::Ok,
        format!("payload_bytes={}", firmware.len()),
        target_sa,
    );

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;
    let firmware_crc = crc32fast::hash(firmware);
    let mut transfer_data_blocks_attempted = 0usize;
    let mut transfer_data_blocks_acked = 0usize;
    let wait_frame_count = 0u32;
    let sequence_error_count = 0u32;
    let flowcontrol_timeout_count = 0u32;

    let preflight = run_flash_preflight(&mut *adapter, cfg, target_sa)?;
    append_flash_audit(
        cfg,
        "flash::preflight",
        FlashAuditStatus::Ok,
        format!(
            "supply_v={:.2}, vin={}, sw={}, hw={}, cal={}, serial={}, fp_present={}",
            preflight.supply_voltage_v,
            preflight.identity.vin,
            preflight.identity.sw_version,
            preflight.identity.hw_version,
            preflight.identity.calibration_id,
            preflight.identity.ecu_serial,
            preflight.identity.fingerprint.is_some()
        ),
        target_sa,
    );

    let default_session_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x10, 0x02],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(
        &default_session_rsp,
        0x10,
        "flash::default_session",
        cfg,
        target_sa,
    )?;

    let seed = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x27, 0x05],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&seed, 0x27, "flash::security_seed", cfg, target_sa)?;
    if seed.len() < 6 {
        append_flash_audit(
            cfg,
            "flash::security_seed",
            FlashAuditStatus::Failed,
            format!("short seed response len={}", seed.len()),
            target_sa,
        );
        adapter.close()?;
        return Err(HwError::Unknown("security seed response invalid".to_string()));
    }
    let seed_u32 = ((seed[2] as u32) << 24)
        | ((seed[3] as u32) << 16)
        | ((seed[4] as u32) << 8)
        | seed[5] as u32;
    let key = seed_u32 ^ 0xDEADBEEF;
    let key_req = [
        0x27,
        0x06,
        ((key >> 24) & 0xFF) as u8,
        ((key >> 16) & 0xFF) as u8,
        ((key >> 8) & 0xFF) as u8,
        (key & 0xFF) as u8,
    ];
    let unlock_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &key_req,
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&unlock_rsp, 0x27, "flash::security_unlock", cfg, target_sa)?;

    let prog_session_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x10, 0x03],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(
        &prog_session_rsp,
        0x10,
        "flash::programming_session",
        cfg,
        target_sa,
    )?;

    let total_len = firmware.len() as u32;
    let req_dl = [
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
    let dl_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &req_dl,
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    ensure_positive_response(&dl_rsp, 0x34, "flash::request_download", cfg, target_sa)?;

    let mut max_block_payload = 127usize;
    if dl_rsp.len() >= 4 && dl_rsp[0] == 0x74 {
        let mbl = ((dl_rsp[2] as u16) << 8) | dl_rsp[3] as u16;
        if mbl > 2 {
            max_block_payload = (mbl as usize).saturating_sub(2).max(1);
        }
    }
    max_block_payload = max_block_payload.min(2048);

    let mut block = 1u8;
    let mut blocks = 0usize;
    for chunk in firmware.chunks(max_block_payload) {
        let mut req = Vec::with_capacity(chunk.len() + 2);
        req.push(0x36);
        req.push(block);
        req.extend_from_slice(chunk);
        transfer_data_blocks_attempted += 1;
        let ack = match send_uds_and_wait(
            &mut *adapter,
            cfg,
            target_sa,
            &req,
            Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
        ) {
            Ok(v) => v,
            Err(HwError::Timeout) => {
                append_flash_audit(
                    cfg,
                    "flash::transfer_data",
                    FlashAuditStatus::Failed,
                    format!("timeout waiting transfer ack block={block}"),
                    target_sa,
                );
                adapter.close()?;
                return Err(HwError::Timeout);
            }
            Err(e) => {
                adapter.close()?;
                return Err(e);
            }
        };
        ensure_positive_response(&ack, 0x36, "flash::transfer_data", cfg, target_sa)?;
        if ack.len() < 2 || ack[1] != block {
            append_flash_audit(
                cfg,
                "flash::transfer_data",
                FlashAuditStatus::Failed,
                format!("ack block mismatch expected={block} got={}", ack.get(1).copied().unwrap_or(0xFF)),
                target_sa,
            );
            adapter.close()?;
            return Err(HwError::Unknown(format!(
                "transfer ack mismatch for block {block}"
            )));
        }
        transfer_data_blocks_acked += 1;
        block = block.wrapping_add(1);
        blocks += 1;
    }

    let exit_req_crc = [
        0x37,
        ((firmware_crc >> 24) & 0xFF) as u8,
        ((firmware_crc >> 16) & 0xFF) as u8,
        ((firmware_crc >> 8) & 0xFF) as u8,
        (firmware_crc & 0xFF) as u8,
    ];
    let exit = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &exit_req_crc,
        Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
    )
    .or_else(|_| {
        send_uds_and_wait(
            &mut *adapter,
            cfg,
            target_sa,
            &[0x37, 0x00],
            Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
        )
    })?;
    ensure_positive_response(&exit, 0x37, "flash::transfer_exit", cfg, target_sa)?;
    if exit.first().copied() != Some(0x77) {
        append_flash_audit(
            cfg,
            "flash::transfer_exit",
            FlashAuditStatus::Failed,
            "transfer exit rejected",
            target_sa,
        );
        adapter.close()?;
        return Err(HwError::Unknown(
            "transfer exit rejected during integrity verification".to_string(),
        ));
    }

    let verify_req = [
        0x31,
        0x01,
        0xF1,
        0x90,
        ((firmware_crc >> 24) & 0xFF) as u8,
        ((firmware_crc >> 16) & 0xFF) as u8,
        ((firmware_crc >> 8) & 0xFF) as u8,
        (firmware_crc & 0xFF) as u8,
    ];
    let verify_rsp = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &verify_req,
        Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
    )?;
    ensure_positive_response(&verify_rsp, 0x31, "flash::verify_routine", cfg, target_sa)?;

    adapter.close()?;

    append_flash_audit(
        cfg,
        "flash::complete",
        FlashAuditStatus::Ok,
        format!("bytes={} blocks={} crc32=0x{firmware_crc:08X}", firmware.len(), blocks),
        target_sa,
    );

    let bridge_diag = read_bridge_diag_since(cfg, flash_start_ms);

    Ok(FlashSummary {
        bytes_sent: firmware.len(),
        blocks_sent: blocks,
        crc32: firmware_crc,
        supply_voltage_v: preflight.supply_voltage_v,
        vin: preflight.identity.vin,
        sw_version: preflight.identity.sw_version,
        hw_version: preflight.identity.hw_version,
        calibration_id: preflight.identity.calibration_id,
        ecu_serial: preflight.identity.ecu_serial,
        transport_diagnostics: FlashTransportDiagnostics {
            security_seed_positive: true,
            security_unlock_positive: true,
            request_download_positive: true,
            transfer_data_blocks_attempted,
            transfer_data_blocks_acked,
            request_transfer_exit_positive: true,
            fc_blocksize_seen: bridge_diag.fc_blocksize_seen.or(Some(cfg.uds_block_size)),
            fc_stmin_seen_ms: bridge_diag.fc_stmin_seen_ms.or(Some(cfg.uds_st_min_ms)),
            wait_frame_count: wait_frame_count.saturating_add(bridge_diag.wait_frame_count),
            sequence_error_count: sequence_error_count
                .saturating_add(bridge_diag.sequence_error_count),
            flowcontrol_timeout_count: flowcontrol_timeout_count
                .saturating_add(bridge_diag.flowcontrol_timeout_count),
        },
    })
}

fn read_bridge_diag_since(cfg: &HwConfig, since_ms: u64) -> BridgeDiagAggregate {
    let path = cfg.log_dir.join("cat_comm_bridge_diag.jsonl");
    let content = match fs::read_to_string(path) {
        Ok(v) => v,
        Err(_) => return BridgeDiagAggregate::default(),
    };

    let mut agg = BridgeDiagAggregate::default();
    for line in content.lines() {
        let Ok(r) = serde_json::from_str::<BridgeDiagLine>(line) else {
            continue;
        };
        if r.ts_ms < since_ms {
            continue;
        }
        agg.wait_frame_count = agg
            .wait_frame_count
            .saturating_add(r.wait_frame_count_delta.unwrap_or(0));
        agg.sequence_error_count = agg
            .sequence_error_count
            .saturating_add(r.sequence_error_count_delta.unwrap_or(0));
        agg.flowcontrol_timeout_count = agg
            .flowcontrol_timeout_count
            .saturating_add(r.flowcontrol_timeout_count_delta.unwrap_or(0));
        if r.fc_blocksize.is_some() {
            agg.fc_blocksize_seen = r.fc_blocksize;
        }
        if r.fc_stmin_ms.is_some() {
            agg.fc_stmin_seen_ms = r.fc_stmin_ms;
        }
    }
    agg
}

#[cfg(test)]
mod tests {
    use super::ensure_positive_response;
    use super::is_uds_response_id;
    use crate::io::hw::HwConfig;

    #[test]
    fn uds_response_id_matches_expected_target() {
        // PF=0xDA, PS=0xF9, SA=0x00
        let id = 0x18DAF900u32;
        assert!(is_uds_response_id(Some(0x00), id));
        assert!(is_uds_response_id(None, id));
    }

    #[test]
    fn uds_response_id_rejects_wrong_pf_ps_or_sa() {
        let wrong_pf = 0x18EAF900u32;
        let wrong_ps = 0x18DA1200u32;
        let wrong_sa = 0x18DAF901u32;

        assert!(!is_uds_response_id(Some(0x00), wrong_pf));
        assert!(!is_uds_response_id(Some(0x00), wrong_ps));
        assert!(!is_uds_response_id(Some(0x00), wrong_sa));
    }

    #[test]
    fn ensure_positive_response_accepts_expected_sid() {
        let cfg = HwConfig::default();
        let rsp = [0x50, 0x02, 0x00, 0x00];
        let result = ensure_positive_response(&rsp, 0x10, "test::session", &cfg, Some(0x00));
        assert!(result.is_ok());
    }

    #[test]
    fn ensure_positive_response_rejects_negative_response_code() {
        let cfg = HwConfig::default();
        let rsp = [0x7F, 0x10, 0x22];
        let result = ensure_positive_response(&rsp, 0x10, "test::session", &cfg, Some(0x00));
        assert!(result.is_err());
        let msg = format!("{}", result.err().expect("error expected"));
        assert!(msg.contains("NRC=0x22"), "unexpected error: {msg}");
    }

    #[test]
    fn ensure_positive_response_rejects_unexpected_sid() {
        let cfg = HwConfig::default();
        let rsp = [0x62, 0xF1, 0x90];
        let result = ensure_positive_response(&rsp, 0x10, "test::session", &cfg, Some(0x00));
        assert!(result.is_err());
        let msg = format!("{}", result.err().expect("error expected"));
        assert!(msg.contains("expected=0x50"), "unexpected error: {msg}");
    }
}
