use std::collections::BTreeSet;
use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::Write;
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use super::ecm_params::{j1939_pgn, merge_snapshot, EcmSnapshot, PGN_EEC1, PGN_EFL_P1, PGN_ET1};
use super::hw::{CanFrame, Frame, HwConfig, HwError, HwMode};
use super::replay::{append_record, frame_to_record, JsonlRecord};

#[derive(Debug, Clone)]
pub struct DetectResult {
    pub source_addresses: Vec<u8>,
}

#[derive(Clone)]
pub struct LiveFeed {
    stop: Arc<AtomicBool>,
    latest_snapshot: Arc<Mutex<EcmSnapshot>>,
    history: Arc<Mutex<VecDeque<SnapshotPoint>>>,
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
        self.latest_snapshot
            .lock()
            .map(|s| s.clone())
            .unwrap_or_default()
    }

    pub fn last_update_ms(&self) -> u64 {
        self.last_update_ms.load(Ordering::Relaxed)
    }

    pub fn frames_total(&self) -> u64 {
        self.frames_total.load(Ordering::Relaxed)
    }

    pub fn recent_points(&self, limit: usize) -> Vec<SnapshotPoint> {
        let Ok(hist) = self.history.lock() else {
            return Vec::new();
        };
        let count = hist.len().min(limit);
        hist.iter().skip(hist.len() - count).cloned().collect()
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

        // Authentication/identification proxy: require at least one response from target or any ECM.
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut got_response = false;
        while Instant::now() < deadline {
            match adapter.read_frame() {
                Ok(Frame::Can(cf)) => {
                    let sa = (cf.id & 0xFF) as u8;
                    if target_sa.is_none() || Some(sa) == target_sa {
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

    adapter.close()?;
    Ok(())
}

pub fn start_retrieve_data(cfg: HwConfig, target_sa: Option<u8>) -> Result<LiveFeed, HwError> {
    if cfg.mode != HwMode::Live {
        return Err(HwError::Unknown(
            "Retrieve requires --hw-mode=live".to_string(),
        ));
    }

    let stop = Arc::new(AtomicBool::new(false));
    let latest_snapshot = Arc::new(Mutex::new(EcmSnapshot::default()));
    let history = Arc::new(Mutex::new(VecDeque::<SnapshotPoint>::new()));
    let last_update_ms = Arc::new(AtomicU64::new(0));
    let frames_total = Arc::new(AtomicU64::new(0));

    let feed = LiveFeed {
        stop: Arc::clone(&stop),
        latest_snapshot: Arc::clone(&latest_snapshot),
        history: Arc::clone(&history),
        last_update_ms: Arc::clone(&last_update_ms),
        frames_total: Arc::clone(&frames_total),
    };

    thread::spawn(move || {
        let mut adapter = match super::hw::open_real_adapter(&cfg) {
            Ok(a) => a,
            Err(e) => {
                eprintln!("[ecm-live] open adapter failed: {e}");
                return;
            }
        };
        if let Err(e) = adapter.init(&cfg) {
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
                    eprintln!("[ecm-live] read error: {e}");
                    thread::sleep(Duration::from_millis(150));
                }
            }
        }

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
}

fn encode_uds_frame(cfg: &HwConfig, target_sa: Option<u8>, req: &[u8]) -> Result<Frame, HwError> {
    if cfg.can_interface.is_some() {
        if req.len() > 7 {
            return Err(HwError::Unknown(
                "UDS single-frame over CAN supports up to 7-byte payload in this workflow"
                    .to_string(),
            ));
        }
        let dst = target_sa.unwrap_or(0x00);
        let mut data = [0u8; 8];
        data[0] = req.len() as u8;
        let len = req.len();
        data[1..=len].copy_from_slice(req);
        let id = 0x18DA0000 | ((dst as u32) << 8) | 0xF9;
        Ok(Frame::Can(CanFrame {
            id,
            dlc: 8,
            data,
            len: 8,
            timestamp_ms: Some(now_ms()),
        }))
    } else {
        Ok(Frame::Serial(super::hw::SerialFrame {
            bytes: req.to_vec(),
            protocol_hint: Some("uds-raw".into()),
            timestamp_ms: Some(now_ms()),
        }))
    }
}

fn decode_uds_response(frame: &Frame) -> Option<Vec<u8>> {
    match frame {
        Frame::Serial(sf) => {
            if sf.bytes.is_empty() {
                None
            } else {
                Some(sf.bytes.clone())
            }
        }
        Frame::Can(cf) => {
            if cf.len == 0 {
                return None;
            }
            let raw = &cf.data[..cf.len.min(8)];
            if raw.is_empty() {
                return None;
            }
            let sf_len = raw[0] as usize;
            if sf_len > 0 && sf_len <= 7 && raw.len() > 1 {
                let payload_len = sf_len.min(raw.len() - 1);
                Some(raw[1..(1 + payload_len)].to_vec())
            } else {
                Some(raw.to_vec())
            }
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
    let frame = encode_uds_frame(cfg, target_sa, req)?;
    adapter.send_frame(frame)?;

    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match adapter.read_frame() {
            Ok(frame) => {
                if let Some(resp) = decode_uds_response(&frame) {
                    return Ok(resp);
                }
            }
            Err(HwError::Timeout) => {}
            Err(e) => return Err(e),
        }
    }
    Err(HwError::Timeout)
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

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;

    let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &[0x10, 0x02], Duration::from_secs(1))?;
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x14, 0xFF, 0xFF, 0xFF],
        Duration::from_secs(1),
    )?;
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x31, 0x01, 0xDF, 0x04],
        Duration::from_secs(1),
    )?;

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

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;

    let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &[0x10, 0x02], Duration::from_secs(1))?;
    let seed = send_uds_and_wait(&mut *adapter, cfg, target_sa, &[0x27, 0x05], Duration::from_secs(1))?;
    if seed.len() < 6 || seed[0] != 0x67 {
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
    let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &key_req, Duration::from_secs(1))?;
    let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &[0x10, 0x03], Duration::from_secs(1))?;

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
    let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &req_dl, Duration::from_secs(1))?;

    let mut block = 1u8;
    let mut blocks = 0usize;
    for chunk in firmware.chunks(5) {
        let mut req = Vec::with_capacity(chunk.len() + 2);
        req.push(0x36);
        req.push(block);
        req.extend_from_slice(chunk);
        let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &req, Duration::from_secs(2))?;
        block = block.wrapping_add(1);
        blocks += 1;
    }

    let _ = send_uds_and_wait(&mut *adapter, cfg, target_sa, &[0x37, 0x00], Duration::from_secs(1))?;
    adapter.close()?;

    Ok(FlashSummary {
        bytes_sent: firmware.len(),
        blocks_sent: blocks,
    })
}
