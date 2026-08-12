use std::collections::BTreeSet;
use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::Write;
use std::net::{TcpStream, ToSocketAddrs};
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
    check_live_channel_policy(&cfg)?;

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
    pub crc32: u32,
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

    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x10, 0x02],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x14, 0xFF, 0xFF, 0xFF],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x31, 0x01, 0xDF, 0x04],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
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
    check_live_channel_policy(cfg)?;

    let mut adapter = super::hw::open_real_adapter(cfg)?;
    adapter.init(cfg)?;
    let firmware_crc = crc32fast::hash(firmware);

    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x10, 0x02],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    let seed = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x27, 0x05],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
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
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &key_req,
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
    )?;
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &[0x10, 0x03],
        Duration::from_millis(cfg.uds_timeout_p2_ms.max(1)),
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
        let ack = send_uds_and_wait(
            &mut *adapter,
            cfg,
            target_sa,
            &req,
            Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
        )?;
        if ack.len() < 2 || ack[0] != 0x76 || ack[1] != block {
            adapter.close()?;
            return Err(HwError::Unknown(format!(
                "transfer ack mismatch for block {block}"
            )));
        }
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
    if exit.first().copied() != Some(0x77) {
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
    let _ = send_uds_and_wait(
        &mut *adapter,
        cfg,
        target_sa,
        &verify_req,
        Duration::from_millis(cfg.uds_timeout_p2star_ms.max(1)),
    );

    adapter.close()?;

    Ok(FlashSummary {
        bytes_sent: firmware.len(),
        blocks_sent: blocks,
        crc32: firmware_crc,
    })
}
