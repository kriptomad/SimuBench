use std::fs::{self, File, OpenOptions};
use std::io::Write;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use flate2::write::GzEncoder;
use flate2::Compression;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum HwMode {
    Sim,
    Live,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CanFrame {
    pub id: u32,
    pub dlc: u8,
    pub data: [u8; 8],
    pub len: usize,
    pub timestamp_ms: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SerialFrame {
    pub bytes: Vec<u8>,
    pub protocol_hint: Option<String>,
    pub timestamp_ms: Option<u64>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum Frame {
    Can(CanFrame),
    Serial(SerialFrame),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct HwConfig {
    pub mode: HwMode,
    pub serial_port: Option<String>,
    pub serial_baud: u32,
    pub serial_parity: Option<String>,
    pub serial_stopbits: u8,
    pub can_interface: Option<String>,
    pub enable_write: bool,
    pub allowlist_path: Option<PathBuf>,
    pub rate_limit_global_per_sec: u32,
    pub rate_limit_per_id_per_sec: u32,
    pub write_retry_count: u8,
    pub write_retry_backoff_ms: u64,
    pub reconnect_backoff_base_ms: u64,
    pub reconnect_backoff_max_ms: u64,
    pub parser_max_frame_size: usize,
    pub dry_run: bool,
    pub use_isolation_hw: bool,
    pub log_dir: PathBuf,
    pub metrics_enabled: bool,
    pub emergency_stop_hook: Option<String>,
    pub allow_untrusted_ecu: bool,
    pub noninteractive_approved: bool,
}

impl Default for HwConfig {
    fn default() -> Self {
        Self {
            mode: HwMode::Sim,
            serial_port: None,
            serial_baud: 115_200,
            serial_parity: Some("None".to_string()),
            serial_stopbits: 1,
            can_interface: None,
            enable_write: false,
            allowlist_path: None,
            rate_limit_global_per_sec: 100,
            rate_limit_per_id_per_sec: 10,
            write_retry_count: 3,
            write_retry_backoff_ms: 200,
            reconnect_backoff_base_ms: 500,
            reconnect_backoff_max_ms: 30_000,
            parser_max_frame_size: 4096,
            dry_run: false,
            use_isolation_hw: false,
            log_dir: PathBuf::from("./artifacts/hw_logs"),
            metrics_enabled: false,
            emergency_stop_hook: None,
            allow_untrusted_ecu: false,
            noninteractive_approved: false,
        }
    }
}

impl HwConfig {
    pub fn from_cli_args(args: &[String]) -> Result<Self, HwError> {
        let mut cfg = HwConfig::default();
        for a in args {
            if let Some(v) = a.strip_prefix("--hw-mode=") {
                cfg.mode = match v.to_ascii_lowercase().as_str() {
                    "sim" => HwMode::Sim,
                    "live" => HwMode::Live,
                    _ => return Err(HwError::Unknown(format!("invalid --hw-mode value: {v}"))),
                };
            } else if a == "--dry-run" {
                cfg.dry_run = true;
            } else if a == "--enable-write" {
                cfg.enable_write = true;
            } else if a == "--noninteractive-approved" {
                cfg.noninteractive_approved = true;
            } else if let Some(v) = a.strip_prefix("--allowlist=") {
                cfg.allowlist_path = Some(PathBuf::from(v));
            } else if let Some(v) = a.strip_prefix("--log-dir=") {
                cfg.log_dir = PathBuf::from(v);
            }
        }
        Ok(cfg)
    }

    pub fn write_effectively_enabled(&self) -> bool {
        self.enable_write
            && self.allowlist_path.is_some()
            && self.noninteractive_approved
            && !self.dry_run
    }
}

#[derive(Debug, Error)]
pub enum HwError {
    #[error("port not found: {0}")]
    PortNotFound(String),
    #[error("permission denied: {0}")]
    PermissionDenied(String),
    #[error("parse error: {cause}; raw_data={raw_data}")]
    ParseError { cause: String, raw_data: String },
    #[error("write blocked by allowlist")]
    WriteBlockedAllowlist,
    #[error("rate limited")]
    RateLimited,
    #[error("bus off")]
    BusOff,
    #[error("transceiver error")]
    TransceiverError,
    #[error("checksum error")]
    ChecksumError,
    #[error("timeout")]
    Timeout,
    #[error("unknown: {0}")]
    Unknown(String),
}

pub trait HardwareInterface {
    fn init(&mut self, config: &HwConfig) -> Result<(), HwError>;
    fn read_frame(&mut self) -> Result<Frame, HwError>;
    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError>;
    fn send_frame(&mut self, frame: Frame) -> Result<(), HwError>;
    fn close(&mut self) -> Result<(), HwError>;
}

#[derive(Debug, Serialize)]
struct StartupAudit<'a> {
    timestamp_ms: u64,
    component: &'a str,
    event: &'a str,
    level: &'a str,
    direction: &'a str,
    reason: &'a str,
    mode: &'a str,
    dry_run: bool,
    enable_write: bool,
    allowlist_present: bool,
    write_effectively_enabled: bool,
}

pub fn write_listen_only_startup_log(cfg: &HwConfig) -> Result<PathBuf, HwError> {
    fs::create_dir_all(&cfg.log_dir)
        .map_err(|e| HwError::Unknown(format!("create log dir failed: {e}")))?;

    let ts = now_ms();
    let session = format!("session-{ts}");
    let path = cfg.log_dir.join(format!("{session}.jsonl.gz"));

    let file = File::create(&path)
        .map_err(|e| HwError::Unknown(format!("create log file failed: {e}")))?;
    let mut gz = GzEncoder::new(file, Compression::default());

    let mode = match cfg.mode {
        HwMode::Sim => "sim",
        HwMode::Live => "live",
    };

    let audit = StartupAudit {
        timestamp_ms: ts,
        component: "io::hw",
        event: "startup_policy",
        level: "info",
        direction: "blocked",
        reason: "safe_by_default_listen_only",
        mode,
        dry_run: cfg.dry_run,
        enable_write: cfg.enable_write,
        allowlist_present: cfg.allowlist_path.is_some(),
        write_effectively_enabled: cfg.write_effectively_enabled(),
    };

    let line = serde_json::to_string(&audit)
        .map_err(|e| HwError::Unknown(format!("serialize audit failed: {e}")))?;
    writeln!(gz, "{line}").map_err(|e| HwError::Unknown(format!("write audit failed: {e}")))?;
    gz.finish()
        .map_err(|e| HwError::Unknown(format!("finalize gzip failed: {e}")))?;

    if cfg.write_effectively_enabled() {
        let idx_path = cfg.log_dir.join("write_enabled_sessions.log");
        let mut idx = OpenOptions::new()
            .create(true)
            .append(true)
            .open(idx_path)
            .map_err(|e| HwError::Unknown(format!("open write-enabled index failed: {e}")))?;
        writeln!(idx, "{session}")
            .map_err(|e| HwError::Unknown(format!("append write-enabled index failed: {e}")))?;
    }

    Ok(path)
}

fn now_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
