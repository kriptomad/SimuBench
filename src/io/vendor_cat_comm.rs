use std::fs::{self, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, Command, Stdio};

use serde::{Deserialize, Serialize};

use super::hw::{Frame, HardwareInterface, HwConfig, HwError};

const DEFAULT_TEMPLATE_BRIDGE_EXE: &str = "cat_comm_bridge.exe";

#[derive(Debug)]
pub struct CatCommAdapter {
    connected: bool,
    child: Child,
    child_stdin: ChildStdin,
    child_stdout: BufReader<std::process::ChildStdout>,
    timeout_ms: u64,
    bridge_info: CatCommBridgeInfo,
    diag_log_path: PathBuf,
}

#[derive(Debug, Clone)]
struct CatCommBridgeInfo {
    protocol_version: Option<String>,
    capabilities: Vec<String>,
}

impl CatCommBridgeInfo {
    fn supports(&self, cap: &str) -> bool {
        self.capabilities.iter().any(|c| c.eq_ignore_ascii_case(cap))
    }
}

#[derive(Debug, Serialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
enum BridgeRequest {
    Init {
        dry_run: bool,
        enable_write: bool,
        can_interface: Option<String>,
        serial_port: Option<String>,
        template_dir: Option<String>,
    },
    Read {
        timeout_ms: u64,
    },
    TryRead,
    Send {
        frame: Frame,
    },
    Close,
    Ping,
}

#[derive(Debug, Deserialize)]
struct BridgeResponse {
    ok: bool,
    frame: Option<Frame>,
    error: Option<String>,
    kind: Option<String>,
    protocol_version: Option<String>,
    capabilities: Option<Vec<String>>,
    diag: Option<BridgeDiagPayload>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
struct BridgeDiagPayload {
    wait_frame_count_delta: Option<u32>,
    sequence_error_count_delta: Option<u32>,
    flowcontrol_timeout_count_delta: Option<u32>,
    fc_blocksize: Option<u8>,
    fc_stmin_ms: Option<u64>,
}

#[derive(Debug, Serialize)]
struct BridgeDiagRecord<'a> {
    ts_ms: u64,
    source: &'a str,
    wait_frame_count_delta: u32,
    sequence_error_count_delta: u32,
    flowcontrol_timeout_count_delta: u32,
    fc_blocksize: Option<u8>,
    fc_stmin_ms: Option<u64>,
}

impl CatCommAdapter {
    pub fn open(cfg: &HwConfig) -> Result<Self, HwError> {
        let _vendor = cfg
            .vendor_name
            .as_deref()
            .unwrap_or("cat_comm")
            .to_ascii_lowercase();

        let bridge_exe = resolve_bridge_exe(cfg)?;
        let mut child = Command::new(&bridge_exe)
            .arg("--mode=stdio-jsonl")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| {
                HwError::Unknown(format!(
                    "failed to start Cat Comm bridge at {}: {e}",
                    bridge_exe.display()
                ))
            })?;

        let child_stdin = child.stdin.take().ok_or_else(|| {
            HwError::Unknown("failed to capture Cat Comm bridge stdin".to_string())
        })?;
        let child_stdout = child.stdout.take().ok_or_else(|| {
            HwError::Unknown("failed to capture Cat Comm bridge stdout".to_string())
        })?;

        let mut adapter = Self {
            connected: false,
            child,
            child_stdin,
            child_stdout: BufReader::new(child_stdout),
            timeout_ms: cfg.vendor_bridge_timeout_ms.max(100),
            bridge_info: CatCommBridgeInfo {
                protocol_version: None,
                capabilities: Vec::new(),
            },
            diag_log_path: cfg.log_dir.join("cat_comm_bridge_diag.jsonl"),
        };
        fs::create_dir_all(&cfg.log_dir)
            .map_err(|e| HwError::Unknown(format!("create bridge log dir failed: {e}")))?;

        let init_req = BridgeRequest::Init {
            dry_run: cfg.dry_run,
            enable_write: cfg.write_effectively_enabled(),
            can_interface: cfg.can_interface.clone(),
            serial_port: cfg.serial_port.clone(),
            template_dir: cfg
                .vendor_template_dir
                .as_ref()
                .map(|p| p.display().to_string()),
        };
        let init_rsp = adapter.send_bridge_request(&init_req)?;
        if !init_rsp.ok {
            return Err(map_bridge_error(
                init_rsp.kind,
                init_rsp.error.unwrap_or_else(|| "bridge init failed".to_string()),
            ));
        }
        adapter.bridge_info = bridge_info_from_response(&init_rsp);
        ensure_bridge_compatibility(cfg, &adapter.bridge_info)?;

        Ok(adapter)
    }

    fn ensure_connected(&self) -> Result<(), HwError> {
        if self.connected {
            Ok(())
        } else {
            Err(HwError::Unknown(
                "Cat Comm bridge is not initialized".to_string(),
            ))
        }
    }

    fn send_bridge_request(&mut self, req: &BridgeRequest) -> Result<BridgeResponse, HwError> {
        let line = serde_json::to_string(req)
            .map_err(|e| HwError::Unknown(format!("bridge request encode failed: {e}")))?;
        self.child_stdin
            .write_all(line.as_bytes())
            .map_err(|e| HwError::Unknown(format!("bridge stdin write failed: {e}")))?;
        self.child_stdin
            .write_all(b"\n")
            .map_err(|e| HwError::Unknown(format!("bridge stdin newline failed: {e}")))?;
        self.child_stdin
            .flush()
            .map_err(|e| HwError::Unknown(format!("bridge stdin flush failed: {e}")))?;

        let rsp = self.read_bridge_response()?;
        self.append_diag_record(&rsp);
        Ok(rsp)
    }

    fn read_bridge_response(&mut self) -> Result<BridgeResponse, HwError> {
        let mut buf = String::new();
        self.child_stdout
            .read_line(&mut buf)
            .map_err(|e| HwError::Unknown(format!("bridge stdout read failed: {e}")))?;
        if buf.trim().is_empty() {
            return Err(HwError::Unknown(
                "Cat Comm bridge closed stream or returned empty response".to_string(),
            ));
        }

        serde_json::from_str::<BridgeResponse>(buf.trim()).map_err(|e| {
            HwError::ParseError {
                cause: format!("bridge response parse failed: {e}"),
                raw_data: buf,
            }
        })
    }

    fn append_diag_record(&self, rsp: &BridgeResponse) {
        let Some(diag) = &rsp.diag else {
            return;
        };

        let rec = BridgeDiagRecord {
            ts_ms: now_ms(),
            source: "vendor_cat_comm",
            wait_frame_count_delta: diag.wait_frame_count_delta.unwrap_or(0),
            sequence_error_count_delta: diag.sequence_error_count_delta.unwrap_or(0),
            flowcontrol_timeout_count_delta: diag.flowcontrol_timeout_count_delta.unwrap_or(0),
            fc_blocksize: diag.fc_blocksize,
            fc_stmin_ms: diag.fc_stmin_ms,
        };

        let Ok(mut f) = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&self.diag_log_path)
        else {
            return;
        };

        let line = match serde_json::to_string(&rec) {
            Ok(v) => v,
            Err(_) => return,
        };
        let _ = writeln!(f, "{line}");
    }
}

impl HardwareInterface for CatCommAdapter {
    fn init(&mut self, _config: &HwConfig) -> Result<(), HwError> {
        let rsp = self.send_bridge_request(&BridgeRequest::Ping)?;
        if rsp.ok {
            self.bridge_info = bridge_info_from_response(&rsp);
            self.connected = true;
            return Ok(());
        }
        Err(map_bridge_error(
            rsp.kind,
            rsp.error.unwrap_or_else(|| "bridge ping failed".to_string()),
        ))
    }

    fn read_frame(&mut self) -> Result<Frame, HwError> {
        self.ensure_connected()?;
        let rsp = self.send_bridge_request(&BridgeRequest::Read {
            timeout_ms: self.timeout_ms,
        })?;
        if !rsp.ok {
            return Err(map_bridge_error(
                rsp.kind,
                rsp.error.unwrap_or_else(|| "bridge read failed".to_string()),
            ));
        }
        rsp.frame
            .ok_or_else(|| HwError::Unknown("bridge read returned no frame".to_string()))
    }

    fn try_read_frame(&mut self) -> Result<Option<Frame>, HwError> {
        self.ensure_connected()?;
        let rsp = self.send_bridge_request(&BridgeRequest::TryRead)?;
        if !rsp.ok {
            return Err(map_bridge_error(
                rsp.kind,
                rsp.error.unwrap_or_else(|| "bridge try_read failed".to_string()),
            ));
        }
        Ok(rsp.frame)
    }

    fn send_frame(&mut self, frame: Frame) -> Result<(), HwError> {
        self.ensure_connected()?;
        let rsp = self.send_bridge_request(&BridgeRequest::Send { frame })?;
        if rsp.ok {
            Ok(())
        } else {
            Err(map_bridge_error(
                rsp.kind,
                rsp.error.unwrap_or_else(|| "bridge send failed".to_string()),
            ))
        }
    }

    fn close(&mut self) -> Result<(), HwError> {
        let _ = self.send_bridge_request(&BridgeRequest::Close);
        self.connected = false;

        if let Err(e) = self.child.kill() {
            let already_exited = self
                .child
                .try_wait()
                .ok()
                .flatten()
                .is_some();
            if !already_exited {
                return Err(HwError::Unknown(format!("failed to stop Cat Comm bridge: {e}")));
            }
        }
        let _ = self.child.wait();
        Ok(())
    }

    fn adapter_info(&self) -> Option<String> {
        Some(format!(
            "bridge_protocol={:?} capabilities={}",
            self.bridge_info.protocol_version,
            if self.bridge_info.capabilities.is_empty() {
                "none".to_string()
            } else {
                self.bridge_info.capabilities.join(",")
            }
        ))
    }
}

fn resolve_bridge_exe(cfg: &HwConfig) -> Result<PathBuf, HwError> {
    if let Some(explicit) = &cfg.vendor_bridge_exe {
        if explicit.exists() {
            return Ok(explicit.clone());
        }
        return Err(HwError::PortNotFound(format!(
            "vendor bridge executable not found: {}",
            explicit.display()
        )));
    }

    if let Some(template_dir) = &cfg.vendor_template_dir {
        let candidate = template_dir.join(DEFAULT_TEMPLATE_BRIDGE_EXE);
        if candidate.exists() {
            return Ok(candidate);
        }
        return Err(HwError::PortNotFound(format!(
            "template bridge not found at {}. Upload your template bridge executable as {}",
            candidate.display(),
            DEFAULT_TEMPLATE_BRIDGE_EXE
        )));
    }

    Err(HwError::PortNotFound(
        "Cat Comm bridge not configured. Set --vendor-bridge-exe=<path> or --vendor-template-dir=<dir>"
            .to_string(),
    ))
}

fn map_bridge_error(kind: Option<String>, msg: String) -> HwError {
    match kind
        .as_deref()
        .unwrap_or_default()
        .to_ascii_lowercase()
        .as_str()
    {
        "timeout" => HwError::Timeout,
        "rate_limited" => HwError::RateLimited,
        "bus_off" => HwError::BusOff,
        "transceiver" => HwError::TransceiverError,
        "permission" => HwError::PermissionDenied(msg),
        "write_blocked" => HwError::WriteBlockedAllowlist,
        "parse" => HwError::ParseError {
            cause: "bridge parse error".to_string(),
            raw_data: msg,
        },
        _ => HwError::Unknown(msg),
    }
}

fn bridge_info_from_response(rsp: &BridgeResponse) -> CatCommBridgeInfo {
    CatCommBridgeInfo {
        protocol_version: rsp.protocol_version.clone(),
        capabilities: rsp.capabilities.clone().unwrap_or_default(),
    }
}

fn ensure_bridge_compatibility(cfg: &HwConfig, info: &CatCommBridgeInfo) -> Result<(), HwError> {
    if let Some(ver) = &info.protocol_version {
        let major = ver
            .split('.')
            .next()
            .and_then(|s| s.parse::<u32>().ok())
            .unwrap_or(0);
        if major > 1 {
            return Err(HwError::Unknown(format!(
                "unsupported Cat Comm bridge protocol version {ver}; expected major version 1"
            )));
        }
    }

    // Capability checks are strict only when the bridge explicitly reports capabilities.
    if !info.capabilities.is_empty() {
        if cfg.write_effectively_enabled() && !info.supports("write") {
            return Err(HwError::PermissionDenied(
                "bridge does not advertise write capability".to_string(),
            ));
        }
        if cfg.can_interface.is_some() && !info.supports("can") {
            return Err(HwError::Unknown(
                "bridge does not advertise CAN capability for requested --can-if path"
                    .to_string(),
            ));
        }
        if cfg.serial_port.is_some() && !info.supports("serial") {
            return Err(HwError::Unknown(
                "bridge does not advertise serial capability for requested --serial-port path"
                    .to_string(),
            ));
        }
    }

    Ok(())
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
