use std::collections::VecDeque;
use std::io::{self, BufRead, Write};

use auto_breaking::io::hw::{Frame, SerialFrame};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize)]
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

#[derive(Debug, Serialize)]
struct BridgeDiagPayload {
    wait_frame_count_delta: u32,
    sequence_error_count_delta: u32,
    flowcontrol_timeout_count_delta: u32,
    fc_blocksize: u8,
    fc_stmin_ms: u64,
}

#[derive(Debug, Serialize)]
struct BridgeResponse {
    ok: bool,
    frame: Option<Frame>,
    error: Option<String>,
    kind: Option<String>,
    protocol_version: Option<String>,
    capabilities: Option<Vec<String>>,
    diag: Option<BridgeDiagPayload>,
}

#[derive(Debug)]
struct BridgeState {
    rx_queue: VecDeque<Frame>,
    dry_run: bool,
    enable_write: bool,
    initialized: bool,
    negotiated_stmin_ms: u64,
    negotiated_blocksize: u8,
}

impl Default for BridgeState {
    fn default() -> Self {
        Self {
            rx_queue: VecDeque::new(),
            dry_run: true,
            enable_write: false,
            initialized: false,
            negotiated_stmin_ms: 5,
            negotiated_blocksize: 8,
        }
    }
}

impl BridgeState {
    fn capabilities(&self) -> Vec<String> {
        vec![
            "serial".to_string(),
            "can".to_string(),
            "write".to_string(),
            "uds_flash".to_string(),
        ]
    }

    fn make_ok(&self) -> BridgeResponse {
        BridgeResponse {
            ok: true,
            frame: None,
            error: None,
            kind: None,
            protocol_version: Some("1.0".to_string()),
            capabilities: Some(self.capabilities()),
            diag: None,
        }
    }

    fn push_serial(&mut self, bytes: Vec<u8>, hint: &str) {
        self.rx_queue.push_back(Frame::Serial(SerialFrame {
            bytes,
            protocol_hint: Some(hint.to_string()),
            timestamp_ms: Some(now_ms()),
        }));
    }

    fn handle_uds_serial_request(&mut self, req: &[u8]) {
        if req.is_empty() {
            return;
        }

        match req[0] {
            0x10 => {
                let sub = req.get(1).copied().unwrap_or(0x01);
                self.push_serial(vec![0x50, sub, 0x00, 0x32, 0x01, 0xF4], "uds-raw");
            }
            0x22 => {
                if req.len() < 3 {
                    self.push_serial(vec![0x7F, 0x22, 0x13], "uds-raw");
                    return;
                }
                let did = ((req[1] as u16) << 8) | req[2] as u16;
                match did {
                    0xF190 => self.did_ascii(did, b"CATVIN1234567890"),
                    0xF189 => self.did_ascii(did, b"SW-1.0.0"),
                    0xF191 => self.did_ascii(did, b"HW-1.0.0"),
                    0xF180 => self.did_ascii(did, b"CAL-001"),
                    0xF18C => self.did_ascii(did, b"ECU-SN-0001"),
                    0xF184 => self.did_ascii(did, b"FP-TRUSTED-01"),
                    0xDD0E => {
                        // 12.5V with factor 0.05 => raw 250 (0x00FA)
                        self.push_serial(vec![0x62, 0xDD, 0x0E, 0x00, 0xFA], "uds-raw");
                    }
                    _ => self.push_serial(vec![0x7F, 0x22, 0x31], "uds-raw"),
                }
            }
            0x27 => {
                let sub = req.get(1).copied().unwrap_or(0x00);
                match sub {
                    0x05 => self.push_serial(vec![0x67, 0x05, 0x12, 0x34, 0x56, 0x78], "uds-raw"),
                    0x06 => self.push_serial(vec![0x67, 0x06], "uds-raw"),
                    _ => self.push_serial(vec![0x7F, 0x27, 0x12], "uds-raw"),
                }
            }
            0x34 => {
                // Positive response with max block length hint.
                self.push_serial(vec![0x74, 0x20, 0x00, 0x40], "uds-raw");
            }
            0x36 => {
                let seq = req.get(1).copied().unwrap_or(0x00);
                self.push_serial(vec![0x76, seq], "uds-raw");
            }
            0x37 => {
                self.push_serial(vec![0x77, 0x00], "uds-raw");
            }
            0x31 => {
                let sub = req.get(1).copied().unwrap_or(0x01);
                let h = req.get(2).copied().unwrap_or(0x00);
                let l = req.get(3).copied().unwrap_or(0x00);
                self.push_serial(vec![0x71, sub, h, l], "uds-raw");
            }
            0x14 => {
                self.push_serial(vec![0x54], "uds-raw");
            }
            0x3E => {
                self.push_serial(vec![0x7E, 0x00], "uds-raw");
            }
            _ => {
                self.push_serial(vec![0x7F, req[0], 0x11], "uds-raw");
            }
        }
    }

    fn did_ascii(&mut self, did: u16, payload: &[u8]) {
        let mut rsp = vec![0x62, (did >> 8) as u8, (did & 0xFF) as u8];
        rsp.extend_from_slice(payload);
        self.push_serial(rsp, "uds-raw");
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if !args.iter().any(|a| a == "--mode=stdio-jsonl") {
        eprintln!("cat_comm_bridge: use --mode=stdio-jsonl");
        std::process::exit(2);
    }

    let stdin = io::stdin();
    let mut stdout = io::stdout();
    let mut state = BridgeState::default();

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(v) => v,
            Err(e) => {
                write_response(
                    &mut stdout,
                    BridgeResponse {
                        ok: false,
                        frame: None,
                        error: Some(format!("stdin read failed: {e}")),
                        kind: Some("parse".to_string()),
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: None,
                    },
                );
                continue;
            }
        };

        let req = match serde_json::from_str::<BridgeRequest>(line.trim()) {
            Ok(v) => v,
            Err(e) => {
                write_response(
                    &mut stdout,
                    BridgeResponse {
                        ok: false,
                        frame: None,
                        error: Some(format!("invalid request json: {e}")),
                        kind: Some("parse".to_string()),
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: None,
                    },
                );
                continue;
            }
        };

        let rsp = match req {
            BridgeRequest::Init {
                dry_run,
                enable_write,
                can_interface,
                serial_port,
                template_dir,
            } => {
                state.dry_run = dry_run;
                state.enable_write = enable_write;
                state.initialized = true;
                let _ = (can_interface, serial_port, template_dir);
                // Seed one serial frame so detect_ecms has at least one observed channel.
                state.push_serial(vec![0x62, 0xF1, 0x90, b'O', b'K'], "bridge-heartbeat");
                state.make_ok()
            }
            BridgeRequest::Ping => {
                if state.initialized {
                    state.make_ok()
                } else {
                    BridgeResponse {
                        ok: false,
                        frame: None,
                        error: Some("bridge not initialized".to_string()),
                        kind: Some("permission".to_string()),
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: None,
                    }
                }
            }
            BridgeRequest::Read { timeout_ms } => {
                let _ = timeout_ms;
                if let Some(frame) = state.rx_queue.pop_front() {
                    BridgeResponse {
                        ok: true,
                        frame: Some(frame),
                        error: None,
                        kind: None,
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: Some(BridgeDiagPayload {
                            wait_frame_count_delta: 0,
                            sequence_error_count_delta: 0,
                            flowcontrol_timeout_count_delta: 0,
                            fc_blocksize: state.negotiated_blocksize,
                            fc_stmin_ms: state.negotiated_stmin_ms,
                        }),
                    }
                } else {
                    BridgeResponse {
                        ok: false,
                        frame: None,
                        error: Some("read timeout".to_string()),
                        kind: Some("timeout".to_string()),
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: None,
                    }
                }
            }
            BridgeRequest::TryRead => BridgeResponse {
                ok: true,
                frame: state.rx_queue.pop_front(),
                error: None,
                kind: None,
                protocol_version: Some("1.0".to_string()),
                capabilities: Some(state.capabilities()),
                diag: None,
            },
            BridgeRequest::Send { frame } => {
                if !state.dry_run && !state.enable_write {
                    BridgeResponse {
                        ok: false,
                        frame: None,
                        error: Some("write blocked by bridge policy".to_string()),
                        kind: Some("write_blocked".to_string()),
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: None,
                    }
                } else {
                    if let Frame::Serial(sf) = &frame {
                        let is_uds = sf
                            .protocol_hint
                            .as_deref()
                            .map(|h| h.eq_ignore_ascii_case("uds-raw"))
                            .unwrap_or(false);
                        if is_uds {
                            state.handle_uds_serial_request(&sf.bytes);
                        }
                    }
                    BridgeResponse {
                        ok: true,
                        frame: None,
                        error: None,
                        kind: None,
                        protocol_version: Some("1.0".to_string()),
                        capabilities: Some(state.capabilities()),
                        diag: Some(BridgeDiagPayload {
                            wait_frame_count_delta: 0,
                            sequence_error_count_delta: 0,
                            flowcontrol_timeout_count_delta: 0,
                            fc_blocksize: state.negotiated_blocksize,
                            fc_stmin_ms: state.negotiated_stmin_ms,
                        }),
                    }
                }
            }
            BridgeRequest::Close => {
                let rsp = state.make_ok();
                write_response(&mut stdout, rsp);
                break;
            }
        };

        write_response(&mut stdout, rsp);
    }
}

fn write_response(stdout: &mut io::Stdout, rsp: BridgeResponse) {
    let line = match serde_json::to_string(&rsp) {
        Ok(v) => v,
        Err(e) => {
            let fallback = format!(
                "{{\"ok\":false,\"error\":\"serialize failed: {}\",\"kind\":\"parse\"}}",
                e
            );
            let _ = writeln!(stdout, "{fallback}");
            let _ = stdout.flush();
            return;
        }
    };
    let _ = writeln!(stdout, "{line}");
    let _ = stdout.flush();
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}
