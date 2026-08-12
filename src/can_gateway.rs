//! Central CAN Bus Gateway — routes frames between all ECUs, monitors bus health.
//!
//! Models a real HS-CAN (500 kbps) bus as used in heavy machinery.
//! Implements J1939 network management: address claiming, bus-off recovery,
//! error counting per ISO 11898-1 (TEC/REC counters), and frame arbitration.

use crate::j1939::{J1939Bus, J1939Frame};
use std::collections::VecDeque;

// ── Bus State Machine (ISO 11898-1) ─────────────────────────────────────────
/// ISO 11898-1 error confinement states
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusState {
    /// TEC < 128 && REC < 128 — normal operation
    ErrorActive,
    /// TEC ≥ 128 or REC ≥ 128 — sends passive error flags
    ErrorPassive,
    /// TEC ≥ 256 — node disconnects from bus, recovery after 128 × 11 recessive bits
    BusOff,
}

impl std::fmt::Display for BusState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BusState::ErrorActive => write!(f, "ERROR-ACTIVE "),
            BusState::ErrorPassive => write!(f, "ERROR-PASSIVE"),
            BusState::BusOff => write!(f, "BUS-OFF      "),
        }
    }
}

// ── Error Frame Log ──────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct BusError {
    pub timestamp: f64,
    pub kind: BusErrorKind,
    pub source_sa: Option<u8>,
    pub description: &'static str,
}

#[derive(Debug, Clone, Copy)]
pub enum BusErrorKind {
    BitError,
    StuffError,
    CrcError,
    FormError,
    AcknowledgeError,
    Overload,
}

// ── Registered Node ──────────────────────────────────────────────────────────
/// Every ECU that connects to the CAN bus registers as a CanNode.
#[derive(Debug, Clone)]
pub struct CanNode {
    pub source_addr: u8,
    pub name: &'static str,
    pub tec: u16, // Transmit Error Counter
    pub rec: u16, // Receive Error Counter
    pub state: BusState,
    pub online: bool,
    pub last_tx_ts: f64,
}

impl CanNode {
    pub fn new(sa: u8, name: &'static str) -> Self {
        CanNode {
            source_addr: sa,
            name,
            tec: 0,
            rec: 0,
            state: BusState::ErrorActive,
            online: false,
            last_tx_ts: 0.0,
        }
    }

    /// ISO 11898-1: update error counters and derive state.
    pub fn record_tx_error(&mut self) {
        self.tec = self.tec.saturating_add(8);
        self.update_state();
    }

    pub fn record_rx_error(&mut self) {
        self.rec = self.rec.saturating_add(1);
        self.update_state();
    }

    pub fn record_success(&mut self) {
        // Decrement both counters on success
        self.tec = self.tec.saturating_sub(1);
        self.rec = self.rec.saturating_sub(1);
        self.update_state();
    }

    fn update_state(&mut self) {
        self.state = if self.tec >= 256 {
            BusState::BusOff
        } else if self.tec >= 128 || self.rec >= 128 {
            BusState::ErrorPassive
        } else {
            BusState::ErrorActive
        };
    }
}

// ── CAN Gateway ──────────────────────────────────────────────────────────────

/// Central gateway / arbiter for the J1939 CAN bus.
pub struct CanGateway {
    /// Full frame log (for CAN Analyzer tab)
    pub bus: J1939Bus,

    /// Frames queued for transmission this cycle (before arbitration)
    tx_queue: Vec<J1939Frame>,

    /// All frames dispatched this cycle (read by each ECU on its tick)
    pub dispatched: Vec<J1939Frame>,

    /// Registered nodes / ECUs
    pub nodes: Vec<CanNode>,

    /// Bus-level statistics
    pub total_tx: u64,
    pub total_rx: u64,
    pub total_errors: u64,
    pub bus_state: BusState,

    /// Error log (last 64)
    pub error_log: VecDeque<BusError>,

    /// Bus load (updated every second)
    pub bus_load_pct: f64,
    frame_counter_1s: u32,
    time_accumulator: f64,

    // Fault injection flags (for the fault panel)
    pub inject_bit_error: bool,
    pub inject_bus_off: bool,
    pub inject_missing_ack: bool,
}

impl Default for CanGateway {
    fn default() -> Self {
        Self::new()
    }
}

impl CanGateway {
    pub fn new() -> Self {
        CanGateway {
            bus: J1939Bus::new(),
            tx_queue: Vec::new(),
            dispatched: Vec::new(),
            nodes: Vec::new(),
            total_tx: 0,
            total_rx: 0,
            total_errors: 0,
            bus_state: BusState::ErrorActive,
            error_log: VecDeque::new(),
            bus_load_pct: 0.0,
            frame_counter_1s: 0,
            time_accumulator: 0.0,
            inject_bit_error: false,
            inject_bus_off: false,
            inject_missing_ack: false,
        }
    }

    /// Register an ECU on the bus
    pub fn register_node(&mut self, sa: u8, name: &'static str) {
        if !self.nodes.iter().any(|n| n.source_addr == sa) {
            self.nodes.push(CanNode::new(sa, name));
        }
    }

    /// Mark a node as online (called after successful address claim)
    pub fn set_node_online(&mut self, sa: u8) {
        if let Some(node) = self.nodes.iter_mut().find(|n| n.source_addr == sa) {
            node.online = true;
            node.tec = 0;
            node.rec = 0;
        }
    }

    /// Called by each ECU to submit a frame for transmission
    pub fn transmit(&mut self, frame: J1939Frame) {
        self.tx_queue.push(frame);
    }

    /// Tick: arbitrate all queued frames, dispatch them, update counters.
    /// Call once per simulation step.
    pub fn tick(&mut self, dt: f64) {
        // ── Fault injection ──────────────────────────────────────────────────
        if self.inject_bus_off {
            self.bus_state = BusState::BusOff;
            self.inject_bus_off = false;
            self.log_error(
                0.0,
                BusErrorKind::BitError,
                None,
                "INJECTED: Bus-Off condition",
            );
        }

        // ── Arbitration: sort by CAN ID (lower ID = higher priority) ────────
        self.tx_queue.sort_by_key(|f| f.raw_id);

        self.dispatched.clear();

        // Collect frames to process (avoids borrow conflict with self.log_error)
        let frames: Vec<J1939Frame> = self.tx_queue.drain(..).collect();

        for frame in frames {
            // Fault injection: drop frame + log error
            if self.inject_bit_error && self.total_tx.is_multiple_of(100) {
                self.total_errors += 1;
                let ts = frame.timestamp;
                let sa = frame.sa;
                self.error_log.push_front(BusError {
                    timestamp: ts,
                    kind: BusErrorKind::BitError,
                    source_sa: Some(sa),
                    description: "INJECTED: Bit error",
                });
                if self.error_log.len() > 64 {
                    self.error_log.pop_back();
                }
                continue;
            }
            if self.inject_missing_ack && self.total_tx.is_multiple_of(50) {
                self.total_errors += 1;
                let ts = frame.timestamp;
                let sa = frame.sa;
                self.error_log.push_front(BusError {
                    timestamp: ts,
                    kind: BusErrorKind::AcknowledgeError,
                    source_sa: Some(sa),
                    description: "INJECTED: ACK missing",
                });
                if self.error_log.len() > 64 {
                    self.error_log.pop_back();
                }
                if let Some(node) = self.nodes.iter_mut().find(|n| n.source_addr == sa) {
                    node.record_tx_error();
                }
                continue;
            }

            // Update node stats
            if let Some(node) = self.nodes.iter_mut().find(|n| n.source_addr == frame.sa) {
                node.last_tx_ts = frame.timestamp;
                node.record_success();
            }
            self.bus.push(frame.clone());
            self.dispatched.push(frame);
            self.total_tx += 1;
            self.frame_counter_1s += 1;
        }

        // ── Bus load calculation ─────────────────────────────────────────────
        // 500 kbps, avg frame = ~120 bits (8B data + overhead) → max ~4166 frames/s
        self.time_accumulator += dt;
        if self.time_accumulator >= 1.0 {
            let max_fps = 4166.0;
            self.bus_load_pct = (self.frame_counter_1s as f64 / max_fps * 100.0).min(100.0);
            self.bus.fps = self.frame_counter_1s;
            self.bus.tick(dt);
            self.frame_counter_1s = 0;
            self.time_accumulator = 0.0;
        }
    }

    /// Get all frames dispatched this cycle visible to a given SA.
    /// Returns: broadcast frames + peer-to-peer frames addressed to `sa`.
    pub fn receive_for(&self, sa: u8) -> impl Iterator<Item = &J1939Frame> {
        self.dispatched
            .iter()
            .filter(move |f| f.sa != sa && (f.da == 0xFF || f.da == sa))
    }

    /// Peek at latest N frames from the bus log (for CAN Analyzer)
    pub fn analyzer_frames(&self, count: usize) -> impl Iterator<Item = &J1939Frame> {
        self.bus.frames.iter().take(count)
    }

    fn log_error(&mut self, ts: f64, kind: BusErrorKind, sa: Option<u8>, desc: &'static str) {
        self.error_log.push_front(BusError {
            timestamp: ts,
            kind,
            source_sa: sa,
            description: desc,
        });
        if self.error_log.len() > 64 {
            self.error_log.pop_back();
        }
        self.total_errors += 1;
    }

    /// Simulate a node going offline (e.g., ECU power cut)
    pub fn kill_node(&mut self, sa: u8) {
        if let Some(node) = self.nodes.iter_mut().find(|n| n.source_addr == sa) {
            node.online = false;
            node.tec = 256;
            node.state = BusState::BusOff;
        }
    }

    /// Revive a killed node
    pub fn revive_node(&mut self, sa: u8) {
        if let Some(node) = self.nodes.iter_mut().find(|n| n.source_addr == sa) {
            node.online = true;
            node.tec = 0;
            node.rec = 0;
            node.state = BusState::ErrorActive;
        }
    }
}
