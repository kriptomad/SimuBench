//! CAN Network — Multi-bus CAN simulation 1:1 with real heavy machinery.
#![allow(dead_code)]
//!
//! Real vehicles have multiple CAN buses with different speeds and purposes:
//!
//!   BUS-1  HS-CAN Powertrain  500 kbps  ECM, TCM, ABS/ESP, VCM
//!   BUS-2  HS-CAN Chassis     500 kbps  ABS, Steering, Suspension, Brakes
//!   BUS-3  MS-CAN Body        250 kbps  BCM, ICM, PEPS, HVAC, Doors
//!   BUS-4  HS-CAN ISOBUS      250 kbps  HCM, Implement, TaskCtrl, VT
//!   BUS-5  Diagnostic CAN     500 kbps  OBD-II port, UDS client connection
//!   BUS-6  LIN (master bus)    20 kbps  Simple sensors/actuators (seats, mirrors)
//!
//! Each bus has independent:
//!   • Bit timing (prescaler, phase segments, SJW)
//!   • Error counters (TEC/REC) per node
//!   • Bus load calculation
//!   • Frame arbitration (priority-based)
//!   • Error frame injection
//!   • CAN FD payload support (up to 64 bytes)
//!
//! J1939 Transport Protocol (TP) is implemented over BUS-1 and BUS-4:
//!   BAM  — Broadcast Announce Message (multi-packet broadcast)
//!   CMDT — Connection Mode Data Transfer (peer-to-peer with flow control)
//!
//! The VCM (Vehicle Control Module) acts as gateway between buses, forwarding
//! signals between powertrain, body, and ISOBUS domains.

use crate::j1939::{addr, pgn, J1939Frame};
use serde::Serialize;
use std::collections::{HashMap, VecDeque};
use std::fs;
use std::io::Write;
use std::path::Path;

// ── CAN Bus Speed ─────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CanSpeed {
    Kbps20 = 20_000,
    Kbps125 = 125_000,
    Kbps250 = 250_000,
    Kbps500 = 500_000,
    Kbps1000 = 1_000_000,
    /// CAN FD data phase (up to 5 Mbit/s, ISO 11898-2:2016)
    FdMbps2 = 2_000_000,
    FdMbps5 = 5_000_000,
}

impl CanSpeed {
    pub fn kbps(&self) -> u32 {
        *self as u32 / 1000
    }
    /// Maximum frames per second for this speed (approx, 8-byte payload, stuffing)
    pub fn max_fps(&self) -> u32 {
        match self {
            CanSpeed::Kbps20 => 150,
            CanSpeed::Kbps125 => 930,
            CanSpeed::Kbps250 => 1860,
            CanSpeed::Kbps500 => 3720,
            CanSpeed::Kbps1000 => 7440,
            CanSpeed::FdMbps2 => 40_000,
            CanSpeed::FdMbps5 => 100_000,
        }
    }
}

// ── Bus ID ────────────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BusId {
    PowertrainHs, // BUS-1: ECM, TCM, ABS, VCM @ 500kbps
    ChassisHs,    // BUS-2: ABS, Steering, Suspension @ 500kbps
    BodyMs,       // BUS-3: BCM, ICM, PEPS, HVAC @ 250kbps
    IsoBus,       // BUS-4: HCM, Implements, ISOBUS @ 250kbps
    Diagnostic,   // BUS-5: OBD-II port, UDS @ 500kbps
    Lin,          // BUS-6: LIN master @ 20kbps
}

impl std::fmt::Display for BusId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BusId::PowertrainHs => write!(f, "HS-CAN-1 Powertrain"),
            BusId::ChassisHs => write!(f, "HS-CAN-2 Chassis   "),
            BusId::BodyMs => write!(f, "MS-CAN-3 Body      "),
            BusId::IsoBus => write!(f, "ISOBUS-4 Implement "),
            BusId::Diagnostic => write!(f, "HS-CAN-5 Diag      "),
            BusId::Lin => write!(f, "LIN-6    Sensors   "),
        }
    }
}

// ── Bus State (ISO 11898-1 §6.15) ─────────────────────────────────────────────
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusState {
    ErrorActive,  // TEC < 128 && REC < 128
    ErrorPassive, // TEC >= 128 or REC >= 128
    BusOff,       // TEC >= 256 — node disconnected
}

impl std::fmt::Display for BusState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BusState::ErrorActive => write!(f, "ERR-ACT "),
            BusState::ErrorPassive => write!(f, "ERR-PSV "),
            BusState::BusOff => write!(f, "BUS-OFF!"),
        }
    }
}

// ── Error kind ────────────────────────────────────────────────────────────────
#[derive(Debug, Clone, Copy)]
pub enum CanErrorKind {
    Bit,
    Stuff,
    Crc,
    Form,
    Ack,
    Overload,
    BablingIdiot,
}

impl std::fmt::Display for CanErrorKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CanErrorKind::Bit => write!(f, "Bit"),
            CanErrorKind::Stuff => write!(f, "Stuff"),
            CanErrorKind::Crc => write!(f, "CRC"),
            CanErrorKind::Form => write!(f, "Form"),
            CanErrorKind::Ack => write!(f, "Ack"),
            CanErrorKind::Overload => write!(f, "Overload"),
            CanErrorKind::BablingIdiot => write!(f, "Babbling"),
        }
    }
}

// ── Bus error record ──────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct BusError {
    pub timestamp: f64,
    pub bus: BusId,
    pub kind: CanErrorKind,
    pub source_sa: Option<u8>,
    pub raw_id: Option<u32>,
    pub description: &'static str,
}

// ── Registered node ───────────────────────────────────────────────────────────
#[derive(Debug, Clone)]
pub struct CanNode {
    pub sa: u8,
    pub name: &'static str,
    pub bus: BusId,
    pub tec: u16, // Transmit Error Counter
    pub rec: u16, // Receive Error Counter
    pub state: BusState,
    pub online: bool,
    pub last_tx_ts: f64,
    pub tx_count: u64,
    pub rx_count: u64,
    /// Frames per second (exponentially-smoothed)
    pub fps: f64,
    /// True if this node is transmitting too fast (babbling idiot)
    pub babbling: bool,
    fps_timer: f64,
    fps_count: u32,
}

impl CanNode {
    pub fn new(sa: u8, name: &'static str, bus: BusId) -> Self {
        CanNode {
            sa,
            name,
            bus,
            tec: 0,
            rec: 0,
            state: BusState::ErrorActive,
            online: false,
            last_tx_ts: 0.0,
            tx_count: 0,
            rx_count: 0,
            fps: 0.0,
            babbling: false,
            fps_timer: 0.0,
            fps_count: 0,
        }
    }
    pub fn tx_success(&mut self) {
        self.tec = self.tec.saturating_sub(1);
        self.update_state();
    }
    pub fn tx_error(&mut self) {
        self.tec = self.tec.saturating_add(8);
        self.update_state();
    }
    pub fn rx_error(&mut self) {
        self.rec = self.rec.saturating_add(1);
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

// ═════════════════════════════════════════════════════════════════════════════
// CAN Transport Protocol — J1939-21 TP.BAM and TP.CMDT
// ═════════════════════════════════════════════════════════════════════════════

/// State of an active TP session
#[derive(Debug, Clone)]
struct TpSession {
    pgn: u32,
    sa: u8,
    da: u8, // 0xFF = BAM broadcast
    data: Vec<u8>,
    total_bytes: u16,
    total_packets: u8,
    next_seq: u8,
    timer: f64,
    is_bam: bool,
}

/// J1939 Transport Protocol engine (BAM + CMDT)
pub struct CanTransportProtocol {
    /// Active send sessions: key = (sa, da, pgn)
    tx_sessions: Vec<TpSession>,
    /// Active receive sessions: key = (sa, da)
    rx_sessions: Vec<TpSession>,
    /// Completed reassembled messages ready for delivery
    pub completed: Vec<(u32, u8, u8, Vec<u8>)>, // (pgn, sa, da, data)
}

impl CanTransportProtocol {
    pub fn new() -> Self {
        CanTransportProtocol {
            tx_sessions: Vec::new(),
            rx_sessions: Vec::new(),
            completed: Vec::new(),
        }
    }

    /// Queue a large J1939 message for BAM broadcast (> 8 bytes)
    pub fn send_bam(&mut self, ts: f64, pgn: u32, sa: u8, data: Vec<u8>) -> Vec<J1939Frame> {
        let total = data.len() as u16;
        let packets = ((total + 6) / 7) as u8;
        let mut frames = Vec::new();

        // TP.CM_BAM — announce the broadcast
        let mut bam = [0xFFu8; 8];
        bam[0] = 0x20; // Control byte: BAM
        bam[1] = (total & 0xFF) as u8;
        bam[2] = ((total >> 8) & 0xFF) as u8;
        bam[3] = packets;
        bam[4] = 0xFF;
        bam[5] = (pgn & 0xFF) as u8;
        bam[6] = ((pgn >> 8) & 0xFF) as u8;
        bam[7] = ((pgn >> 16) & 0xFF) as u8;
        frames.push(J1939Frame::from_raw(
            ts,
            J1939Frame::build_id(7, pgn::TP_CM, sa, 0xFF),
            &bam,
        ));

        // TP.DT — data transfer packets (send immediately for BAM)
        for seq in 0..packets {
            let start = seq as usize * 7;
            let end = (start + 7).min(data.len());
            let mut dt = [0xFFu8; 8];
            dt[0] = seq + 1; // sequence 1-based
            for (i, &b) in data[start..end].iter().enumerate() {
                dt[i + 1] = b;
            }
            frames.push(J1939Frame::from_raw(
                ts + (seq as f64 * 0.010),
                J1939Frame::build_id(7, pgn::TP_DT, sa, 0xFF),
                &dt,
            ));
        }
        frames
    }

    /// Process received TP frame — reassemble multi-packet messages
    pub fn process_rx(&mut self, frame: &J1939Frame) -> bool {
        match frame.pgn {
            p if p == pgn::TP_CM => {
                let ctrl = frame.data[0];
                if ctrl == 0x20 {
                    // BAM announce
                    let total = (frame.data[1] as u16) | ((frame.data[2] as u16) << 8);
                    let packets = frame.data[3];
                    let pgn_rx = (frame.data[5] as u32)
                        | ((frame.data[6] as u32) << 8)
                        | ((frame.data[7] as u32) << 16);
                    self.rx_sessions.retain(|s| s.sa != frame.sa);
                    self.rx_sessions.push(TpSession {
                        pgn: pgn_rx,
                        sa: frame.sa,
                        da: 0xFF,
                        data: Vec::new(),
                        total_bytes: total,
                        total_packets: packets,
                        next_seq: 1,
                        timer: 0.0,
                        is_bam: true,
                    });
                    return true;
                }
                false
            }
            p if p == pgn::TP_DT => {
                let seq = frame.data[0];
                if let Some(sess) = self
                    .rx_sessions
                    .iter_mut()
                    .find(|s| s.sa == frame.sa && s.next_seq == seq)
                {
                    let payload = &frame.data[1..];
                    let remaining = (sess.total_bytes as usize).saturating_sub(sess.data.len());
                    let to_copy = remaining.min(7);
                    sess.data
                        .extend_from_slice(&payload[..to_copy.min(payload.len())]);
                    sess.next_seq += 1;
                    if sess.data.len() >= sess.total_bytes as usize {
                        let done = sess.clone();
                        self.rx_sessions.retain(|s| s.sa != frame.sa);
                        self.completed.push((done.pgn, done.sa, done.da, done.data));
                    }
                    return true;
                }
                false
            }
            _ => false,
        }
    }

    pub fn tick(&mut self, dt: f64) {
        for s in &mut self.tx_sessions {
            s.timer += dt;
        }
        for s in &mut self.rx_sessions {
            s.timer += dt;
        }
        // Timeout stale sessions after 750ms (J1939 specification)
        self.tx_sessions.retain(|s| s.timer < 0.750);
        self.rx_sessions.retain(|s| s.timer < 0.750);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Message Scheduling — periodic transmission matrix
// ═════════════════════════════════════════════════════════════════════════════

/// Scheduled J1939 message descriptor
pub struct MessageSchedule {
    pub pgn: u32,
    pub sa: u8,
    pub da: u8, // 0xFF = broadcast
    pub bus: BusId,
    pub period_ms: f64,
    pub priority: u8,
    /// Description for CAN database / DBC compatibility
    pub description: &'static str,
    /// Current timer (fires when >= period_ms)
    timer_ms: f64,
    /// Transmission jitter ±jitter_ms (realistic)
    pub jitter_ms: f64,
    jitter_offset: f64,
}

impl MessageSchedule {
    pub fn new(
        pgn: u32,
        sa: u8,
        da: u8,
        bus: BusId,
        period_ms: f64,
        priority: u8,
        desc: &'static str,
    ) -> Self {
        MessageSchedule {
            pgn,
            sa,
            da,
            bus,
            period_ms,
            priority,
            description: desc,
            timer_ms: 0.0,
            jitter_ms: 1.0,
            jitter_offset: 0.0,
        }
    }
    /// Returns true if it's time to transmit, applying jitter
    pub fn should_fire(&mut self, dt_ms: f64, noise: f64) -> bool {
        self.jitter_offset = noise * self.jitter_ms;
        self.timer_ms += dt_ms;
        if self.timer_ms >= self.period_ms + self.jitter_offset {
            self.timer_ms = 0.0;
            true
        } else {
            false
        }
    }
}

/// The complete J1939 message matrix for a heavy agricultural vehicle
pub fn build_message_matrix() -> Vec<MessageSchedule> {
    vec![
        // ─ Powertrain bus (BUS-1 HS-CAN 500kbps) ────────────────────────────
        MessageSchedule::new(
            pgn::EEC1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            10.0,
            3,
            "EEC1: Engine Speed/Torque",
        ),
        MessageSchedule::new(
            pgn::EEC2,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            50.0,
            3,
            "EEC2: Throttle/Load",
        ),
        MessageSchedule::new(
            pgn::IC1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            500.0,
            6,
            "IC1: Boost/Intake Conditions",
        ),
        MessageSchedule::new(
            pgn::ET1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "ET1: Engine Temperatures",
        ),
        MessageSchedule::new(
            pgn::EFL_P1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            500.0,
            6,
            "EFL/P1: Oil/Fuel Pressures",
        ),
        MessageSchedule::new(
            pgn::LFE,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            100.0,
            6,
            "LFE: Fuel Economy",
        ),
        MessageSchedule::new(
            pgn::HOURS,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "HOURS: Engine Hours",
        ),
        MessageSchedule::new(
            pgn::DM1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "DM1: Active DTCs (ECM)",
        ),
        MessageSchedule::new(
            pgn::EEC3,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            250.0,
            6,
            "EEC3: Engine Demand/Nominal",
        ),
        MessageSchedule::new(
            pgn::FUEL1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "FUEL1: Cumulative Fuel",
        ),
        MessageSchedule::new(
            0,
            addr::ECM_1,
            addr::TRANSMISSION,
            BusId::PowertrainHs,
            20.0,
            3,
            "TSC1: Torque/Speed Ctrl → TCM",
        ),
        MessageSchedule::new(
            pgn::ETC1,
            addr::TRANSMISSION,
            0xFF,
            BusId::PowertrainHs,
            20.0,
            3,
            "ETC1: Trans Output Shaft Speed",
        ),
        MessageSchedule::new(
            pgn::ETC2,
            addr::TRANSMISSION,
            0xFF,
            BusId::PowertrainHs,
            20.0,
            3,
            "ETC2: Gear/Range Position",
        ),
        MessageSchedule::new(
            pgn::DM1,
            addr::TRANSMISSION,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "DM1: Active DTCs (TCM)",
        ),
        MessageSchedule::new(
            pgn::TCFG,
            addr::TRANSMISSION,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            7,
            "TCFG: Transmission Config",
        ),
        MessageSchedule::new(
            pgn::EBC1,
            addr::BRAKES,
            0xFF,
            BusId::PowertrainHs,
            20.0,
            2,
            "EBC1: Brake Status (ABS/ESP)",
        ),
        MessageSchedule::new(
            pgn::EBC2,
            addr::BRAKES,
            0xFF,
            BusId::PowertrainHs,
            100.0,
            6,
            "EBC2: Wheel Speeds",
        ),
        MessageSchedule::new(
            pgn::DM1,
            addr::BRAKES,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "DM1: Active DTCs (ABS)",
        ),
        MessageSchedule::new(
            pgn::CCVS,
            addr::INSTRUMENT,
            0xFF,
            BusId::PowertrainHs,
            100.0,
            6,
            "CCVS: Vehicle Speed/CC",
        ),
        MessageSchedule::new(
            pgn::VD,
            addr::INSTRUMENT,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "VD: Vehicle Distance",
        ),
        // ─ ISOBUS (BUS-4, 250kbps) ───────────────────────────────────────────
        MessageSchedule::new(
            pgn::PTO,
            addr::HITCH,
            0xFF,
            BusId::IsoBus,
            100.0,
            6,
            "PTO: Power Take-Off Status",
        ),
        MessageSchedule::new(
            pgn::HITCH,
            addr::HITCH,
            0xFF,
            BusId::IsoBus,
            100.0,
            6,
            "HITCH: 3-Point Hitch Status",
        ),
        MessageSchedule::new(
            pgn::DM1,
            addr::HITCH,
            0xFF,
            BusId::IsoBus,
            1000.0,
            6,
            "DM1: Active DTCs (HCM)",
        ),
        MessageSchedule::new(
            pgn::WS_MST,
            addr::TASK_CTRL,
            0xFF,
            BusId::IsoBus,
            100.0,
            6,
            "WS_MASTER: Working Set Master",
        ),
        // ─ Body bus (BUS-3, 250kbps) ─────────────────────────────────────────
        MessageSchedule::new(
            pgn::PROP_A,
            addr::CAB,
            0xFF,
            BusId::BodyMs,
            100.0,
            7,
            "BCM Status: Lights/Battery",
        ),
        MessageSchedule::new(
            pgn::PROP_A + 1,
            addr::INSTRUMENT,
            0xFF,
            BusId::BodyMs,
            100.0,
            7,
            "ICM Status: Warning Lamps",
        ),
        MessageSchedule::new(
            pgn::AMB,
            addr::CAB,
            0xFF,
            BusId::BodyMs,
            1000.0,
            6,
            "AMB: Ambient Conditions",
        ),
        MessageSchedule::new(
            pgn::VEP1,
            addr::CAB,
            0xFF,
            BusId::BodyMs,
            500.0,
            6,
            "VEP1: Vehicle Electrical Power",
        ),
        // ─ Network Management (all buses) ────────────────────────────────────
        MessageSchedule::new(
            pgn::ECAN,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "AC: Address Claim (ECM)",
        ),
        MessageSchedule::new(
            pgn::ECAN,
            addr::TRANSMISSION,
            0xFF,
            BusId::PowertrainHs,
            1000.0,
            6,
            "AC: Address Claim (TCM)",
        ),
        MessageSchedule::new(
            pgn::DM13,
            addr::ECM_1,
            addr::TRANSMISSION,
            BusId::PowertrainHs,
            5000.0,
            7,
            "DM13: Stop/Start Broadcast",
        ),
    ]
}

// ═════════════════════════════════════════════════════════════════════════════
// Single CAN Bus
// ═════════════════════════════════════════════════════════════════════════════

pub struct CanBus {
    pub id: BusId,
    pub speed: CanSpeed,
    pub state: BusState,
    pub can_fd: bool, // CAN FD support

    // ─ Frame buffers ─────────────────────────────────────────────────────────
    tx_queue: Vec<J1939Frame>,           // waiting to be arbitrated
    pub dispatched: Vec<J1939Frame>,     // sent this cycle
    pub frame_log: VecDeque<J1939Frame>, // audit log, last 500 frames

    // ─ Statistics ────────────────────────────────────────────────────────────
    pub total_tx: u64,
    pub total_errors: u64,
    pub bus_load_pct: f64,
    frame_counter: u32,
    load_timer: f64,

    // ─ Error injection ───────────────────────────────────────────────────────
    pub inject_bit_error: bool,
    pub inject_bus_off: bool,
    pub inject_missing_ack: bool,
    pub inject_babbling_sa: Option<u8>, // force a node to babble

    // ─ Bus-off recovery ───────────────────────────────────────────────────────
    /// Counts 128-occurrence groups of 11 recessive bits for bus-off recovery
    busoff_recovery_counter: u8,
    busoff_recovery_timer: f64,

    // ─ Error log ─────────────────────────────────────────────────────────────
    pub error_log: VecDeque<BusError>,

    // ─ Transport Protocol engine ─────────────────────────────────────────────
    pub tp: CanTransportProtocol,
}

impl CanBus {
    pub fn new(id: BusId, speed: CanSpeed) -> Self {
        CanBus {
            id,
            speed,
            state: BusState::ErrorActive,
            can_fd: false,
            tx_queue: Vec::new(),
            dispatched: Vec::new(),
            frame_log: VecDeque::new(),
            total_tx: 0,
            total_errors: 0,
            bus_load_pct: 0.0,
            frame_counter: 0,
            load_timer: 0.0,
            inject_bit_error: false,
            inject_bus_off: false,
            inject_missing_ack: false,
            inject_babbling_sa: None,
            busoff_recovery_counter: 0,
            busoff_recovery_timer: 0.0,
            error_log: VecDeque::new(),
            tp: CanTransportProtocol::new(),
        }
    }

    /// Enqueue a frame for transmission
    pub fn transmit(&mut self, frame: J1939Frame) {
        self.tx_queue.push(frame);
    }

    /// Transmit a large message via TP.BAM
    pub fn transmit_tp(&mut self, ts: f64, pgn: u32, sa: u8, data: Vec<u8>) {
        if data.len() <= 8 {
            // Fits in one frame
            let id = J1939Frame::build_id(6, pgn, sa, 0xFF);
            let mut arr = [0xFFu8; 8];
            arr[..data.len()].copy_from_slice(&data);
            self.transmit(J1939Frame::from_raw(ts, id, &arr));
        } else {
            let frames = self.tp.send_bam(ts, pgn, sa, data);
            for f in frames {
                self.transmit(f);
            }
        }
    }

    /// Arbitrate and dispatch all queued frames for this cycle.
    pub fn tick(&mut self, nodes: &mut HashMap<u8, CanNode>, dt: f64) {
        self.dispatched.clear();
        self.tp.tick(dt);

        // ─ Bus-off recovery timer ──────────────────────────────────────────
        if self.state == BusState::BusOff {
            self.busoff_recovery_timer += dt;
            // 128 occurrences of 11 recessive bits = ~128 × 11 / speed
            let recovery_bit_time = 128.0 * 11.0 / self.speed.kbps() as f64 / 1000.0;
            if self.busoff_recovery_timer >= recovery_bit_time {
                self.state = BusState::ErrorActive;
                self.busoff_recovery_timer = 0.0;
                self.log_error(
                    0.0,
                    CanErrorKind::Bit,
                    None,
                    None,
                    "Bus-Off RECOVERED after 128×11 recessive bits",
                );
            }
            // No transmission during bus-off
            self.tx_queue.clear();
            return;
        }

        // ─ Injected bus-off ───────────────────────────────────────────────
        if self.inject_bus_off {
            self.state = BusState::BusOff;
            self.busoff_recovery_timer = 0.0;
            self.inject_bus_off = false;
            self.log_error(
                0.0,
                CanErrorKind::Bit,
                None,
                None,
                "INJECTED: Bus-Off condition",
            );
        }

        // ─ Babbling-idiot detection ────────────────────────────────────────
        // Real CAN: a node transmitting more than ~2× its rated period is a babbler
        if let Some(bab_sa) = self.inject_babbling_sa {
            if let Some(node) = nodes.get_mut(&bab_sa) {
                node.babbling = true;
                node.tx_error(); // Force error state
                self.log_error(
                    0.0,
                    CanErrorKind::BablingIdiot,
                    Some(bab_sa),
                    None,
                    "Babbling-idiot: node transmitting too fast",
                );
            }
        }

        // ─ Sort by CAN ID: lower = higher priority ────────────────────────
        self.tx_queue.sort_by_key(|f| f.raw_id);

        let frames: Vec<J1939Frame> = self.tx_queue.drain(..).collect();
        for frame in frames {
            // ─ Bit error injection (1 in 200) ───────────────────────────────
            if self.inject_bit_error && self.total_tx % 200 == 0 {
                if let Some(node) = nodes.get_mut(&frame.sa) {
                    node.tx_error();
                }
                self.total_errors += 1;
                self.log_error(
                    frame.timestamp,
                    CanErrorKind::Bit,
                    Some(frame.sa),
                    Some(frame.raw_id),
                    "INJECTED: Bit error — frame dropped",
                );
                continue;
            }
            // ─ Missing ACK injection ────────────────────────────────────────
            if self.inject_missing_ack && self.total_tx % 75 == 0 {
                if let Some(node) = nodes.get_mut(&frame.sa) {
                    node.tx_error();
                }
                self.total_errors += 1;
                self.log_error(
                    frame.timestamp,
                    CanErrorKind::Ack,
                    Some(frame.sa),
                    Some(frame.raw_id),
                    "INJECTED: No ACK received",
                );
                continue;
            }
            // ─ Success: update node stats ────────────────────────────────────
            if let Some(node) = nodes.get_mut(&frame.sa) {
                node.tx_success();
                node.last_tx_ts = frame.timestamp;
                node.tx_count += 1;
                node.fps_count += 1;
            }

            // ─ TP reassembly for received frames ────────────────────────────
            let _ = self.tp.process_rx(&frame);

            self.frame_log.push_front(frame.clone());
            if self.frame_log.len() > 500 {
                self.frame_log.pop_back();
            }
            self.dispatched.push(frame);
            self.total_tx += 1;
            self.frame_counter += 1;
        }

        // ─ Bus load (updated each second) ───────────────────────────────────
        self.load_timer += dt;
        if self.load_timer >= 1.0 {
            self.bus_load_pct =
                (self.frame_counter as f64 / self.speed.max_fps() as f64 * 100.0).min(100.0);
            // Update fps per node
            for node in nodes.values_mut() {
                if node.bus == self.id {
                    node.fps = node.fps_count as f64;
                    node.fps_count = 0;
                }
            }
            self.frame_counter = 0;
            self.load_timer = 0.0;
        }
    }

    fn log_error(
        &mut self,
        ts: f64,
        kind: CanErrorKind,
        sa: Option<u8>,
        id: Option<u32>,
        desc: &'static str,
    ) {
        self.error_log.push_front(BusError {
            timestamp: ts,
            bus: self.id,
            kind,
            source_sa: sa,
            raw_id: id,
            description: desc,
        });
        if self.error_log.len() > 100 {
            self.error_log.pop_back();
        }
    }

    pub fn receive_for(&self, sa: u8) -> impl Iterator<Item = &J1939Frame> {
        self.dispatched
            .iter()
            .filter(move |f| f.sa != sa && (f.da == 0xFF || f.da == sa))
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Multi-Bus Network Manager
// ═════════════════════════════════════════════════════════════════════════════

pub struct CanNetwork {
    pub powertrain: CanBus,
    pub chassis: CanBus,
    pub body: CanBus,
    pub isobus: CanBus,
    pub diagnostic: CanBus,

    // ─ Node registry (all buses, keyed by SA) ────────────────────────────────
    pub nodes: HashMap<u8, CanNode>,

    // ─ Message scheduling matrix ─────────────────────────────────────────────
    pub schedule: Vec<MessageSchedule>,

    // ─ Cross-bus gateway routing table ───────────────────────────────────────
    /// PGNs that get forwarded from one bus to another (by VCM)
    pub routing: Vec<(u32, BusId, BusId)>, // (pgn, src_bus, dst_bus)

    // ─ Network-wide stats ────────────────────────────────────────────────────
    pub total_frames_all_buses: u64,
    pub elapsed: f64,
    noise_t: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct BusSnapshot {
    pub bus: String,
    pub speed_kbps: u32,
    pub state: String,
    pub load_pct: f64,
    pub total_tx: u64,
    pub total_errors: u64,
    pub log_depth: usize,
}

#[derive(Debug, Clone, Serialize)]
pub struct CanNetworkSnapshot {
    pub elapsed_s: f64,
    pub health_score_01: f64,
    pub total_frames_all_buses: u64,
    pub total_errors_all_buses: u64,
    pub online_nodes: usize,
    pub buses: Vec<BusSnapshot>,
}

impl CanNetwork {
    pub fn new() -> Self {
        let mut net = CanNetwork {
            powertrain: CanBus::new(BusId::PowertrainHs, CanSpeed::Kbps500),
            chassis: CanBus::new(BusId::ChassisHs, CanSpeed::Kbps500),
            body: CanBus::new(BusId::BodyMs, CanSpeed::Kbps250),
            isobus: CanBus::new(BusId::IsoBus, CanSpeed::Kbps250),
            diagnostic: CanBus::new(BusId::Diagnostic, CanSpeed::Kbps500),
            nodes: HashMap::new(),
            schedule: build_message_matrix(),
            routing: vec![
                // VCM forwards engine speed from powertrain to body bus (for speedometer over MS-CAN)
                (pgn::EEC1, BusId::PowertrainHs, BusId::BodyMs),
                (pgn::CCVS, BusId::PowertrainHs, BusId::BodyMs),
                (pgn::DM1, BusId::PowertrainHs, BusId::Diagnostic),
                (pgn::ETC1, BusId::PowertrainHs, BusId::BodyMs),
            ],
            total_frames_all_buses: 0,
            elapsed: 0.0,
            noise_t: 0.0,
        };
        // Register all known nodes on their respective buses
        net.register(addr::ECM_1, "ECM #1", BusId::PowertrainHs);
        net.register(addr::TRANSMISSION, "TCM", BusId::PowertrainHs);
        net.register(addr::BRAKES, "ABS/ESP", BusId::PowertrainHs);
        net.register(addr::CAB, "BCM/CAB", BusId::BodyMs);
        net.register(addr::INSTRUMENT, "ICM/DASH", BusId::BodyMs);
        net.register(addr::HITCH, "HCM/HITCH", BusId::IsoBus);
        net.register(addr::ISOBUS_VT, "ISOBUS-VT", BusId::IsoBus);
        net.register(addr::TASK_CTRL, "ISOBUS-TC", BusId::IsoBus);
        net.register(addr::IMPLEMENT, "IMPLEMENT", BusId::IsoBus);
        net.register(addr::HEADWAY, "VCM/GW", BusId::PowertrainHs);
        net
    }

    pub fn register(&mut self, sa: u8, name: &'static str, bus: BusId) {
        self.nodes.insert(sa, CanNode::new(sa, name, bus));
    }

    pub fn set_online(&mut self, sa: u8) {
        if let Some(n) = self.nodes.get_mut(&sa) {
            n.online = true;
            n.tec = 0;
            n.rec = 0;
            n.state = BusState::ErrorActive;
        }
    }

    /// Transmit a frame on the appropriate bus for the SA
    pub fn transmit(&mut self, frame: J1939Frame) {
        let bus_id = self
            .nodes
            .get(&frame.sa)
            .map(|n| n.bus)
            .unwrap_or(BusId::PowertrainHs);
        self.bus_mut(bus_id).transmit(frame);
    }

    pub fn inject_error_once(&mut self, bus: BusId, kind: CanErrorKind) {
        let babbling_sa = self.nodes.values().find(|n| n.bus == bus).map(|n| n.sa);
        let b = self.bus_mut(bus);
        match kind {
            CanErrorKind::Bit => b.inject_bit_error = true,
            CanErrorKind::Ack => b.inject_missing_ack = true,
            CanErrorKind::BablingIdiot => {
                b.inject_babbling_sa = babbling_sa;
            }
            _ => b.inject_bit_error = true,
        }
    }

    pub fn inject_bus_off_once(&mut self, bus: BusId) {
        self.bus_mut(bus).inject_bus_off = true;
    }

    pub fn clear_injections(&mut self, bus: Option<BusId>) {
        match bus {
            Some(id) => {
                let b = self.bus_mut(id);
                b.inject_bit_error = false;
                b.inject_missing_ack = false;
                b.inject_bus_off = false;
                b.inject_babbling_sa = None;
            }
            None => {
                for id in [
                    BusId::PowertrainHs,
                    BusId::ChassisHs,
                    BusId::BodyMs,
                    BusId::IsoBus,
                    BusId::Diagnostic,
                ] {
                    let b = self.bus_mut(id);
                    b.inject_bit_error = false;
                    b.inject_missing_ack = false;
                    b.inject_bus_off = false;
                    b.inject_babbling_sa = None;
                }
            }
        }
    }

    /// Tick all buses and collect stats
    pub fn tick(&mut self, dt: f64) {
        self.elapsed += dt;
        self.noise_t += dt;
        let _noise = ((self.noise_t * 127.1 + 311.7).sin() * 43758.5).fract() - 0.5;

        // Tick each bus
        self.powertrain.tick(&mut self.nodes, dt);
        self.chassis.tick(&mut self.nodes, dt);
        self.body.tick(&mut self.nodes, dt);
        self.isobus.tick(&mut self.nodes, dt);
        self.diagnostic.tick(&mut self.nodes, dt);

        // VCM routing: forward frames between buses
        self.route_frames();

        self.total_frames_all_buses = self.powertrain.total_tx
            + self.chassis.total_tx
            + self.body.total_tx
            + self.isobus.total_tx
            + self.diagnostic.total_tx;
    }

    fn route_frames(&mut self) {
        // Collect frames to route (can't borrow self.powertrain and self.body mutably simultaneously)
        let mut to_route: Vec<(J1939Frame, BusId)> = Vec::new();
        for (pgn_filter, src_bus, dst_bus) in &self.routing {
            let src_frames: Vec<J1939Frame> = self
                .bus_ref(*src_bus)
                .dispatched
                .iter()
                .filter(|f| f.pgn == *pgn_filter)
                .cloned()
                .collect();
            for f in src_frames {
                to_route.push((f, *dst_bus));
            }
        }
        for (f, dst) in to_route {
            self.bus_mut(dst).transmit(f);
        }
    }

    fn bus_ref(&self, id: BusId) -> &CanBus {
        match id {
            BusId::PowertrainHs => &self.powertrain,
            BusId::ChassisHs => &self.chassis,
            BusId::BodyMs => &self.body,
            BusId::IsoBus => &self.isobus,
            BusId::Diagnostic => &self.diagnostic,
            BusId::Lin => &self.body, // LIN master reuses body bus object
        }
    }

    fn bus_mut(&mut self, id: BusId) -> &mut CanBus {
        match id {
            BusId::PowertrainHs => &mut self.powertrain,
            BusId::ChassisHs => &mut self.chassis,
            BusId::BodyMs => &mut self.body,
            BusId::IsoBus => &mut self.isobus,
            BusId::Diagnostic => &mut self.diagnostic,
            BusId::Lin => &mut self.body,
        }
    }

    /// All frames from all buses received this cycle for a given SA
    pub fn receive_for_sa(&self, sa: u8) -> Vec<&J1939Frame> {
        let bus_id = self
            .nodes
            .get(&sa)
            .map(|n| n.bus)
            .unwrap_or(BusId::PowertrainHs);
        self.bus_ref(bus_id).receive_for(sa).collect()
    }

    /// Combined error log from all buses
    pub fn all_errors(&self) -> impl Iterator<Item = &BusError> {
        self.powertrain
            .error_log
            .iter()
            .chain(self.chassis.error_log.iter())
            .chain(self.body.error_log.iter())
            .chain(self.isobus.error_log.iter())
    }

    pub fn online_count(&self) -> usize {
        self.nodes.values().filter(|n| n.online).count()
    }
    pub fn total_errors(&self) -> u64 {
        self.powertrain.total_errors
            + self.chassis.total_errors
            + self.body.total_errors
            + self.isobus.total_errors
    }

    pub fn per_bus_health_01(&self, bus: BusId) -> f64 {
        let b = self.bus_ref(bus);
        let mut score = 1.0;
        score -= (b.bus_load_pct / 100.0).powf(1.3) * 0.25;
        score -= (b.total_errors as f64 / 2000.0).min(0.35);
        score -= match b.state {
            BusState::ErrorActive => 0.0,
            BusState::ErrorPassive => 0.30,
            BusState::BusOff => 0.85,
        };
        score.clamp(0.0, 1.0)
    }

    pub fn network_health_score_01(&self) -> f64 {
        let buses = [
            BusId::PowertrainHs,
            BusId::ChassisHs,
            BusId::BodyMs,
            BusId::IsoBus,
            BusId::Diagnostic,
        ];
        let avg_bus = buses
            .iter()
            .map(|b| self.per_bus_health_01(*b))
            .sum::<f64>()
            / buses.len() as f64;
        let online_ratio = if self.nodes.is_empty() {
            1.0
        } else {
            self.online_count() as f64 / self.nodes.len() as f64
        };
        (avg_bus * 0.82 + online_ratio * 0.18).clamp(0.0, 1.0)
    }

    pub fn snapshot(&self) -> CanNetworkSnapshot {
        let buses = [
            (BusId::PowertrainHs, &self.powertrain),
            (BusId::ChassisHs, &self.chassis),
            (BusId::BodyMs, &self.body),
            (BusId::IsoBus, &self.isobus),
            (BusId::Diagnostic, &self.diagnostic),
        ];
        let bus_vec = buses
            .iter()
            .map(|(id, b)| BusSnapshot {
                bus: format!("{}", id),
                speed_kbps: b.speed.kbps(),
                state: format!("{}", b.state),
                load_pct: b.bus_load_pct,
                total_tx: b.total_tx,
                total_errors: b.total_errors,
                log_depth: b.frame_log.len(),
            })
            .collect();

        CanNetworkSnapshot {
            elapsed_s: self.elapsed,
            health_score_01: self.network_health_score_01(),
            total_frames_all_buses: self.total_frames_all_buses,
            total_errors_all_buses: self.total_errors(),
            online_nodes: self.online_count(),
            buses: bus_vec,
        }
    }

    pub fn export_snapshot_json<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        if let Some(parent) = path.as_ref().parent() {
            fs::create_dir_all(parent)?;
        }
        let data = serde_json::to_string_pretty(&self.snapshot()).map_err(|e| {
            std::io::Error::new(std::io::ErrorKind::Other, format!("json serialize: {}", e))
        })?;
        fs::write(path, data)
    }

    pub fn export_snapshot_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
        if let Some(parent) = path.as_ref().parent() {
            fs::create_dir_all(parent)?;
        }
        let snap = self.snapshot();
        let mut f = fs::File::create(path)?;
        writeln!(
            f,
            "elapsed_s,health_score_01,total_frames_all_buses,total_errors_all_buses,online_nodes"
        )?;
        writeln!(
            f,
            "{:.3},{:.5},{},{},{}",
            snap.elapsed_s,
            snap.health_score_01,
            snap.total_frames_all_buses,
            snap.total_errors_all_buses,
            snap.online_nodes
        )?;
        writeln!(f)?;
        writeln!(
            f,
            "bus,speed_kbps,state,load_pct,total_tx,total_errors,log_depth"
        )?;
        for b in snap.buses {
            writeln!(
                f,
                "\"{}\",{},\"{}\",{:.4},{},{},{}",
                b.bus, b.speed_kbps, b.state, b.load_pct, b.total_tx, b.total_errors, b.log_depth
            )?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arbitration_orders_by_can_id() {
        let mut bus = CanBus::new(BusId::PowertrainHs, CanSpeed::Kbps500);
        let mut nodes: HashMap<u8, CanNode> = HashMap::new();
        nodes.insert(
            addr::ECM_1,
            CanNode::new(addr::ECM_1, "ECM", BusId::PowertrainHs),
        );
        nodes.insert(
            addr::TRANSMISSION,
            CanNode::new(addr::TRANSMISSION, "TCM", BusId::PowertrainHs),
        );

        let id_low = J1939Frame::build_id(2, pgn::EBC1, addr::TRANSMISSION, 0xFF);
        let id_high = J1939Frame::build_id(6, pgn::DM1, addr::ECM_1, 0xFF);
        bus.transmit(J1939Frame::from_raw(1.0, id_high, &[0; 8]));
        bus.transmit(J1939Frame::from_raw(1.0, id_low, &[0; 8]));
        bus.tick(&mut nodes, 0.01);

        assert_eq!(bus.dispatched.len(), 2);
        assert!(bus.dispatched[0].raw_id < bus.dispatched[1].raw_id);
    }

    #[test]
    fn scheduler_jitter_stays_in_expected_band() {
        let mut s = MessageSchedule::new(
            pgn::EEC1,
            addr::ECM_1,
            0xFF,
            BusId::PowertrainHs,
            100.0,
            3,
            "test",
        );
        s.jitter_ms = 4.0;
        let mut t_ms = 0.0;
        let mut fire_times: Vec<f64> = Vec::new();
        while fire_times.len() < 12 {
            t_ms += 1.0;
            if s.should_fire(1.0, 0.5) {
                fire_times.push(t_ms);
            }
        }
        for w in fire_times.windows(2) {
            let dt = w[1] - w[0];
            assert!((98.0..=104.5).contains(&dt), "interval {} out of band", dt);
        }
    }

    #[test]
    fn routed_message_appears_next_tick_on_destination_bus() {
        let mut net = CanNetwork::new();
        net.set_online(addr::ECM_1);
        net.set_online(addr::INSTRUMENT);

        let id = J1939Frame::build_id(3, pgn::EEC1, addr::ECM_1, 0xFF);
        let frame = J1939Frame::from_raw(0.0, id, &[0xFF; 8]);
        net.transmit(frame);
        net.tick(0.01);

        let body_after_first = net.body.total_tx;
        net.tick(0.01);
        let body_after_second = net.body.total_tx;
        assert!(
            body_after_second > body_after_first,
            "expected routed frame on next tick"
        );
    }
}
