//! Boot Sequence Module — models every step from turning the key to full operation.
//!
//! Implements the real power-up sequence of a heavy machine:
//!   OFF → ACCESSORY → IGN_ON → SELF_TEST → ADDRESS_CLAIM → PRE_START_CHECKS
//!   → CRANKING → START_CONFIRM → RUNNING
//!
//! J1939 Address Claiming (AC) procedure per SAE J1939-81:
//!   1. Node broadcasts its 64-bit NAME on null address (0xFE)
//!   2. Waits 250 ms for conflicts
//!   3. If no conflict: claims the desired SA
//!   4. If conflict: lower NAME wins; loser tries next available SA or goes null

use crate::can_gateway::CanGateway;
use crate::j1939::{self, addr, J1939Frame};

// ── Ignition Key States ──────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum IgnitionState {
    /// Key removed / fully off — no ECU power
    Off,
    /// Key in ACCESSORY position — BCM, radio, accessories
    Accessory,
    /// Key in ON position — all ECUs power up, self-test, address claim
    On,
    /// Key in START (momentary) — starter motor engaged
    Cranking,
    /// Engine running — normal operation
    Running,
}

impl std::fmt::Display for IgnitionState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IgnitionState::Off => write!(f, "  OFF   "),
            IgnitionState::Accessory => write!(f, "  ACC   "),
            IgnitionState::On => write!(f, "  ON    "),
            IgnitionState::Cranking => write!(f, " START  "),
            IgnitionState::Running => write!(f, " RUN ✓  "),
        }
    }
}

// ── Module Boot Stage ────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EcuBootStage {
    /// Power off / not initialised
    Unpowered,
    /// Hardware reset, ROM check, RAM test (~50 ms)
    HardwareInit,
    /// Loading calibration / NVM data (~80 ms)
    LoadingCalibration,
    /// J1939 address claiming — broadcasting NAME, waiting 250 ms
    AddressClaiming,
    /// Address claimed, exchanging presence frames with peers
    Handshaking,
    /// Pre-operational checks (sensors in range, comms OK)
    PreOperational,
    /// Fully operational
    Running,
    /// Fault — cannot enter normal operation
    Fault,
}

impl std::fmt::Display for EcuBootStage {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            EcuBootStage::Unpowered => "UNPOWERED      ",
            EcuBootStage::HardwareInit => "HW INIT        ",
            EcuBootStage::LoadingCalibration => "LOADING CAL    ",
            EcuBootStage::AddressClaiming => "ADDR CLAIM     ",
            EcuBootStage::Handshaking => "HANDSHAKING    ",
            EcuBootStage::PreOperational => "PRE-OPER CHECK ",
            EcuBootStage::Running => "RUNNING ✓      ",
            EcuBootStage::Fault => "FAULT ✗        ",
        };
        write!(f, "{}", s)
    }
}

// ── Per-ECU boot record ──────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct EcuBootRecord {
    pub name: &'static str,
    pub sa: u8,
    /// 64-bit J1939 NAME (unique per device class/instance)
    pub j1939_name: u64,
    pub stage: EcuBootStage,
    /// Accumulated time in the current stage
    pub stage_timer: f64,
    /// When the ECU first transitioned to Running
    pub online_at: Option<f64>,
    pub boot_error: Option<&'static str>,
}

impl EcuBootRecord {
    pub fn new(name: &'static str, sa: u8, j1939_name: u64) -> Self {
        EcuBootRecord {
            name,
            sa,
            j1939_name,
            stage: EcuBootStage::Unpowered,
            stage_timer: 0.0,
            online_at: None,
            boot_error: None,
        }
    }

    pub fn is_online(&self) -> bool {
        self.stage == EcuBootStage::Running
    }

    pub fn is_faulted(&self) -> bool {
        self.stage == EcuBootStage::Fault
    }
}

// ── Boot Event Log ───────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct BootEvent {
    pub timestamp: f64,
    pub ecu_name: &'static str,
    pub event: BootEventKind,
    pub description: String,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BootEventKind {
    PowerOn,
    StageChange,
    AddressClaimed,
    HandshakeOk,
    HandshakeFail,
    PreCheckOk,
    PreCheckFail,
    Running,
    Fault,
}

// ── Pre-Start Safety Checks ──────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct SafetyInterlock {
    pub id: &'static str,
    pub description: &'static str,
    pub satisfied: bool,
    pub blocks_start: bool,
}

// ── Boot Sequence Orchestrator ───────────────────────────────────────────────

pub struct BootSequence {
    pub ignition: IgnitionState,
    pub ecus: Vec<EcuBootRecord>,
    pub event_log: Vec<BootEvent>,
    pub safety_interlocks: Vec<SafetyInterlock>,

    // Global sequence timer (resets each state transition)
    sequence_timer: f64,

    // Are all critical ECUs online?
    pub all_critical_online: bool,

    // Crank inhibit
    pub crank_inhibited: bool,
    pub crank_inhibit_reason: Option<&'static str>,

    // Engine actually running (>400 RPM) — set by ECM
    pub engine_running: bool,

    pub elapsed: f64,
}

impl BootSequence {
    pub fn new() -> Self {
        // Register all ECUs with their J1939 NAMEs.
        // NAME bits: [62:60]=industry(0=global,2=ag), [59:56]=vehicle-system,
        //            [55:52]=function, [47:40]=manufacturer, [28:21]=ECU-instance
        let ecus = vec![
            EcuBootRecord::new("ECM #1", addr::ECM_1, 0x0000_6000_0000_0000),
            EcuBootRecord::new("TCM", addr::TRANSMISSION, 0x0000_6000_0100_0000),
            EcuBootRecord::new("BCM", addr::CAB, 0x0000_6000_0200_0000),
            EcuBootRecord::new("ICM/DASH", addr::INSTRUMENT, 0x0000_6000_0300_0000),
            EcuBootRecord::new("HCM/HITCH", addr::HITCH, 0x0000_6000_0400_0000),
            EcuBootRecord::new("ABS/STAB", addr::BRAKES, 0x0000_6000_0500_0000),
            EcuBootRecord::new("ISOBUS-VT", addr::ISOBUS_VT, 0x0000_6000_0600_0000),
        ];

        // Real machine safety interlocks (prevents engine start if not met)
        let interlocks = vec![
            SafetyInterlock {
                id: "TRANS_NEUTRAL",
                description: "Transmission in neutral/park",
                satisfied: false,
                blocks_start: true,
            },
            SafetyInterlock {
                id: "PARK_BRAKE",
                description: "Parking brake applied",
                satisfied: true,
                blocks_start: false,
            },
            SafetyInterlock {
                id: "SEAT_OCCUPIED",
                description: "Operator seat occupied",
                satisfied: true,
                blocks_start: true,
            },
            SafetyInterlock {
                id: "DOOR_CLOSED",
                description: "Cab door closed",
                satisfied: true,
                blocks_start: false,
            },
            SafetyInterlock {
                id: "PTO_OFF",
                description: "PTO disengaged",
                satisfied: true,
                blocks_start: false,
            },
            SafetyInterlock {
                id: "HITCH_UP",
                description: "Rear hitch raised for road",
                satisfied: true,
                blocks_start: false,
            },
            SafetyInterlock {
                id: "OIL_PRESS_OK",
                description: "Engine oil pressure in range",
                satisfied: true,
                blocks_start: false,
            },
            SafetyInterlock {
                id: "COOLANT_OK",
                description: "Coolant level OK",
                satisfied: true,
                blocks_start: false,
            },
            SafetyInterlock {
                id: "DEF_LEVEL_OK",
                description: "DEF level ≥ 5%",
                satisfied: true,
                blocks_start: false,
            },
        ];

        BootSequence {
            ignition: IgnitionState::Off,
            ecus,
            event_log: Vec::new(),
            safety_interlocks: interlocks,
            sequence_timer: 0.0,
            all_critical_online: false,
            crank_inhibited: false,
            crank_inhibit_reason: None,
            engine_running: false,
            elapsed: 0.0,
        }
    }

    /// Advance the ignition key one step
    pub fn key_advance(&mut self) {
        self.ignition = match self.ignition {
            IgnitionState::Off => IgnitionState::Accessory,
            IgnitionState::Accessory => IgnitionState::On,
            IgnitionState::On => {
                if self.crank_inhibited {
                    IgnitionState::On
                }
                // can't start
                else {
                    IgnitionState::Cranking
                }
            }
            IgnitionState::Cranking => IgnitionState::On, // spring-return
            IgnitionState::Running => IgnitionState::On,  // engine still running
        };
        self.sequence_timer = 0.0;
        self.push_event(
            "IGNITION",
            BootEventKind::PowerOn,
            format!("Key advanced to {:?}", self.ignition),
        );
    }

    /// Turn key off
    pub fn key_off(&mut self) {
        self.ignition = IgnitionState::Off;
        self.engine_running = false;
        // Power down all ECUs
        for ecu in &mut self.ecus {
            ecu.stage = EcuBootStage::Unpowered;
            ecu.stage_timer = 0.0;
            ecu.online_at = None;
        }
        self.push_event(
            "IGNITION",
            BootEventKind::PowerOn,
            "Key OFF — all ECUs unpowered".into(),
        );
    }

    /// Update the boot sequence state machine. Returns CAN frames to transmit.
    pub fn tick(
        &mut self,
        dt: f64,
        gateway: &mut CanGateway,
        trans_neutral: bool,
    ) -> Vec<J1939Frame> {
        self.elapsed += dt;
        self.sequence_timer += dt;

        // Update safety interlocks
        self.update_interlocks(trans_neutral);
        self.check_crank_inhibit();

        let mut frames: Vec<J1939Frame> = Vec::new();

        match self.ignition {
            IgnitionState::Off => {
                // Nothing to do — all ECUs off
            }

            IgnitionState::Accessory => {
                // Only BCM and ICM get power in accessory
                self.boot_ecu(addr::CAB, dt, &mut frames);
                self.boot_ecu(addr::INSTRUMENT, dt, &mut frames);
            }

            IgnitionState::On | IgnitionState::Cranking | IgnitionState::Running => {
                // All ECUs boot in parallel
                for i in 0..self.ecus.len() {
                    let sa = self.ecus[i].sa;
                    self.boot_ecu(sa, dt, &mut frames);
                }
                // Check if all critical ECUs are online
                self.check_all_critical_online();
            }
        }

        // Crank → Running transition (engine fires)
        if self.ignition == IgnitionState::Cranking && self.engine_running {
            self.ignition = IgnitionState::Running;
            self.push_event(
                "IGNITION",
                BootEventKind::Running,
                "Engine fired — transition to RUNNING".into(),
            );
        }

        // Dispatch frames to gateway
        for f in &frames {
            gateway.transmit(f.clone());
        }
        frames
    }

    /// Run one ECU through its boot state machine
    fn boot_ecu(&mut self, sa: u8, dt: f64, frames: &mut Vec<J1939Frame>) {
        let idx = match self.ecus.iter().position(|e| e.sa == sa) {
            Some(i) => i,
            None => return,
        };
        let ts = self.elapsed;

        // Copy all data we need BEFORE taking a mutable borrow of self
        let stage = self.ecus[idx].stage;
        let stage_timer = self.ecus[idx].stage_timer;
        let ecu_name = self.ecus[idx].name;
        let ecu_sa = self.ecus[idx].sa;
        let j1939_name = self.ecus[idx].j1939_name;

        self.ecus[idx].stage_timer += dt;

        match stage {
            EcuBootStage::Unpowered => {
                self.ecus[idx].stage = EcuBootStage::HardwareInit;
                self.ecus[idx].stage_timer = 0.0;
                self.ecus[idx].boot_error = None;
            }
            EcuBootStage::HardwareInit => {
                if stage_timer >= 0.050 {
                    self.ecus[idx].stage = EcuBootStage::LoadingCalibration;
                    self.ecus[idx].stage_timer = 0.0;
                    self.push_event(
                        ecu_name,
                        BootEventKind::StageChange,
                        format!("{}: HW init OK → loading calibration", ecu_name),
                    );
                }
            }
            EcuBootStage::LoadingCalibration => {
                if stage_timer >= 0.080 {
                    self.ecus[idx].stage = EcuBootStage::AddressClaiming;
                    self.ecus[idx].stage_timer = 0.0;
                    self.push_event(
                        ecu_name,
                        BootEventKind::StageChange,
                        format!(
                            "{}: Calibration loaded → claiming SA 0x{:02X}",
                            ecu_name, ecu_sa
                        ),
                    );
                    frames.push(build_address_claim(ts, j1939_name, ecu_sa));
                }
            }
            EcuBootStage::AddressClaiming => {
                if stage_timer >= 0.250 {
                    self.ecus[idx].stage = EcuBootStage::Handshaking;
                    self.ecus[idx].stage_timer = 0.0;
                    self.push_event(
                        ecu_name,
                        BootEventKind::AddressClaimed,
                        format!(
                            "{}: Address 0x{:02X} claimed successfully",
                            ecu_name, ecu_sa
                        ),
                    );
                    frames.push(build_address_claim(ts, j1939_name, ecu_sa));
                }
            }
            EcuBootStage::Handshaking => {
                if stage_timer >= 0.150 {
                    self.ecus[idx].stage = EcuBootStage::PreOperational;
                    self.ecus[idx].stage_timer = 0.0;
                    self.push_event(
                        ecu_name,
                        BootEventKind::HandshakeOk,
                        format!("{}: Peer handshake OK", ecu_name),
                    );
                }
            }
            EcuBootStage::PreOperational => {
                if stage_timer >= 0.200 {
                    self.ecus[idx].stage = EcuBootStage::Running;
                    self.ecus[idx].stage_timer = 0.0;
                    self.ecus[idx].online_at = Some(ts);
                    self.push_event(
                        ecu_name,
                        BootEventKind::Running,
                        format!("{}: Pre-op checks passed → RUNNING at {:.3}s", ecu_name, ts),
                    );
                }
            }

            EcuBootStage::Running | EcuBootStage::Fault => {
                // Nothing more to do in boot state machine
            }
        }
    }

    fn update_interlocks(&mut self, trans_neutral: bool) {
        for il in &mut self.safety_interlocks {
            match il.id {
                "TRANS_NEUTRAL" => il.satisfied = trans_neutral,
                _ => {} // others set externally
            }
        }
    }

    fn check_crank_inhibit(&mut self) {
        self.crank_inhibited = false;
        self.crank_inhibit_reason = None;
        for il in &self.safety_interlocks {
            if il.blocks_start && !il.satisfied {
                self.crank_inhibited = true;
                self.crank_inhibit_reason = Some(il.id);
                break;
            }
        }
    }

    fn check_all_critical_online(&mut self) {
        // Critical ECUs: ECM and TCM
        let ecm_ok = self
            .ecus
            .iter()
            .any(|e| e.sa == addr::ECM_1 && e.is_online());
        let tcm_ok = self
            .ecus
            .iter()
            .any(|e| e.sa == addr::TRANSMISSION && e.is_online());
        self.all_critical_online = ecm_ok && tcm_ok;
    }

    fn push_event(&mut self, ecu: &'static str, kind: BootEventKind, desc: String) {
        self.event_log.push(BootEvent {
            timestamp: self.elapsed,
            ecu_name: ecu,
            event: kind,
            description: desc,
        });
        // Keep only last 100 events
        if self.event_log.len() > 100 {
            self.event_log.remove(0);
        }
    }

    /// Get ECU record by SA
    pub fn ecu_by_sa(&self, sa: u8) -> Option<&EcuBootRecord> {
        self.ecus.iter().find(|e| e.sa == sa)
    }
}

// ── J1939 Address Claim frame builder ────────────────────────────────────────

/// Build a J1939 Address Claim (PGN 0xEE00) frame carrying the 64-bit NAME.
fn build_address_claim(ts: f64, name: u64, sa: u8) -> J1939Frame {
    let mut data = [0u8; 8];
    // NAME is 8 bytes, little-endian
    for i in 0..8 {
        data[i] = ((name >> (i * 8)) & 0xFF) as u8;
    }
    // PGN 60928 = 0xEE00, priority 6
    let raw_id = J1939Frame::build_id(6, j1939::pgn::ECAN, sa, 0xFF);
    J1939Frame::from_raw(ts, raw_id, &data)
}
