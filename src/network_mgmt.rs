//! Network Management — OSEK NM / AUTOSAR NM (simplified)
//!
//! In a real vehicle, Network Management coordinates ECU lifecycle:
//!   • When the bus goes idle (all ECUs stop transmitting), the bus transitions
//!     to "Bus Sleep" to save power.
//!   • Any ECU needing the bus wakes it up by sending NM frames.
//!   • ECUs that need to communicate request "Network Requested" state.
//!   • Coordinated shutdown: every ECU confirms it is ready to sleep before
//!     the gateway allows the bus to go off.
//!
//! OSEK NM (ISO 17356-5) — used in heavy machinery / trucks
//! AUTOSAR NM (AUTOSAR_SWS_NetworkManagement) — newer passenger cars
//!
//! State machine per ECU:
//!   BusSleep → Prepare (wake) → Normal → Ready Sleep → Bus Sleep
//!
//! PGN used: J1939 Prop-B in this simulation; real OSEK NM uses CAN IDs 0x400-0x7FF

// ── NM Node State ────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NmState {
    /// ECU is powered off / uninitialized
    Unpowered,
    /// Bus was sleeping; this ECU initiated wakeup
    Initializing,
    /// ECU is active and requesting bus (sending NM frames)
    NormalOperation,
    /// ECU has finished its work and is ready to allow sleep
    ReadyToSleep,
    /// Bus is in sleep mode — low power, no communication
    BusSleep,
}

impl std::fmt::Display for NmState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NmState::Unpowered => write!(f, "UNPOWERED "),
            NmState::Initializing => write!(f, "INIT      "),
            NmState::NormalOperation => write!(f, "NORMAL ✓  "),
            NmState::ReadyToSleep => write!(f, "RDY-SLEEP "),
            NmState::BusSleep => write!(f, "BUS-SLEEP "),
        }
    }
}

// ── NM Frame type ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NmFrameType {
    /// ECU announces it is alive and needs the bus
    Alive,
    /// ECU announces it is ready to sleep (waiting for all nodes)
    Ring,
    /// ECU is waking up the bus
    Wakeup,
    /// Limphome: bus failure, ECU is in limp-home state
    Limphome,
}

// ── NM Node record (one per ECU) ─────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct NmNode {
    pub sa: u8,
    pub name: &'static str,
    pub state: NmState,
    /// Last time we received a NM frame from this node
    pub last_nm_ts: f64,
    /// True if this node has confirmed ready-to-sleep
    pub sleep_confirmed: bool,
    /// True if this node is a "network coordinator" (makes final sleep decision)
    pub is_coordinator: bool,
    pub wakeup_reason: Option<&'static str>,
}

impl NmNode {
    pub fn new(sa: u8, name: &'static str, is_coordinator: bool) -> Self {
        NmNode {
            sa,
            name,
            state: NmState::Unpowered,
            last_nm_ts: 0.0,
            sleep_confirmed: false,
            is_coordinator,
            wakeup_reason: None,
        }
    }

    pub fn is_online(&self) -> bool {
        matches!(self.state, NmState::NormalOperation | NmState::ReadyToSleep)
    }
}

// ── Bus NM State ──────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusNmState {
    /// All nodes powered off
    Off,
    /// Bus waking up (transition from sleep)
    WakingUp,
    /// At least one node needs the bus active
    Active,
    /// All active nodes have requested sleep — counting down
    PrepShutdown,
    /// Bus is silent, all nodes in sleep mode
    Sleep,
}

impl std::fmt::Display for BusNmState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BusNmState::Off => write!(f, "OFF         "),
            BusNmState::WakingUp => write!(f, "WAKING UP   "),
            BusNmState::Active => write!(f, "ACTIVE ✓    "),
            BusNmState::PrepShutdown => write!(f, "PREP SHUTDN "),
            BusNmState::Sleep => write!(f, "BUS SLEEP   "),
        }
    }
}

// ── Network Manager ───────────────────────────────────────────────────────────

pub struct NetworkManager {
    pub nodes: Vec<NmNode>,
    pub bus_state: BusNmState,

    /// Timeout before declaring a node dead (no NM frame received)
    pub node_timeout_s: f64,
    /// How long all nodes must agree to sleep before bus goes off
    pub sleep_countdown_s: f64,
    sleep_timer: f64,

    /// How long bus has been in wakeup phase
    wakeup_timer: f64,

    /// Elapsed simulation time
    elapsed: f64,

    /// Log of recent NM events
    pub event_log: Vec<NmEvent>,
}

#[derive(Debug, Clone)]
pub struct NmEvent {
    pub timestamp: f64,
    pub sa: u8,
    pub node_name: &'static str,
    pub from: NmState,
    pub to: NmState,
    pub reason: &'static str,
}

impl NetworkManager {
    pub fn new() -> Self {
        // Register all ECUs that participate in NM
        // In real OSEK NM, each ECU has a "node ID" separate from J1939 SA
        let nodes = vec![
            NmNode::new(0x00, "ECM #1", false),
            NmNode::new(0x03, "TCM", false),
            NmNode::new(0x27, "BCM/CAB", true), // BCM is typically NM coordinator
            NmNode::new(0x1C, "ICM/DASH", false),
            NmNode::new(0x1E, "HCM", false),
            NmNode::new(0x0B, "ABS/ESP", false),
            NmNode::new(0x26, "ISOBUS-VT", false),
        ];

        NetworkManager {
            nodes,
            bus_state: BusNmState::Off,
            node_timeout_s: 0.5,
            sleep_countdown_s: 2.0,
            sleep_timer: 0.0,
            wakeup_timer: 0.0,
            elapsed: 0.0,
            event_log: Vec::new(),
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Tick. `powered_sas` = list of SA addresses that currently have power.
    pub fn tick(&mut self, powered_sas: &[u8], dt: f64) {
        self.elapsed += dt;

        // ─ Power state transitions ────────────────────────────────────────────
        for node in &mut self.nodes {
            let powered = powered_sas.contains(&node.sa);
            match node.state {
                NmState::Unpowered if powered => {
                    node.state = NmState::Initializing;
                    node.last_nm_ts = self.elapsed;
                    node.sleep_confirmed = false;
                }
                NmState::Initializing => {
                    // ~200 ms init time before sending first NM frame
                    if self.elapsed - node.last_nm_ts >= 0.2 {
                        node.state = NmState::NormalOperation;
                        node.last_nm_ts = self.elapsed;
                    }
                }
                NmState::NormalOperation if !powered => {
                    node.state = NmState::Unpowered;
                }
                NmState::BusSleep if powered => {
                    // Wakeup: any event (ignition, CAN frame) brings node back
                    node.state = NmState::Initializing;
                    node.last_nm_ts = self.elapsed;
                    node.wakeup_reason = Some("IGN ON");
                }
                _ => {}
            }

            // NM heartbeat: each online node sends NM frame every 200 ms
            if node.state == NmState::NormalOperation {
                node.last_nm_ts = self.elapsed; // simulated heartbeat
            }

            // Timeout detection: node went silent
            let silent_for = self.elapsed - node.last_nm_ts;
            if node.state == NmState::NormalOperation && silent_for > self.node_timeout_s {
                let prev = node.state;
                node.state = NmState::BusSleep; // node seems dead
                self.event_log.push(NmEvent {
                    timestamp: self.elapsed,
                    sa: node.sa,
                    node_name: node.name,
                    from: prev,
                    to: NmState::BusSleep,
                    reason: "NM timeout",
                });
            }
        }

        // ─ Bus state machine ──────────────────────────────────────────────────
        let any_active = self
            .nodes
            .iter()
            .any(|n| n.state == NmState::NormalOperation);
        let _all_ready = !self.nodes.is_empty()
            && self
                .nodes
                .iter()
                .filter(|n| n.is_online())
                .all(|n| n.sleep_confirmed || n.state == NmState::ReadyToSleep);

        self.bus_state = match self.bus_state {
            BusNmState::Off => {
                if any_active {
                    self.wakeup_timer = 0.0;
                    BusNmState::WakingUp
                } else {
                    BusNmState::Off
                }
            }
            BusNmState::WakingUp => {
                self.wakeup_timer += dt;
                if self.wakeup_timer >= 0.1 {
                    BusNmState::Active
                } else {
                    BusNmState::WakingUp
                }
            }
            BusNmState::Active => {
                if !any_active {
                    self.sleep_timer = 0.0;
                    BusNmState::PrepShutdown
                } else {
                    BusNmState::Active
                }
            }
            BusNmState::PrepShutdown => {
                self.sleep_timer += dt;
                if any_active {
                    BusNmState::Active // someone woke up again
                } else if self.sleep_timer >= self.sleep_countdown_s {
                    // All nodes confirmed sleep
                    for node in &mut self.nodes {
                        if node.state == NmState::NormalOperation {
                            node.state = NmState::BusSleep;
                        }
                    }
                    BusNmState::Sleep
                } else {
                    BusNmState::PrepShutdown
                }
            }
            BusNmState::Sleep => {
                if any_active {
                    BusNmState::WakingUp
                } else {
                    BusNmState::Sleep
                }
            }
        };
    }

    // ─────────────────────────────────────────────────────────────────────────
    /// Called when an ECU is done and ready for bus sleep
    pub fn request_sleep(&mut self, sa: u8) {
        if let Some(node) = self.nodes.iter_mut().find(|n| n.sa == sa) {
            if node.state == NmState::NormalOperation {
                node.state = NmState::ReadyToSleep;
                node.sleep_confirmed = true;
            }
        }
    }

    /// Wake the bus (e.g., key-on event)
    pub fn wake_all(&mut self, reason: &'static str) {
        for node in &mut self.nodes {
            if node.state == NmState::BusSleep {
                node.state = NmState::Initializing;
                node.last_nm_ts = self.elapsed;
                node.wakeup_reason = Some(reason);
                node.sleep_confirmed = false;
            }
        }
        self.bus_state = BusNmState::WakingUp;
        self.wakeup_timer = 0.0;
    }

    pub fn node_state(&self, sa: u8) -> Option<NmState> {
        self.nodes.iter().find(|n| n.sa == sa).map(|n| n.state)
    }

    pub fn active_count(&self) -> usize {
        self.nodes.iter().filter(|n| n.is_online()).count()
    }

    pub fn total_nodes(&self) -> usize {
        self.nodes.len()
    }
}
