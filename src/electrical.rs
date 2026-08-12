use crate::{ecu_bcm::Fuse, IgnitionState};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElectricalLine {
    Line30,
    Line15Acc,
    Line15Ign,
    Line31,
    HsCan,
    MsCan,
}

impl std::fmt::Display for ElectricalLine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            ElectricalLine::Line30 => "30",
            ElectricalLine::Line15Acc => "15-ACC",
            ElectricalLine::Line15Ign => "15-IGN",
            ElectricalLine::Line31 => "31-GND",
            ElectricalLine::HsCan => "HS-CAN",
            ElectricalLine::MsCan => "MS-CAN",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElectricalBranchKind {
    Supply,
    Load,
    Network,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElectricalFaultMode {
    None,
    OpenCircuit,
    ShortToGround,
    HighResistance,
}

impl std::fmt::Display for ElectricalFaultMode {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            ElectricalFaultMode::None => "OK",
            ElectricalFaultMode::OpenCircuit => "OPEN",
            ElectricalFaultMode::ShortToGround => "SHORT-GND",
            ElectricalFaultMode::HighResistance => "HI-R",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ElectricalControlPoint {
    MainGround,
    AccessoryRelay,
    IgnitionRelay,
}

impl std::fmt::Display for ElectricalControlPoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            ElectricalControlPoint::MainGround => "MAIN-GND",
            ElectricalControlPoint::AccessoryRelay => "ACC-RELAY",
            ElectricalControlPoint::IgnitionRelay => "IGN-RELAY",
        };
        write!(f, "{}", s)
    }
}

#[derive(Debug, Clone)]
pub struct ElectricalBranch {
    pub id: &'static str,
    pub from: &'static str,
    pub to: &'static str,
    pub line: ElectricalLine,
    pub kind: ElectricalBranchKind,
    pub fuse_id: &'static str,
    pub fuse_rating_a: f64,
    pub gauge_mm2: f64,
    pub length_m: f64,
    pub requested_current_a: f64,
    pub actual_current_a: f64,
    pub voltage_v: f64,
    pub capacitance_nf: f64,
    pub resistance_ohm: f64,
    pub fault: ElectricalFaultMode,
    pub blown: bool,
    pub energized: bool,
    pub target_sa: Option<u8>,
    pub note: &'static str,
}

impl ElectricalBranch {
    #[allow(clippy::too_many_arguments)]
    fn new(
        id: &'static str,
        from: &'static str,
        to: &'static str,
        line: ElectricalLine,
        kind: ElectricalBranchKind,
        fuse_id: &'static str,
        fuse_rating_a: f64,
        gauge_mm2: f64,
        length_m: f64,
        target_sa: Option<u8>,
        note: &'static str,
    ) -> Self {
        let capacitance_nf = length_m * ElectricalSystem::capacitance_nf_per_m(gauge_mm2, kind == ElectricalBranchKind::Network);
        let resistance_ohm = ElectricalSystem::wire_resistance_ohm(length_m, gauge_mm2, kind == ElectricalBranchKind::Network);
        Self {
            id,
            from,
            to,
            line,
            kind,
            fuse_id,
            fuse_rating_a,
            gauge_mm2,
            length_m,
            requested_current_a: 0.0,
            actual_current_a: 0.0,
            voltage_v: 0.0,
            capacitance_nf,
            resistance_ohm,
            fault: ElectricalFaultMode::None,
            blown: false,
            energized: false,
            target_sa,
            note,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ElectricalRelayState {
    pub accessory_closed: bool,
    pub ignition_closed: bool,
}

#[derive(Debug, Clone)]
pub struct ElectricalSnapshot {
    pub line30_v: f64,
    pub line15_acc_v: f64,
    pub line15_ign_v: f64,
    pub line31_v: f64,
    pub source_v: f64,
    pub total_current_a: f64,
    pub ground_drop_v: f64,
}

impl Default for ElectricalSnapshot {
    fn default() -> Self {
        Self {
            line30_v: 12.6,
            line15_acc_v: 0.0,
            line15_ign_v: 0.0,
            line31_v: 0.0,
            source_v: 12.6,
            total_current_a: 0.0,
            ground_drop_v: 0.0,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct ElectricalLoadRequest {
    pub bcm_supply_a: f64,
    pub vcm_supply_a: f64,
    pub ecm_supply_a: f64,
    pub tcm_supply_a: f64,
    pub abs_supply_a: f64,
    pub hcm_supply_a: f64,
    pub icm_supply_a: f64,
    pub body_load_a: f64,
    pub hs_can_a: f64,
    pub ms_can_a: f64,
}

#[derive(Debug, Clone)]
pub struct ElectricalSystem {
    pub relays: ElectricalRelayState,
    pub snapshot: ElectricalSnapshot,
    pub branches: Vec<ElectricalBranch>,
    pub main_ground_fault: ElectricalFaultMode,
    pub accessory_relay_fault: ElectricalFaultMode,
    pub ignition_relay_fault: ElectricalFaultMode,
}

impl Default for ElectricalSystem {
    fn default() -> Self {
        Self::new()
    }
}

impl ElectricalSystem {
    pub fn new() -> Self {
        Self {
            relays: ElectricalRelayState::default(),
            snapshot: ElectricalSnapshot::default(),
            branches: vec![
                ElectricalBranch::new("BAT-MAIN", "Battery", "Main Fuse Box", ElectricalLine::Line30, ElectricalBranchKind::Supply, "MEGA175", 175.0, 35.0, 1.8, None, "primary battery feed"),
                ElectricalBranch::new("ALT-B+", "Alternator", "Battery/Main Bus", ElectricalLine::Line30, ElectricalBranchKind::Supply, "ALT150", 150.0, 25.0, 1.4, None, "charging branch"),
                ElectricalBranch::new("BCM-PWR", "Main Fuse Box", "BCM", ElectricalLine::Line15Acc, ElectricalBranchKind::Supply, "BCM40", 40.0, 6.0, 2.8, Some(crate::j1939::addr::CAB), "body controller supply"),
                ElectricalBranch::new("ICM-PWR", "BCM", "ICM", ElectricalLine::Line15Acc, ElectricalBranchKind::Supply, "DASH7.5", 7.5, 1.0, 2.2, Some(crate::j1939::addr::INSTRUMENT), "instrument feed"),
                ElectricalBranch::new("VCM-PWR", "Main Fuse Box", "VCM", ElectricalLine::Line15Ign, ElectricalBranchKind::Supply, "VCM15", 15.0, 2.5, 3.4, None, "gateway and power coordination"),
                ElectricalBranch::new("ECM-PWR", "Main Fuse Box", "ECM", ElectricalLine::Line15Ign, ElectricalBranchKind::Supply, "ECM15", 15.0, 2.5, 3.8, Some(crate::j1939::addr::ECM_1), "engine controller branch"),
                ElectricalBranch::new("TCM-PWR", "Main Fuse Box", "TCM", ElectricalLine::Line15Ign, ElectricalBranchKind::Supply, "TCM10", 10.0, 1.5, 4.1, Some(crate::j1939::addr::TRANSMISSION), "transmission controller branch"),
                ElectricalBranch::new("ABS-PWR", "Main Fuse Box", "ABS/ESP", ElectricalLine::Line15Ign, ElectricalBranchKind::Supply, "ABS20", 20.0, 2.5, 4.6, Some(crate::j1939::addr::BRAKES), "braking controller branch"),
                ElectricalBranch::new("HCM-PWR", "Main Fuse Box", "HCM", ElectricalLine::Line15Ign, ElectricalBranchKind::Supply, "HCM15", 15.0, 2.5, 4.2, Some(crate::j1939::addr::HITCH), "hydraulic controller branch"),
                ElectricalBranch::new("BODY-LOAD", "BCM", "Lights/HVAC/Wiper/Horn", ElectricalLine::Line15Acc, ElectricalBranchKind::Load, "BODY40", 40.0, 4.0, 5.0, None, "aggregate body loads"),
                ElectricalBranch::new("HS-CAN-1", "VCM", "ECM/TCM/ABS/HCM", ElectricalLine::HsCan, ElectricalBranchKind::Network, "120R x2", 1.0, 0.75, 8.5, None, "500 kbps twisted pair backbone"),
                ElectricalBranch::new("MS-CAN-BODY", "BCM", "ICM/body domain", ElectricalLine::MsCan, ElectricalBranchKind::Network, "120R x2", 1.0, 0.5, 6.0, None, "250 kbps twisted pair backbone"),
            ],
            main_ground_fault: ElectricalFaultMode::None,
            accessory_relay_fault: ElectricalFaultMode::None,
            ignition_relay_fault: ElectricalFaultMode::None,
        }
    }

    pub fn wire_resistance_ohm(length_m: f64, gauge_mm2: f64, twisted_pair: bool) -> f64 {
        let rho = 0.0175;
        let loop_factor = if twisted_pair { 2.0 } else { 2.2 };
        (rho * length_m * loop_factor / gauge_mm2.max(0.35)).max(0.0005)
    }

    pub fn capacitance_nf_per_m(gauge_mm2: f64, twisted_pair: bool) -> f64 {
        if twisted_pair {
            0.05
        } else if gauge_mm2 <= 1.0 {
            0.09
        } else if gauge_mm2 <= 2.5 {
            0.11
        } else if gauge_mm2 <= 6.0 {
            0.13
        } else {
            0.16
        }
    }

    fn requested_current_for_branch(branch_id: &str, loads: &ElectricalLoadRequest) -> f64 {
        match branch_id {
            "BAT-MAIN" => loads.bcm_supply_a
                + loads.vcm_supply_a
                + loads.ecm_supply_a
                + loads.tcm_supply_a
                + loads.abs_supply_a
                + loads.hcm_supply_a
                + loads.icm_supply_a
                + loads.body_load_a,
            "ALT-B+" => 0.0,
            "BCM-PWR" => loads.bcm_supply_a,
            "ICM-PWR" => loads.icm_supply_a,
            "VCM-PWR" => loads.vcm_supply_a,
            "ECM-PWR" => loads.ecm_supply_a,
            "TCM-PWR" => loads.tcm_supply_a,
            "ABS-PWR" => loads.abs_supply_a,
            "HCM-PWR" => loads.hcm_supply_a,
            "BODY-LOAD" => loads.body_load_a,
            "HS-CAN-1" => loads.hs_can_a,
            "MS-CAN-BODY" => loads.ms_can_a,
            _ => 0.0,
        }
    }

    pub fn set_branch_fault(&mut self, branch_id: &str, fault: ElectricalFaultMode) {
        if let Some(branch) = self.branches.iter_mut().find(|b| b.id == branch_id) {
            branch.fault = fault;
            if fault == ElectricalFaultMode::None {
                branch.blown = false;
            }
        }
    }

    pub fn set_control_fault(&mut self, point: ElectricalControlPoint, fault: ElectricalFaultMode) {
        match point {
            ElectricalControlPoint::MainGround => self.main_ground_fault = fault,
            ElectricalControlPoint::AccessoryRelay => self.accessory_relay_fault = fault,
            ElectricalControlPoint::IgnitionRelay => self.ignition_relay_fault = fault,
        }
    }

    pub fn reset_faults(&mut self) {
        self.main_ground_fault = ElectricalFaultMode::None;
        self.accessory_relay_fault = ElectricalFaultMode::None;
        self.ignition_relay_fault = ElectricalFaultMode::None;
        for branch in &mut self.branches {
            branch.fault = ElectricalFaultMode::None;
            branch.blown = false;
        }
    }

    pub fn tick(
        &mut self,
        ignition: IgnitionState,
        source_voltage_v: f64,
        alternator_voltage_v: f64,
        loads: &ElectricalLoadRequest,
        bcm_fuses: &[Fuse],
        dt: f64,
    ) {
        self.relays.accessory_closed = matches!(ignition, IgnitionState::Accessory | IgnitionState::On | IgnitionState::Cranking | IgnitionState::Running)
            && self.accessory_relay_fault != ElectricalFaultMode::OpenCircuit;
        self.relays.ignition_closed = matches!(ignition, IgnitionState::On | IgnitionState::Cranking | IgnitionState::Running)
            && self.ignition_relay_fault != ElectricalFaultMode::OpenCircuit;

        let ground_r = match self.main_ground_fault {
            ElectricalFaultMode::None => 0.004,
            ElectricalFaultMode::HighResistance => 0.09,
            ElectricalFaultMode::OpenCircuit => 1.5,
            ElectricalFaultMode::ShortToGround => 0.004,
        };

        self.snapshot.source_v = source_voltage_v.max(alternator_voltage_v).max(0.0);
        self.snapshot.line30_v = self.snapshot.source_v;
        self.snapshot.line15_acc_v = if self.relays.accessory_closed {
            (self.snapshot.line30_v - if self.accessory_relay_fault == ElectricalFaultMode::HighResistance { 1.8 } else { 0.15 }).max(0.0)
        } else {
            0.0
        };
        self.snapshot.line15_ign_v = if self.relays.ignition_closed {
            (self.snapshot.line30_v - if self.ignition_relay_fault == ElectricalFaultMode::HighResistance { 2.2 } else { 0.18 }).max(0.0)
        } else {
            0.0
        };

        let line30_v = self.snapshot.line30_v;
        let line15_acc_v = self.snapshot.line15_acc_v;
        let line15_ign_v = self.snapshot.line15_ign_v;
        let line31_v = self.snapshot.line31_v;
        let mut total_current = 0.0;
        for branch in &mut self.branches {
            branch.requested_current_a = Self::requested_current_for_branch(branch.id, loads);

            if let Some(fuse) = bcm_fuses.iter().find(|f| f.id == branch.id || f.id == branch.fuse_id) {
                if fuse.blown {
                    branch.blown = true;
                }
            }

            let source_v = match branch.line {
                ElectricalLine::Line30 => line30_v,
                ElectricalLine::Line15Acc => line15_acc_v,
                ElectricalLine::Line15Ign => line15_ign_v,
                ElectricalLine::Line31 => line31_v,
                ElectricalLine::HsCan => line15_ign_v,
                ElectricalLine::MsCan => line15_acc_v,
            };
            let extra_r = match branch.fault {
                ElectricalFaultMode::None => 0.0,
                ElectricalFaultMode::HighResistance => 0.35,
                ElectricalFaultMode::OpenCircuit => 1.0e9,
                ElectricalFaultMode::ShortToGround => 0.02,
            };

            if branch.fault == ElectricalFaultMode::OpenCircuit || branch.blown || source_v < 1.0 {
                branch.actual_current_a = 0.0;
                branch.energized = false;
                branch.voltage_v += (0.0 - branch.voltage_v) * (dt / 0.04).min(1.0);
                continue;
            }

            if branch.fault == ElectricalFaultMode::ShortToGround {
                let short_i = source_v / (branch.resistance_ohm + ground_r + extra_r).max(0.005);
                if short_i > branch.fuse_rating_a * 1.15 {
                    branch.blown = true;
                    branch.actual_current_a = 0.0;
                    branch.energized = false;
                    branch.voltage_v = 0.0;
                    continue;
                }
                branch.actual_current_a = short_i;
                branch.energized = true;
            } else {
                branch.actual_current_a = branch.requested_current_a.max(0.0);
                branch.energized = source_v > 9.0 || branch.kind == ElectricalBranchKind::Network;
            }

            total_current += branch.actual_current_a;
            let drop_v = branch.actual_current_a * (branch.resistance_ohm + ground_r + extra_r.min(1.0));
            let target_v = (source_v - drop_v).max(0.0);
            let tau_s = (((branch.resistance_ohm + extra_r.min(1.0)).max(0.005))
                * (branch.capacitance_nf * 1.0e-9))
                .clamp(0.002, 0.08);
            branch.voltage_v += (target_v - branch.voltage_v) * (dt / tau_s).min(1.0);
            if branch.actual_current_a > branch.fuse_rating_a * 1.15 && branch.kind != ElectricalBranchKind::Network {
                branch.blown = true;
                branch.energized = false;
                branch.actual_current_a = 0.0;
                branch.voltage_v = 0.0;
            }
        }

        self.snapshot.total_current_a = total_current;
        self.snapshot.ground_drop_v = total_current * ground_r;
        self.snapshot.line31_v = self.snapshot.ground_drop_v;
    }

    pub fn powered_sas(&self) -> Vec<u8> {
        self.branches
            .iter()
            .filter_map(|b| {
                if b.energized && !b.blown && b.voltage_v > 9.0 {
                    b.target_sa
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn branch(&self, branch_id: &str) -> Option<&ElectricalBranch> {
        self.branches.iter().find(|b| b.id == branch_id)
    }
}

#[cfg(test)]
mod tests {
    use super::{ElectricalControlPoint, ElectricalFaultMode, ElectricalLoadRequest, ElectricalSystem};
    use crate::IgnitionState;

    #[test]
    fn ignition_relay_controls_keyed_ecu_power() {
        let mut es = ElectricalSystem::new();
        let loads = ElectricalLoadRequest {
            ecm_supply_a: 6.0,
            tcm_supply_a: 3.0,
            abs_supply_a: 4.0,
            hcm_supply_a: 3.0,
            bcm_supply_a: 8.0,
            icm_supply_a: 2.0,
            ..Default::default()
        };
        es.tick(IgnitionState::On, 12.6, 14.2, &loads, &[], 0.016);
        assert!(es.powered_sas().contains(&crate::j1939::addr::ECM_1));

        es.set_control_fault(ElectricalControlPoint::IgnitionRelay, ElectricalFaultMode::OpenCircuit);
        es.tick(IgnitionState::On, 12.6, 14.2, &loads, &[], 0.016);
        assert!(!es.powered_sas().contains(&crate::j1939::addr::ECM_1));
        assert!(es.powered_sas().contains(&crate::j1939::addr::CAB));
    }

    #[test]
    fn short_to_ground_blows_ecu_branch() {
        let mut es = ElectricalSystem::new();
        let loads = ElectricalLoadRequest {
            ecm_supply_a: 6.0,
            ..Default::default()
        };
        es.set_branch_fault("ECM-PWR", ElectricalFaultMode::ShortToGround);
        es.tick(IgnitionState::On, 12.4, 14.0, &loads, &[], 0.016);
        let branch = es.branch("ECM-PWR").expect("branch present");
        assert!(branch.blown);
        assert!(!branch.energized);
    }
}