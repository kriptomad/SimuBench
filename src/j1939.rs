//! J1939 Protocol Stack — SAE J1939 / ISO 11783 (ISOBUS)
//!
//! 29-bit CAN ID layout:
//!  [28:26] Priority (0=highest, 7=lowest)
//!  [25]    Reserved
//!  [24]    Data Page (DP)
//!  [23:16] PDU Format (PF)
//!  [15:8]  PDU Specific (PS) — DA if PF<0xF0, group-ext if PF≥0xF0
//!  [7:0]   Source Address (SA)

use std::collections::VecDeque;

// ── Source Address Constants ─────────────────────────────────────────────────
pub mod addr {
    pub const ECM_1: u8 = 0x00;
    pub const ECM_2: u8 = 0x01;
    pub const TURBOCHARGER: u8 = 0x02;
    pub const TRANSMISSION: u8 = 0x03;
    pub const BRAKES: u8 = 0x0B;
    pub const TASK_CTRL: u8 = 0x0D;
    pub const HEADWAY: u8 = 0x20;
    pub const IMPLEMENT: u8 = 0x25;
    pub const ISOBUS_VT: u8 = 0x26;
    pub const CAB: u8 = 0x27;
    pub const ARMREST: u8 = 0x28;
    pub const INSTRUMENT: u8 = 0x1C;
    pub const HITCH: u8 = 0x1E;
    pub const NULL: u8 = 0xFE;
    pub const BROADCAST: u8 = 0xFF;
}

// ── PGN Constants ───────────────────────────────────────────────────────────
pub mod pgn {
    // Engine (ECM)
    pub const EBC1: u32 = 61441; // Electronic Brake Controller 1
    pub const ETC1: u32 = 61442; // Electronic Transmission Ctrl 1
    pub const ETC2: u32 = 61443; // Electronic Transmission Ctrl 2
    pub const EEC1: u32 = 61444; // Electronic Engine Control 1
    pub const EEC2: u32 = 61445; // Electronic Engine Control 2
    pub const EEC3: u32 = 65247; // Electronic Engine Control 3
    pub const ET1: u32 = 65262; // Engine Temperature 1
    pub const EFL_P1: u32 = 65263; // Engine Fluid Level/Pressure 1
    pub const IC1: u32 = 65270; // Inlet/Exhaust Conditions 1
    pub const LFE: u32 = 65266; // Fuel Economy
    pub const HOURS: u32 = 65253; // Engine Hours
    pub const FUEL1: u32 = 65257; // Fuel Consumption (cumulative)
    pub const EEC5: u32 = 65249; // Engine Controller 5 (aux)
                                 // Vehicle
    pub const CCVS: u32 = 65265; // Cruise Control/Vehicle Speed
    pub const VD: u32 = 65248; // Vehicle Distance
    pub const AMB: u32 = 65269; // Ambient Conditions
    pub const FD: u32 = 65164; // Fan Drive
                               // Implements / ISOBUS
    pub const PTO: u32 = 65093; // Power Take-Off Information
    pub const HITCH: u32 = 65091; // Hitch & PTO Commands
    pub const WS_MST: u32 = 65534; // Working Set Master
    pub const WS_MBR: u32 = 65533; // Working Set Member
    pub const SC: u32 = 7168; // Selected Control Channel
    pub const GSC: u32 = 7437; // Guidance System Command
                               // Diagnostics
    pub const DM1: u32 = 65226; // Active DTCs
    pub const DM2: u32 = 65227; // Previously Active DTCs
    pub const DM3: u32 = 65228; // Clear Active DTCs
    pub const DM11: u32 = 65235; // Reset to Factory Defaults
    pub const DM15: u32 = 49408; // Memory Access Response
    pub const DM16: u32 = 49152; // Binary Data Transfer
    pub const RQST: u32 = 59904; // Request PGN
                                 // Transport Protocol
    pub const TP_CM: u32 = 60160; // TP Connection Management
    pub const TP_DT: u32 = 60416; // TP Data Transfer
    pub const ECAN: u32 = 60928; // Address Claim (NAME)
    pub const PROP_A: u32 = 61184; // Proprietary A
    pub const ADAS1: u32 = 65280; // ADAS consolidated status (proprietary B range)
    pub const FUS1: u32 = 65281; // Sensor fusion hazard/status
    pub const ENGY1: u32 = 65282; // Vehicle energy and load balance

    // ── Previously missing — now added ──────────────────────────────────────
    pub const TSC1: u32 = 0; // Torque/Speed Control 1 (PGN 0 — peer-to-peer)
    pub const CM1: u32 = 57344; // Cab Message 1 (switches, buttons)
    pub const TCFG: u32 = 65092; // Transmission Configuration
    pub const EC1: u32 = 61448; // Electronic Clutch Controller 1
    pub const HRVD: u32 = 65264; // High Resolution Vehicle Distance
    pub const SHUTDN: u32 = 65252; // Shutdown
    pub const SOFT: u32 = 65251; // Software Identification
    pub const ECFG: u32 = 57600; // Engine Configuration
    pub const PTODE: u32 = 57088; // PTO Drive Engagement
    pub const TD: u32 = 65254; // Time / Date
    pub const TIRE: u32 = 65268; // Tire condition (TPMS)
    pub const EI: u32 = 65255; // Engine Information
    pub const VEP1: u32 = 65271; // Vehicle Electrical Power 1
    pub const SERV: u32 = 65226; // Service (same PGN space — DM1 alias)
    pub const EBC2: u32 = 65215; // Electronic Brake Controller 2 (wheel speeds)
                                 // J1939-75: Generator sets
    pub const GSC1: u32 = 65008; // Generator AC Voltage/Current
                                 // Network Management
    pub const NM_ACK: u32 = 59392; // NM Acknowledgement
    pub const DM13: u32 = 49664; // Stop/Start Broadcast (NM coordination)
    pub const DM14: u32 = 49920; // Memory Access Request
    pub const DM18: u32 = 54528; // Data Security
    pub const DM19: u32 = 54784; // Calibration Information
    pub const DM20: u32 = 49152; // Monitor Performance Ratio
    pub const DM21: u32 = 49408; // Diagnostic Readiness 2
    pub const DM25: u32 = 57088; // Expanded Freeze Frame
    pub const DM26: u32 = 57344; // Diagnostic Readiness 3
    pub const DM27: u32 = 57600; // All Pending DTCs
                                 // J1939-76 Security
    pub const SEC: u32 = 57856; // Security Key Exchange
    pub const SECACK: u32 = 58112; // Security Acknowledgement
}

// ── SPN/PGN Registry ────────────────────────────────────────────────────────

#[derive(Clone, Copy)]
pub struct SpnDef {
    pub spn: u32,
    pub name: &'static str,
    pub byte_offset: usize,
    pub bit_offset: u8,
    pub bit_length: u8,
    pub factor: f64,
    pub offset: f64,
    pub unit: &'static str,
}

pub struct PgnDef {
    pub pgn: u32,
    pub name: &'static str,
    pub desc: &'static str,
    pub rate_ms: u32,
    pub spns: &'static [SpnDef],
}

static REGISTRY: &[PgnDef] = &[
    PgnDef {
        pgn: 61444,
        name: "EEC1",
        desc: "Electronic Engine Control 1",
        rate_ms: 10,
        spns: &[
            SpnDef {
                spn: 899,
                name: "Engine Torque Mode",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 4,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 512,
                name: "Driver Demand Torque",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -125.0,
                unit: "%",
            },
            SpnDef {
                spn: 513,
                name: "Actual Engine Torque",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -125.0,
                unit: "%",
            },
            SpnDef {
                spn: 190,
                name: "Engine Speed",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "rpm",
            },
            SpnDef {
                spn: 1675,
                name: "Engine Starter Mode",
                byte_offset: 6,
                bit_offset: 0,
                bit_length: 4,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
        ],
    },
    PgnDef {
        pgn: 61445,
        name: "EEC2",
        desc: "Electronic Engine Control 2",
        rate_ms: 50,
        spns: &[
            SpnDef {
                spn: 558,
                name: "Accel Pedal Pos 1",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
            SpnDef {
                spn: 91,
                name: "Throttle Position",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
            SpnDef {
                spn: 92,
                name: "Engine Percent Load",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
            SpnDef {
                spn: 974,
                name: "Remote Accel Pedal",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
        ],
    },
    PgnDef {
        pgn: 61442,
        name: "ETC1",
        desc: "Electronic Transmission Control 1",
        rate_ms: 20,
        spns: &[
            SpnDef {
                spn: 560,
                name: "Trans Driveline Engaged",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 573,
                name: "Torque Conv Lockup",
                byte_offset: 0,
                bit_offset: 2,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 191,
                name: "Output Shaft Speed",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "rpm",
            },
        ],
    },
    PgnDef {
        pgn: 61443,
        name: "ETC2",
        desc: "Electronic Transmission Control 2",
        rate_ms: 20,
        spns: &[
            SpnDef {
                spn: 524,
                name: "Selected Gear",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -125.0,
                unit: "idx",
            },
            SpnDef {
                spn: 523,
                name: "Current Gear",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -125.0,
                unit: "idx",
            },
            SpnDef {
                spn: 525,
                name: "Current Range",
                byte_offset: 7,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
        ],
    },
    PgnDef {
        pgn: 61441,
        name: "EBC1",
        desc: "Electronic Brake Controller 1",
        rate_ms: 20,
        spns: &[
            SpnDef {
                spn: 561,
                name: "ABS Active",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 562,
                name: "ABS Offroad Switch",
                byte_offset: 0,
                bit_offset: 2,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 563,
                name: "ASR Brake Control",
                byte_offset: 0,
                bit_offset: 4,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 1121,
                name: "ESP Active",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 521,
                name: "Front Axle Brake Demand",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
            SpnDef {
                spn: 522,
                name: "Rear Axle Brake Demand",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
        ],
    },
    PgnDef {
        pgn: 65215,
        name: "EBC2",
        desc: "Electronic Brake Controller 2",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 904,
                name: "Wheel Speed FL",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "km/h",
            },
            SpnDef {
                spn: 905,
                name: "Wheel Speed FR",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "km/h",
            },
            SpnDef {
                spn: 906,
                name: "Wheel Speed RL",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "km/h",
            },
            SpnDef {
                spn: 907,
                name: "Wheel Speed RR",
                byte_offset: 6,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "km/h",
            },
        ],
    },
    PgnDef {
        pgn: 65262,
        name: "ET1",
        desc: "Engine Temperature 1",
        rate_ms: 1000,
        spns: &[
            SpnDef {
                spn: 110,
                name: "Engine Coolant Temp",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -40.0,
                unit: "°C",
            },
            SpnDef {
                spn: 174,
                name: "Fuel Temperature",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -40.0,
                unit: "°C",
            },
            SpnDef {
                spn: 175,
                name: "Engine Oil Temperature",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.03125,
                offset: -273.0,
                unit: "°C",
            },
            SpnDef {
                spn: 176,
                name: "Turbo Oil Temperature",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.03125,
                offset: -273.0,
                unit: "°C",
            },
        ],
    },
    PgnDef {
        pgn: 65263,
        name: "EFL/P1",
        desc: "Engine Fluid Level/Pressure 1",
        rate_ms: 500,
        spns: &[
            SpnDef {
                spn: 94,
                name: "Fuel Delivery Pressure",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 8,
                factor: 4.0,
                offset: 0.0,
                unit: "kPa",
            },
            SpnDef {
                spn: 98,
                name: "Engine Oil Level",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
            SpnDef {
                spn: 100,
                name: "Engine Oil Pressure",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 8,
                factor: 4.0,
                offset: 0.0,
                unit: "kPa",
            },
            SpnDef {
                spn: 109,
                name: "Coolant Pressure",
                byte_offset: 6,
                bit_offset: 0,
                bit_length: 8,
                factor: 2.0,
                offset: 0.0,
                unit: "kPa",
            },
        ],
    },
    PgnDef {
        pgn: 65270,
        name: "IC1",
        desc: "Inlet/Exhaust Conditions 1",
        rate_ms: 500,
        spns: &[
            SpnDef {
                spn: 102,
                name: "Boost Pressure",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 8,
                factor: 2.0,
                offset: 0.0,
                unit: "kPa",
            },
            SpnDef {
                spn: 105,
                name: "Intake Manifold Temp",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -40.0,
                unit: "°C",
            },
            SpnDef {
                spn: 106,
                name: "Air Inlet Pressure",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 8,
                factor: 2.0,
                offset: 0.0,
                unit: "kPa",
            },
            SpnDef {
                spn: 107,
                name: "Air Filter Diff Pressure",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.5,
                offset: 0.0,
                unit: "kPa",
            },
            SpnDef {
                spn: 173,
                name: "Exhaust Gas Temperature",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.03125,
                offset: -273.0,
                unit: "°C",
            },
        ],
    },
    PgnDef {
        pgn: 65266,
        name: "LFE",
        desc: "Fuel Economy",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 183,
                name: "Fuel Rate",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.05,
                offset: 0.0,
                unit: "L/h",
            },
            SpnDef {
                spn: 184,
                name: "Instant Fuel Economy",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 16,
                factor: 1.0 / 512.0,
                offset: 0.0,
                unit: "km/L",
            },
            SpnDef {
                spn: 51,
                name: "Throttle Position",
                byte_offset: 6,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
        ],
    },
    PgnDef {
        pgn: 65253,
        name: "HOURS",
        desc: "Engine Hours/Revolutions",
        rate_ms: 1000,
        spns: &[SpnDef {
            spn: 247,
            name: "Total Engine Hours",
            byte_offset: 0,
            bit_offset: 0,
            bit_length: 32,
            factor: 0.05,
            offset: 0.0,
            unit: "h",
        }],
    },
    PgnDef {
        pgn: 65265,
        name: "CCVS",
        desc: "Cruise Control/Vehicle Speed",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 70,
                name: "Parking Brake Switch",
                byte_offset: 0,
                bit_offset: 2,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 84,
                name: "Wheel-Based Speed",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 16,
                factor: 1.0 / 256.0,
                offset: 0.0,
                unit: "km/h",
            },
            SpnDef {
                spn: 85,
                name: "Cruise Control Active",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 86,
                name: "CC Set Speed",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: 0.0,
                unit: "km/h",
            },
        ],
    },
    PgnDef {
        pgn: 65093,
        name: "PTO",
        desc: "Power Take-Off Info",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 1691,
                name: "Rear PTO State",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 5,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 900,
                name: "PTO Engagement Control",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 1693,
                name: "Rear PTO Output RPM",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "rpm",
            },
            SpnDef {
                spn: 1694,
                name: "Front PTO Output RPM",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.125,
                offset: 0.0,
                unit: "rpm",
            },
        ],
    },
    PgnDef {
        pgn: 65091,
        name: "HITCH",
        desc: "Hitch / Hydraulic Status",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 50001,
                name: "Hydraulic System Pressure",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 8,
                factor: 2.0,
                offset: 0.0,
                unit: "bar",
            },
            SpnDef {
                spn: 50002,
                name: "Hydraulic Fluid Temp",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -40.0,
                unit: "C",
            },
            SpnDef {
                spn: 50003,
                name: "Pump Flow",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 8,
                factor: 2.0,
                offset: 0.0,
                unit: "L/min",
            },
            SpnDef {
                spn: 50004,
                name: "Hydraulic Alarm Flags",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: 0.0,
                unit: "bits",
            },
            SpnDef {
                spn: 50005,
                name: "Hitch Position",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 8,
                factor: 100.0 / 255.0,
                offset: 0.0,
                unit: "%",
            },
        ],
    },
    PgnDef {
        pgn: 65269,
        name: "AMB",
        desc: "Ambient Conditions",
        rate_ms: 1000,
        spns: &[
            SpnDef {
                spn: 171,
                name: "Ambient Air Temp",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.03125,
                offset: -273.0,
                unit: "C",
            },
            SpnDef {
                spn: 108,
                name: "Barometric Pressure",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.5,
                offset: 0.0,
                unit: "kPa",
            },
            SpnDef {
                spn: 174,
                name: "Cabin / Inlet Temp",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: -40.0,
                unit: "C",
            },
        ],
    },
    PgnDef {
        pgn: 65271,
        name: "VEP1",
        desc: "Vehicle Electrical Power 1",
        rate_ms: 500,
        spns: &[
            SpnDef {
                spn: 168,
                name: "Battery Potential",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.05,
                offset: 0.0,
                unit: "V",
            },
            SpnDef {
                spn: 158,
                name: "Alternator Current",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 16,
                factor: 1.0,
                offset: -32000.0,
                unit: "A",
            },
            SpnDef {
                spn: 167,
                name: "Charging Voltage",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.05,
                offset: 0.0,
                unit: "V",
            },
        ],
    },
    PgnDef {
        pgn: 65280,
        name: "ADAS1",
        desc: "ADAS status and interventions",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 52001,
                name: "LKA Active",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 52002,
                name: "ACC Active",
                byte_offset: 0,
                bit_offset: 2,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 52003,
                name: "AEB Active",
                byte_offset: 0,
                bit_offset: 4,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 52004,
                name: "Lead Distance",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.1,
                offset: 0.0,
                unit: "m",
            },
            SpnDef {
                spn: 52005,
                name: "TTC",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.01,
                offset: 0.0,
                unit: "s",
            },
            SpnDef {
                spn: 52006,
                name: "Speed Limit",
                byte_offset: 5,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: 0.0,
                unit: "km/h",
            },
        ],
    },
    PgnDef {
        pgn: 65281,
        name: "FUS1",
        desc: "Sensor fusion hazard status",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 52101,
                name: "Fused Objects",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 8,
                factor: 1.0,
                offset: 0.0,
                unit: "count",
            },
            SpnDef {
                spn: 52102,
                name: "Critical TTC",
                byte_offset: 1,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.01,
                offset: 0.0,
                unit: "s",
            },
            SpnDef {
                spn: 52103,
                name: "Fusion Confidence",
                byte_offset: 3,
                bit_offset: 0,
                bit_length: 8,
                factor: 0.4,
                offset: 0.0,
                unit: "%",
            },
            SpnDef {
                spn: 52104,
                name: "In-Path Hazard",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 1,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
        ],
    },
    PgnDef {
        pgn: 65282,
        name: "ENGY1",
        desc: "Vehicle energy balance",
        rate_ms: 100,
        spns: &[
            SpnDef {
                spn: 52201,
                name: "Battery Voltage",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.05,
                offset: 0.0,
                unit: "V",
            },
            SpnDef {
                spn: 52202,
                name: "Alternator Current",
                byte_offset: 2,
                bit_offset: 0,
                bit_length: 16,
                factor: 1.0,
                offset: 0.0,
                unit: "A",
            },
            SpnDef {
                spn: 52203,
                name: "Total Electrical Load",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.01,
                offset: 0.0,
                unit: "kW",
            },
            SpnDef {
                spn: 52204,
                name: "Hydraulic Power",
                byte_offset: 6,
                bit_offset: 0,
                bit_length: 16,
                factor: 0.01,
                offset: 0.0,
                unit: "kW",
            },
        ],
    },
    PgnDef {
        pgn: 65226,
        name: "DM1",
        desc: "Active Diagnostic Trouble Codes",
        rate_ms: 1000,
        spns: &[
            SpnDef {
                spn: 1213,
                name: "MIL Lamp",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 1214,
                name: "Red Stop Lamp",
                byte_offset: 0,
                bit_offset: 2,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 1215,
                name: "Amber Warning Lamp",
                byte_offset: 0,
                bit_offset: 4,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
            SpnDef {
                spn: 1216,
                name: "Protect Lamp",
                byte_offset: 0,
                bit_offset: 6,
                bit_length: 2,
                factor: 1.0,
                offset: 0.0,
                unit: "",
            },
        ],
    },
    PgnDef {
        pgn: 65257,
        name: "FUEL1",
        desc: "Fuel Consumption (cumulative)",
        rate_ms: 1000,
        spns: &[
            SpnDef {
                spn: 182,
                name: "Trip Fuel",
                byte_offset: 0,
                bit_offset: 0,
                bit_length: 32,
                factor: 0.5,
                offset: 0.0,
                unit: "L",
            },
            SpnDef {
                spn: 250,
                name: "Total Fuel Used",
                byte_offset: 4,
                bit_offset: 0,
                bit_length: 32,
                factor: 0.5,
                offset: 0.0,
                unit: "L",
            },
        ],
    },
];

pub fn find_pgn(pgn: u32) -> Option<&'static PgnDef> {
    REGISTRY.iter().find(|p| p.pgn == pgn)
}

// ── J1939 Frame ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct J1939Frame {
    pub timestamp: f64,
    pub raw_id: u32,
    pub priority: u8,
    pub pgn: u32,
    pub sa: u8,
    pub da: u8, // 0xFF = broadcast
    pub data: [u8; 8],
    pub dlc: u8,
    pub decoded: Vec<SpnValue>,
}

#[derive(Debug, Clone)]
pub struct SpnValue {
    pub spn: u32,
    pub name: &'static str,
    pub physical: f64,
    pub unit: &'static str,
}

impl J1939Frame {
    pub fn from_raw(ts: f64, raw_id: u32, data: &[u8]) -> Self {
        let priority = ((raw_id >> 26) & 0x7) as u8;
        let dp = (raw_id >> 24) & 0x1;
        let pf = ((raw_id >> 16) & 0xFF) as u8;
        let ps = ((raw_id >> 8) & 0xFF) as u8;
        let sa = (raw_id & 0xFF) as u8;

        let (pgn, da) = if pf < 0xF0 {
            (((dp << 17) | ((pf as u32) << 8)) as u32, ps)
        } else {
            (((dp << 17) | ((pf as u32) << 8) | (ps as u32)) as u32, 0xFF)
        };

        let mut arr = [0xFFu8; 8];
        let len = data.len().min(8);
        arr[..len].copy_from_slice(&data[..len]);

        let mut f = J1939Frame {
            timestamp: ts,
            raw_id,
            priority,
            pgn,
            sa,
            da,
            data: arr,
            dlc: len as u8,
            decoded: Vec::new(),
        };
        f.decode();
        f
    }

    pub fn build_id(priority: u8, pgn: u32, sa: u8, da: u8) -> u32 {
        let dp = (pgn >> 17) & 0x1;
        let pf = (pgn >> 8) & 0xFF;
        let ps = if pf < 0xF0 { da as u32 } else { pgn & 0xFF };
        ((priority as u32 & 7) << 26) | (dp << 24) | (pf << 16) | (ps << 8) | (sa as u32)
    }

    fn decode(&mut self) {
        let def = match find_pgn(self.pgn) {
            Some(d) => d,
            None => return,
        };
        for spn in def.spns {
            let end_byte =
                spn.byte_offset + ((spn.bit_offset as usize + spn.bit_length as usize + 7) / 8);
            if end_byte > self.dlc as usize {
                continue;
            }
            let raw = bits(&self.data, spn.byte_offset, spn.bit_offset, spn.bit_length);
            if raw == (1u64 << spn.bit_length) - 1 {
                continue;
            } // all-ones = not available
            self.decoded.push(SpnValue {
                spn: spn.spn,
                name: spn.name,
                physical: raw as f64 * spn.factor + spn.offset,
                unit: spn.unit,
            });
        }
    }

    pub fn pgn_name(&self) -> &'static str {
        find_pgn(self.pgn).map(|p| p.name).unwrap_or("???")
    }

    pub fn sa_name(&self) -> &'static str {
        match self.sa {
            0x00 => "ECM1 ",
            0x01 => "ECM2 ",
            0x02 => "TURBO",
            0x03 => "TRANS",
            0x0B => "ABS  ",
            0x0D => "TC   ",
            0x1C => "DASH ",
            0x25 => "IMPL ",
            0x26 => "VT   ",
            0x27 => "CAB  ",
            0x28 => "ARST ",
            0x1E => "HITCH",
            0xFE => "NULL ",
            0xFF => "BCAST",
            _ => "UNK  ",
        }
    }

    pub fn data_hex(&self) -> String {
        self.data[..self.dlc as usize]
            .iter()
            .map(|b| format!("{:02X}", b))
            .collect::<Vec<_>>()
            .join(" ")
    }
}

fn bits(data: &[u8], byte_off: usize, bit_off: u8, len: u8) -> u64 {
    let mut v = 0u64;
    for i in 0..len as usize {
        let abs = byte_off * 8 + bit_off as usize + i;
        let b = abs / 8;
        let k = abs % 8;
        if b < data.len() && (data[b] >> k) & 1 == 1 {
            v |= 1 << i;
        }
    }
    v
}

// ── J1939 Frame Builders ────────────────────────────────────────────────────

pub struct Builder;
impl Builder {
    pub fn eec1(ts: f64, rpm: f64, actual_pct: f64, demand_pct: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = 0xF0;
        d[1] = (demand_pct + 125.0).clamp(0.0, 250.0) as u8;
        d[2] = (actual_pct + 125.0).clamp(0.0, 250.0) as u8;
        let s = (rpm / 0.125) as u16;
        d[3] = (s & 0xFF) as u8;
        d[4] = (s >> 8) as u8;
        d[5] = sa;
        d[6] = 0xF0;
        J1939Frame::from_raw(ts, J1939Frame::build_id(3, pgn::EEC1, sa, 0xFF), &d)
    }
    pub fn eec2(ts: f64, throttle: f64, load: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[1] = (throttle / 0.4).clamp(0.0, 250.0) as u8;
        d[2] = (load / 0.4).clamp(0.0, 250.0) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(3, pgn::EEC2, sa, 0xFF), &d)
    }
    pub fn et1(ts: f64, coolant: f64, fuel_t: f64, oil_t: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = (coolant + 40.0).clamp(0.0, 250.0) as u8;
        d[1] = (fuel_t + 40.0).clamp(0.0, 250.0) as u8;
        let o = ((oil_t + 273.0) / 0.03125) as u16;
        d[2] = (o & 0xFF) as u8;
        d[3] = (o >> 8) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::ET1, sa, 0xFF), &d)
    }
    pub fn efl_p1(ts: f64, fuel_kpa: f64, oil_lvl: f64, oil_kpa: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = (fuel_kpa / 4.0).clamp(0.0, 250.0) as u8;
        d[3] = (oil_lvl / 0.4).clamp(0.0, 250.0) as u8;
        d[4] = (oil_kpa / 4.0).clamp(0.0, 250.0) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::EFL_P1, sa, 0xFF), &d)
    }
    pub fn ic1(ts: f64, boost_kpa: f64, intake_t: f64, exh_t: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = (boost_kpa / 2.0).clamp(0.0, 250.0) as u8;
        d[1] = (intake_t + 40.0).clamp(0.0, 250.0) as u8;
        let e = ((exh_t + 273.0) / 0.03125) as u16;
        d[4] = (e & 0xFF) as u8;
        d[5] = (e >> 8) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::IC1, sa, 0xFF), &d)
    }
    pub fn lfe(ts: f64, fuel_lph: f64, throttle: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        let f = (fuel_lph / 0.05) as u16;
        d[0] = (f & 0xFF) as u8;
        d[1] = (f >> 8) as u8;
        d[6] = (throttle / 0.4).clamp(0.0, 250.0) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::LFE, sa, 0xFF), &d)
    }
    pub fn pto(ts: f64, rpm: f64, enabled: bool, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = if enabled { 0x01 } else { 0x00 };
        let r = (rpm / 0.125) as u16;
        d[2] = (r & 0xFF) as u8;
        d[3] = (r >> 8) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::PTO, sa, 0xFF), &d)
    }
    pub fn ccvs(ts: f64, speed: f64, cc_active: bool, cc_speed: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = 0b11111100;
        let s = (speed * 256.0) as u16;
        d[1] = (s & 0xFF) as u8;
        d[2] = (s >> 8) as u8;
        d[3] = if cc_active { 0x01 } else { 0x00 };
        d[4] = cc_speed.clamp(0.0, 250.0) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::CCVS, sa, 0xFF), &d)
    }
    pub fn dm1(
        ts: f64,
        amber: bool,
        red: bool,
        mil: bool,
        dtc_spn: u32,
        fmi: u8,
        sa: u8,
    ) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = (if mil { 0x40 } else { 0 })
            | (if red { 0x10 } else { 0 })
            | (if amber { 0x04 } else { 0 });
        d[1] = 0xFF;
        d[2] = (dtc_spn & 0xFF) as u8;
        d[3] = ((dtc_spn >> 8) & 0xFF) as u8;
        d[4] = (((dtc_spn >> 16) & 0x7) as u8) | ((fmi & 0x1F) << 3);
        d[5] = 1; // occurrence count
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::DM1, sa, 0xFF), &d)
    }
    pub fn hours(ts: f64, total_h: f64, sa: u8) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        let h = (total_h / 0.05) as u32;
        d[0] = (h & 0xFF) as u8;
        d[1] = ((h >> 8) & 0xFF) as u8;
        d[2] = ((h >> 16) & 0xFF) as u8;
        d[3] = ((h >> 24) & 0xFF) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::HOURS, sa, 0xFF), &d)
    }

    pub fn adas1(
        ts: f64,
        lka_on: bool,
        acc_on: bool,
        aeb_on: bool,
        lead_distance_m: f64,
        ttc_s: f64,
        speed_limit_kmh: f64,
        sa: u8,
    ) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = (if lka_on { 0x01 } else { 0x00 })
            | (if acc_on { 0x01 } else { 0x00 }) << 2
            | (if aeb_on { 0x01 } else { 0x00 }) << 4;
        let lead = (lead_distance_m.clamp(0.0, 6553.0) / 0.1) as u16;
        d[1] = (lead & 0xFF) as u8;
        d[2] = (lead >> 8) as u8;
        let ttc = (ttc_s.clamp(0.0, 655.35) / 0.01) as u16;
        d[3] = (ttc & 0xFF) as u8;
        d[4] = (ttc >> 8) as u8;
        d[5] = speed_limit_kmh.clamp(0.0, 250.0) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::ADAS1, sa, 0xFF), &d)
    }

    pub fn fus1(
        ts: f64,
        fused_objects: usize,
        critical_ttc_s: f64,
        confidence_01: f64,
        in_path_hazard: bool,
        sa: u8,
    ) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        d[0] = fused_objects.min(255) as u8;
        let ttc = (critical_ttc_s.clamp(0.0, 655.35) / 0.01) as u16;
        d[1] = (ttc & 0xFF) as u8;
        d[2] = (ttc >> 8) as u8;
        d[3] = (confidence_01.clamp(0.0, 1.0) * 250.0) as u8;
        d[4] = if in_path_hazard { 0x01 } else { 0x00 };
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::FUS1, sa, 0xFF), &d)
    }

    pub fn engy1(
        ts: f64,
        battery_v: f64,
        alternator_a: f64,
        electrical_kw: f64,
        hydraulic_kw: f64,
        sa: u8,
    ) -> J1939Frame {
        let mut d = [0xFFu8; 8];
        let bv = (battery_v.clamp(0.0, 3276.75) / 0.05) as u16;
        d[0] = (bv & 0xFF) as u8;
        d[1] = (bv >> 8) as u8;
        let ia = alternator_a.clamp(0.0, 65535.0) as u16;
        d[2] = (ia & 0xFF) as u8;
        d[3] = (ia >> 8) as u8;
        let ek = (electrical_kw.clamp(0.0, 655.35) / 0.01) as u16;
        d[4] = (ek & 0xFF) as u8;
        d[5] = (ek >> 8) as u8;
        let hk = (hydraulic_kw.clamp(0.0, 655.35) / 0.01) as u16;
        d[6] = (hk & 0xFF) as u8;
        d[7] = (hk >> 8) as u8;
        J1939Frame::from_raw(ts, J1939Frame::build_id(6, pgn::ENGY1, sa, 0xFF), &d)
    }
}

// ── DTC Definitions ─────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Dtc {
    pub spn: u32,
    pub fmi: u8,
    pub count: u8,
    pub active: bool,
    pub desc: &'static str,
    pub severity: DtcSeverity,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DtcSeverity {
    Protect,
    Amber,
    Red,
    Mil,
}

impl std::fmt::Display for DtcSeverity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DtcSeverity::Protect => write!(f, "PROT"),
            DtcSeverity::Amber => write!(f, "AMBR"),
            DtcSeverity::Red => write!(f, "RED!"),
            DtcSeverity::Mil => write!(f, "MIL "),
        }
    }
}

pub fn fmi_name(fmi: u8) -> &'static str {
    match fmi {
        0 => "Above Normal — Most Severe",
        1 => "Below Normal — Most Severe",
        2 => "Erratic / Intermittent",
        3 => "Voltage High / Short to High",
        4 => "Voltage Low / Short to Low",
        5 => "Current Low / Open Circuit",
        6 => "Current High / Grounded",
        7 => "Mechanical System Not Responding",
        8 => "Abnormal Frequency / Pulse Width",
        9 => "Abnormal Update Rate",
        10 => "Abnormal Rate of Change",
        11 => "Root Cause Not Known",
        12 => "Bad Device / Component",
        13 => "Out of Calibration",
        14 => "Special Instructions",
        15 => "Above Normal — Least Severe",
        16 => "Above Normal — Moderately Severe",
        17 => "Below Normal — Least Severe",
        18 => "Below Normal — Moderately Severe",
        19 => "Received Network Data in Error",
        31 => "Condition Exists",
        _ => "Reserved",
    }
}

// ── J1939 Bus Logger ─────────────────────────────────────────────────────────

pub struct J1939Bus {
    pub frames: VecDeque<J1939Frame>,
    pub total_frames: u64,
    pub fps: u32,
    pub bus_load_pct: f64,
    frame_count: u32,
    frame_timer: f64,
    pub filter_pgn: Option<u32>,
    pub filter_sa: Option<u8>,
}

impl J1939Bus {
    pub fn new() -> Self {
        Self {
            frames: VecDeque::new(),
            total_frames: 0,
            fps: 0,
            bus_load_pct: 0.0,
            frame_count: 0,
            frame_timer: 0.0,
            filter_pgn: None,
            filter_sa: None,
        }
    }

    pub fn push(&mut self, frame: J1939Frame) {
        self.frames.push_front(frame);
        if self.frames.len() > 200 {
            self.frames.pop_back();
        }
        self.total_frames += 1;
        self.frame_count += 1;
    }

    pub fn tick(&mut self, dt: f64) {
        self.frame_timer += dt;
        if self.frame_timer >= 1.0 {
            self.fps = self.frame_count;
            // 500 kbps, min frame ~55 bits → ~9090 frames/s max
            self.bus_load_pct = (self.frame_count as f64 / 9090.0 * 100.0).min(100.0);
            self.frame_count = 0;
            self.frame_timer = 0.0;
        }
    }

    pub fn visible_frames(&self) -> impl Iterator<Item = &J1939Frame> {
        self.frames.iter().filter(|f| {
            self.filter_pgn.map_or(true, |p| f.pgn == p)
                && self.filter_sa.map_or(true, |s| f.sa == s)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pdu1_id_roundtrip_preserves_destination() {
        let id = J1939Frame::build_id(3, pgn::TSC1, addr::ECM_1, addr::TRANSMISSION);
        let frame = J1939Frame::from_raw(0.0, id, &[0xFF; 8]);
        assert_eq!(frame.pgn, pgn::TSC1);
        assert_eq!(frame.da, addr::TRANSMISSION);
        assert_eq!(frame.sa, addr::ECM_1);
    }

    #[test]
    fn ebc2_decodes_all_wheel_speeds() {
        let mut data = [0xFFu8; 8];
        let fl = (48.0 / 0.125) as u16;
        let fr = (50.0 / 0.125) as u16;
        let rl = (47.0 / 0.125) as u16;
        let rr = (49.0 / 0.125) as u16;
        data[0] = (fl & 0xFF) as u8;
        data[1] = (fl >> 8) as u8;
        data[2] = (fr & 0xFF) as u8;
        data[3] = (fr >> 8) as u8;
        data[4] = (rl & 0xFF) as u8;
        data[5] = (rl >> 8) as u8;
        data[6] = (rr & 0xFF) as u8;
        data[7] = (rr >> 8) as u8;
        let id = J1939Frame::build_id(6, pgn::EBC2, addr::BRAKES, 0xFF);
        let frame = J1939Frame::from_raw(0.0, id, &data);
        assert_eq!(frame.pgn_name(), "EBC2");
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 904 && (d.physical - 48.0).abs() < 0.2));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 905 && (d.physical - 50.0).abs() < 0.2));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 906 && (d.physical - 47.0).abs() < 0.2));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 907 && (d.physical - 49.0).abs() < 0.2));
    }

    #[test]
    fn etc2_gear_decode_matches_encoded_value() {
        let mut data = [0xFFu8; 8];
        data[3] = 132; // selected gear = 7 after offset -125
        data[4] = 132; // current gear = 7
        data[7] = 3; // range C
        let id = J1939Frame::build_id(3, pgn::ETC2, addr::TRANSMISSION, 0xFF);
        let frame = J1939Frame::from_raw(0.0, id, &data);
        assert_eq!(frame.pgn_name(), "ETC2");
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 524 && (d.physical - 7.0).abs() < 0.1));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 523 && (d.physical - 7.0).abs() < 0.1));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 525 && (d.physical - 3.0).abs() < 0.1));
    }

    #[test]
    fn adas_builder_roundtrip_exposes_core_spns() {
        let frame = Builder::adas1(0.0, true, true, false, 23.4, 1.85, 80.0, addr::HEADWAY);
        assert_eq!(frame.pgn, pgn::ADAS1);
        assert_eq!(frame.pgn_name(), "ADAS1");
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52001 && d.physical >= 1.0));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52002 && d.physical >= 1.0));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52004 && (d.physical - 23.4).abs() < 0.2));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52005 && (d.physical - 1.85).abs() < 0.05));
    }

    #[test]
    fn energy_builder_roundtrip_exposes_power_values() {
        let frame = Builder::engy1(0.0, 27.4, 84.0, 2.30, 19.75, addr::HEADWAY);
        assert_eq!(frame.pgn, pgn::ENGY1);
        assert_eq!(frame.pgn_name(), "ENGY1");
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52201 && (d.physical - 27.4).abs() < 0.2));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52202 && (d.physical - 84.0).abs() < 1.0));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52203 && (d.physical - 2.30).abs() < 0.05));
        assert!(frame
            .decoded
            .iter()
            .any(|d| d.spn == 52204 && (d.physical - 19.75).abs() < 0.05));
    }
}
