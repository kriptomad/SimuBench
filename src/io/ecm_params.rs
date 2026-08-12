use serde::{Deserialize, Serialize};

use super::hw::CanFrame;

// J1939 standard PGNs often available on heavy-duty ECM networks, including Caterpillar platforms.
pub const PGN_EEC1: u32 = 61444;
pub const PGN_ET1: u32 = 65262;
pub const PGN_EFL_P1: u32 = 65263;

#[derive(Debug, Clone, Serialize, Deserialize, Default, PartialEq)]
pub struct EcmSnapshot {
    pub engine_speed_rpm: Option<f64>,
    pub accel_pedal_pct: Option<f64>,
    pub coolant_temp_c: Option<f64>,
    pub oil_pressure_kpa: Option<f64>,
    pub fuel_temp_c: Option<f64>,
    pub last_seen_pgn: Option<u32>,
    pub source_address: Option<u8>,
    pub timestamp_ms: Option<u64>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct DecodedParam {
    pub key: &'static str,
    pub value: f64,
    pub unit: &'static str,
}

pub fn decode_can_frame(frame: &CanFrame) -> Vec<DecodedParam> {
    let pgn = j1939_pgn(frame.id);
    let d = &frame.data;

    match pgn {
        PGN_EEC1 => {
            // SPN 190 Engine Speed: bytes 4-5, little-endian, 0.125 rpm/bit.
            // SPN 91 Accelerator Pedal Position 1: byte 2, 0.4 %/bit.
            let eng_raw = u16::from_le_bytes([d[3], d[4]]);
            let app1_raw = d[1];
            let mut out = Vec::new();
            if eng_raw != 0xFFFF {
                out.push(DecodedParam {
                    key: "engine_speed_rpm",
                    value: f64::from(eng_raw) * 0.125,
                    unit: "rpm",
                });
            }
            if app1_raw != 0xFF {
                out.push(DecodedParam {
                    key: "accel_pedal_pct",
                    value: f64::from(app1_raw) * 0.4,
                    unit: "%",
                });
            }
            out
        }
        PGN_ET1 => {
            // SPN 110 Coolant Temp: byte 1, 1 C/bit, offset -40 C.
            // SPN 174 Fuel Temp 1: byte 2, 1 C/bit, offset -40 C.
            let mut out = Vec::new();
            if d[0] != 0xFF {
                out.push(DecodedParam {
                    key: "coolant_temp_c",
                    value: f64::from(d[0]) - 40.0,
                    unit: "C",
                });
            }
            if d[1] != 0xFF {
                out.push(DecodedParam {
                    key: "fuel_temp_c",
                    value: f64::from(d[1]) - 40.0,
                    unit: "C",
                });
            }
            out
        }
        PGN_EFL_P1 => {
            // SPN 100 Engine Oil Pressure: byte 4, 4 kPa/bit.
            let oil_raw = d[3];
            if oil_raw != 0xFF {
                vec![DecodedParam {
                    key: "oil_pressure_kpa",
                    value: f64::from(oil_raw) * 4.0,
                    unit: "kPa",
                }]
            } else {
                Vec::new()
            }
        }
        _ => Vec::new(),
    }
}

pub fn merge_snapshot(mut snap: EcmSnapshot, frame: &CanFrame) -> EcmSnapshot {
    let pgn = j1939_pgn(frame.id);
    for p in decode_can_frame(frame) {
        match p.key {
            "engine_speed_rpm" => snap.engine_speed_rpm = Some(p.value),
            "accel_pedal_pct" => snap.accel_pedal_pct = Some(p.value),
            "coolant_temp_c" => snap.coolant_temp_c = Some(p.value),
            "fuel_temp_c" => snap.fuel_temp_c = Some(p.value),
            "oil_pressure_kpa" => snap.oil_pressure_kpa = Some(p.value),
            _ => {}
        }
    }
    snap.last_seen_pgn = Some(pgn);
    snap.source_address = Some((frame.id & 0xFF) as u8);
    snap.timestamp_ms = frame.timestamp_ms;
    snap
}

pub fn j1939_pgn(can_id: u32) -> u32 {
    // 29-bit CAN ID for J1939: priority(3), reserved(1), dp(1), pf(8), ps(8), sa(8).
    let pf = ((can_id >> 16) & 0xFF) as u8;
    let ps = ((can_id >> 8) & 0xFF) as u8;
    let dp = ((can_id >> 24) & 0x01) as u32;

    if pf < 240 {
        (dp << 16) | ((pf as u32) << 8)
    } else {
        (dp << 16) | ((pf as u32) << 8) | ps as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decode_eec1_engine_speed() {
        let frame = CanFrame {
            id: 0x0CF00400, // PGN 61444
            dlc: 8,
            data: [0x00, 50, 0x00, 0x40, 0x1F, 0, 0, 0],
            len: 8,
            timestamp_ms: Some(1),
        };

        let vals = decode_can_frame(&frame);
        assert!(vals.iter().any(|v| v.key == "engine_speed_rpm"));
        assert!(vals.iter().any(|v| v.key == "accel_pedal_pct"));
    }

    #[test]
    fn decode_et1_coolant() {
        let frame = CanFrame {
            id: 0x18FEEE00, // PGN 65262
            dlc: 8,
            data: [90, 100, 0, 0, 0, 0, 0, 0],
            len: 8,
            timestamp_ms: Some(2),
        };

        let vals = decode_can_frame(&frame);
        assert!(vals.iter().any(|v| v.key == "coolant_temp_c"));
        assert!(vals.iter().any(|v| v.key == "fuel_temp_c"));
    }
}
