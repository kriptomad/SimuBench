use std::fs::{self, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use super::hw::{CanFrame, Frame, HwError, SerialFrame};

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct JsonlRecord {
    pub ts: u64,
    pub transport: String,
    pub dir: String,
    pub id: Option<u32>,
    pub dlc: Option<u8>,
    pub data: Option<String>,
    pub raw_hex: Option<String>,
    pub allowed: Option<bool>,
    pub dry_run: Option<bool>,
}

pub fn append_record(path: &Path, rec: &JsonlRecord) -> Result<(), HwError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)
            .map_err(|e| HwError::Unknown(format!("create log dir failed: {e}")))?;
    }

    let mut f = OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|e| HwError::Unknown(format!("open jsonl log failed: {e}")))?;

    let line = serde_json::to_string(rec)
        .map_err(|e| HwError::Unknown(format!("json encode failed: {e}")))?;
    writeln!(f, "{line}").map_err(|e| HwError::Unknown(format!("write jsonl failed: {e}")))
}

pub fn read_records(path: &Path) -> Result<Vec<JsonlRecord>, HwError> {
    let f = OpenOptions::new()
        .read(true)
        .open(path)
        .map_err(|e| HwError::Unknown(format!("open replay file failed: {e}")))?;
    let r = BufReader::new(f);
    let mut out = Vec::new();
    for line in r.lines() {
        let line = line.map_err(|e| HwError::Unknown(format!("read line failed: {e}")))?;
        let rec = serde_json::from_str::<JsonlRecord>(&line)
            .map_err(|e| HwError::Unknown(format!("parse line failed: {e}")))?;
        out.push(rec);
    }
    Ok(out)
}

pub fn frame_to_record(
    frame: &Frame,
    dir: &str,
    allowed: Option<bool>,
    dry_run: Option<bool>,
) -> JsonlRecord {
    match frame {
        Frame::Can(cf) => JsonlRecord {
            ts: cf.timestamp_ms.unwrap_or_else(now_ms),
            transport: "can".to_string(),
            dir: dir.to_string(),
            id: Some(cf.id),
            dlc: Some(cf.dlc),
            data: Some(hex_encode(&cf.data[..cf.len.min(8)])),
            raw_hex: None,
            allowed,
            dry_run,
        },
        Frame::Serial(sf) => JsonlRecord {
            ts: sf.timestamp_ms.unwrap_or_else(now_ms),
            transport: "serial".to_string(),
            dir: dir.to_string(),
            id: None,
            dlc: Some((sf.bytes.len().min(255)) as u8),
            data: Some(hex_encode(&sf.bytes)),
            raw_hex: None,
            allowed,
            dry_run,
        },
    }
}

pub fn record_to_frame(rec: &JsonlRecord) -> Option<Frame> {
    if rec.dir != "rx" {
        return None;
    }

    if rec.transport == "can" {
        let id = rec.id?;
        let data_hex = rec.data.as_deref()?;
        let bytes = hex_decode(data_hex)?;
        let mut data = [0u8; 8];
        let len = bytes.len().min(8);
        data[..len].copy_from_slice(&bytes[..len]);
        return Some(Frame::Can(CanFrame {
            id,
            dlc: len as u8,
            data,
            len,
            timestamp_ms: Some(rec.ts),
        }));
    }

    if rec.transport == "serial" {
        let data_hex = rec.data.as_deref()?;
        let bytes = hex_decode(data_hex)?;
        return Some(Frame::Serial(SerialFrame {
            bytes,
            protocol_hint: None,
            timestamp_ms: Some(rec.ts),
        }));
    }

    None
}

fn now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn hex_encode(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        out.push(nibble((*b >> 4) & 0x0F));
        out.push(nibble(*b & 0x0F));
    }
    out
}

fn hex_decode(hex: &str) -> Option<Vec<u8>> {
    if !hex.len().is_multiple_of(2) {
        return None;
    }
    let mut out = Vec::with_capacity(hex.len() / 2);
    let mut chars = hex.chars();
    while let (Some(h), Some(l)) = (chars.next(), chars.next()) {
        let hi = h.to_digit(16)? as u8;
        let lo = l.to_digit(16)? as u8;
        out.push((hi << 4) | lo);
    }
    Some(out)
}

fn nibble(v: u8) -> char {
    match v {
        0..=9 => (b'0' + v) as char,
        _ => (b'a' + (v - 10)) as char,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_record_roundtrip() {
        let f = Frame::Can(CanFrame {
            id: 0x123,
            dlc: 8,
            data: [1, 2, 3, 4, 5, 6, 7, 8],
            len: 8,
            timestamp_ms: Some(12345),
        });
        let rec = frame_to_record(&f, "rx", None, None);
        let got = record_to_frame(&rec).expect("frame from record");
        assert_eq!(rec.ts, 12345);
        match got {
            Frame::Can(cf) => {
                assert_eq!(cf.id, 0x123);
                assert_eq!(cf.dlc, 8);
                assert_eq!(cf.len, 8);
                assert_eq!(cf.data, [1, 2, 3, 4, 5, 6, 7, 8]);
                assert_eq!(cf.timestamp_ms, Some(12345));
            }
            _ => panic!("expected can frame"),
        }
    }
}
