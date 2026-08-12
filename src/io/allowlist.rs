use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use super::hw::{Frame, HwError};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RuleType {
    Can,
    Serial,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AllowlistRule {
    #[serde(rename = "type")]
    pub rule_type: RuleType,
    pub id: Option<u32>,
    pub mask: Option<String>,
    pub allowed_bytes: Option<Vec<[usize; 2]>>,
    pub max_rate_per_sec: Option<u32>,
    pub pattern_hex: Option<String>,
    pub description: Option<String>,
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Allowlist {
    pub rules: Vec<AllowlistRule>,
}

impl Allowlist {
    pub fn from_path(path: &Path) -> Result<Self, String> {
        let raw = fs::read_to_string(path).map_err(|e| format!("read allowlist failed: {e}"))?;
        let rules: Vec<AllowlistRule> =
            serde_json::from_str(&raw).map_err(|e| format!("parse allowlist failed: {e}"))?;
        let allow = Self { rules };
        allow.validate().map_err(|e| e.to_string())?;
        Ok(allow)
    }

    pub fn is_allowed(&self, frame: &Frame) -> bool {
        self.rules.iter().any(|r| rule_matches_frame(r, frame))
    }

    pub fn per_rule_rate_limit(&self, frame: &Frame) -> Option<u32> {
        self.rules
            .iter()
            .find(|r| rule_matches_frame(r, frame))
            .and_then(|r| r.max_rate_per_sec)
    }

    fn validate(&self) -> Result<(), HwError> {
        for (idx, rule) in self.rules.iter().enumerate() {
            match rule.rule_type {
                RuleType::Can => {
                    if rule.id.is_none() {
                        return Err(HwError::Unknown(format!(
                            "allowlist rule #{idx} missing CAN id"
                        )));
                    }
                    if let Some(mask) = &rule.mask {
                        if parse_u32_hex_or_dec(mask).is_none() {
                            return Err(HwError::Unknown(format!(
                                "allowlist rule #{idx} has invalid mask"
                            )));
                        }
                    }
                    if let Some(windows) = &rule.allowed_bytes {
                        for win in windows {
                            if win[1] == 0 || win[0] >= 8 || win[0].saturating_add(win[1]) > 8 {
                                return Err(HwError::Unknown(format!(
                                    "allowlist rule #{idx} has invalid allowed_bytes window"
                                )));
                            }
                        }
                    }
                }
                RuleType::Serial => {
                    if let Some(p) = &rule.pattern_hex {
                        let compact: String = p.chars().filter(|c| !c.is_whitespace()).collect();
                        if !compact.len().is_multiple_of(2) {
                            return Err(HwError::Unknown(format!(
                                "allowlist rule #{idx} has invalid serial pattern length"
                            )));
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

fn rule_matches_frame(rule: &AllowlistRule, frame: &Frame) -> bool {
    match (rule.rule_type.clone(), frame) {
        (RuleType::Can, Frame::Can(cf)) => {
            let rule_id = match rule.id {
                Some(v) => v,
                None => return false,
            };
            let mask = rule
                .mask
                .as_deref()
                .and_then(parse_u32_hex_or_dec)
                .unwrap_or(0x1FFF_FFFF);

            if (cf.id & mask) != (rule_id & mask) {
                return false;
            }

            // Restrict all payload bytes outside windows when configured.
            if let Some(allowed) = &rule.allowed_bytes {
                for i in 0..cf.len.min(8) {
                    if !allowed
                        .iter()
                        .any(|w| i >= w[0] && i < w[0].saturating_add(w[1]))
                        && cf.data[i] != 0
                    {
                        return false;
                    }
                }
            }

            true
        }
        (RuleType::Serial, Frame::Serial(sf)) => {
            let Some(pattern) = &rule.pattern_hex else {
                return false;
            };
            match_hex_pattern(pattern, &sf.bytes)
        }
        _ => false,
    }
}

fn parse_u32_hex_or_dec(s: &str) -> Option<u32> {
    let t = s.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).ok()
    } else {
        t.parse::<u32>().ok()
    }
}

pub fn parse_mask(mask: &str) -> Result<u32, HwError> {
    parse_u32_hex_or_dec(mask).ok_or_else(|| HwError::Unknown("invalid allowlist mask".into()))
}

fn match_hex_pattern(pattern: &str, bytes: &[u8]) -> bool {
    let compact: String = pattern.chars().filter(|c| !c.is_whitespace()).collect();
    if !compact.len().is_multiple_of(2) {
        return false;
    }

    let mut idx = 0usize;
    for chunk in compact.as_bytes().chunks(2) {
        if idx >= bytes.len() {
            return false;
        }
        let pair = std::str::from_utf8(chunk).unwrap_or_default();
        if pair != ".." {
            let Ok(expected) = u8::from_str_radix(pair, 16) else {
                return false;
            };
            if bytes[idx] != expected {
                return false;
            }
        }
        idx += 1;
    }

    idx == bytes.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::hw::{CanFrame, Frame, SerialFrame};

    #[test]
    fn can_rule_matches_mask() {
        let allow = Allowlist {
            rules: vec![AllowlistRule {
                rule_type: RuleType::Can,
                id: Some(0x18FF50E5),
                mask: Some("0x1FFFFFFF".to_string()),
                allowed_bytes: Some(vec![[2, 2]]),
                max_rate_per_sec: None,
                pattern_hex: None,
                description: None,
            }],
        };

        let mut data = [0u8; 8];
        data[2] = 0xAB;
        let f = Frame::Can(CanFrame {
            id: 0x18FF50E5,
            dlc: 8,
            data,
            len: 8,
            timestamp_ms: None,
        });

        assert!(allow.is_allowed(&f));
    }

    #[test]
    fn serial_rule_matches_pattern() {
        let allow = Allowlist {
            rules: vec![AllowlistRule {
                rule_type: RuleType::Serial,
                id: None,
                mask: None,
                allowed_bytes: None,
                max_rate_per_sec: None,
                pattern_hex: Some("AA55..".to_string()),
                description: None,
            }],
        };

        let f = Frame::Serial(SerialFrame {
            bytes: vec![0xAA, 0x55, 0x01],
            protocol_hint: None,
            timestamp_ms: None,
        });

        assert!(allow.is_allowed(&f));
    }
}
