use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use super::hw::HwError;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FirmwareArtifactReport {
    pub path: String,
    pub bytes: usize,
    pub crc32: u32,
    pub sha256: String,
    pub valid: bool,
}

pub fn validate_firmware_artifact<P: AsRef<Path>>(
    path: P,
    max_bytes: usize,
) -> Result<FirmwareArtifactReport, HwError> {
    let p = path.as_ref();
    let data = fs::read(p)
        .map_err(|e| HwError::Unknown(format!("artifact read failed ({}): {e}", p.display())))?;

    if data.is_empty() {
        return Err(HwError::Unknown("artifact payload is empty".to_string()));
    }
    if data.len() > max_bytes {
        return Err(HwError::Unknown(format!(
            "artifact payload too large: {} bytes (limit {})",
            data.len(),
            max_bytes
        )));
    }

    let crc32 = crc32fast::hash(&data);
    let mut hasher = Sha256::new();
    hasher.update(&data);
    let sha = hasher.finalize();
    let mut sha_hex = String::with_capacity(64);
    for b in sha {
        sha_hex.push_str(&format!("{b:02x}"));
    }

    Ok(FirmwareArtifactReport {
        path: p.display().to_string(),
        bytes: data.len(),
        crc32,
        sha256: sha_hex,
        valid: true,
    })
}
