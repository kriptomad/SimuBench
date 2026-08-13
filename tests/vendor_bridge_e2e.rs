#![cfg(all(target_os = "windows", feature = "vendor-windows"))]

use std::fs;
use std::path::{Path, PathBuf};

use auto_breaking::io::hw::{probe_live_adapter, HwConfig};
use auto_breaking::io::production_program::{run_all_phases, ProgramOptions};

fn bin_path(bin_name: &str) -> PathBuf {
    let env_key = format!("CARGO_BIN_EXE_{bin_name}");
    if let Ok(p) = std::env::var(&env_key) {
        return PathBuf::from(p);
    }

    let mut fallback = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    fallback.push("target");
    fallback.push("debug");
    fallback.push(format!("{bin_name}.exe"));
    fallback
}

fn unique_temp_dir(prefix: &str) -> PathBuf {
    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis())
        .unwrap_or(0);
    let p = std::env::temp_dir().join(format!("{prefix}_{ts}"));
    fs::create_dir_all(&p).expect("create temp dir");
    p
}

#[test]
fn cat_comm_bridge_probe_and_live_phases_flash_e2e() {
    let template_dir = unique_temp_dir("autobreaking_cat_template");
    let bridge_src = bin_path("cat_comm_bridge");
    assert!(bridge_src.exists(), "cat_comm_bridge binary not found at {}", bridge_src.display());

    let bridge_dst = template_dir.join("cat_comm_bridge.exe");
    fs::copy(&bridge_src, &bridge_dst).expect("copy bridge exe");

    let allowlist_path = template_dir.join("allowlist.json");
    fs::write(&allowlist_path, "[]").expect("write allowlist");

    let firmware_path = template_dir.join("firmware.bin");
    let firmware_payload: Vec<u8> = (0..512).map(|i| (i % 251) as u8).collect();
    fs::write(&firmware_path, &firmware_payload).expect("write firmware");

    let args = vec![
        "simulator_cli".to_string(),
        "--hw-mode=live".to_string(),
        "--vendor-name=cat_comm".to_string(),
        format!("--vendor-template-dir={}", template_dir.display()),
        "--enable-write".to_string(),
        "--noninteractive-approved".to_string(),
        "--dry-run=false".to_string(),
        format!("--allowlist={}", allowlist_path.display()),
    ];

    let cfg = HwConfig::from_cli_args(&args).expect("parse cfg");
    let info = probe_live_adapter(&cfg).expect("probe live adapter");
    assert!(info.contains("bridge_protocol"), "unexpected adapter info: {info}");

    let report_dir = template_dir.join("reports");
    let report_path = run_all_phases(
        &cfg,
        Some(0x00),
        Some(Path::new(&firmware_path)),
        &report_dir,
        ProgramOptions {
            strict_mode: true,
            execute_flash: true,
        },
    )
    .expect("run all phases");

    let report = fs::read_to_string(&report_path).expect("read report");
    assert!(report.contains("\"overall_passed\": true"), "report did not pass gate: {report}");
    assert!(report.contains("\"conformance_summary\""), "missing conformance summary: {report}");
    assert!(report.contains("\"service_sid\": \"0x36\""), "missing SID 0x36 evidence: {report}");
    assert!(report.contains("request_transfer_exit_positive=true"), "missing transfer exit evidence: {report}");
}
