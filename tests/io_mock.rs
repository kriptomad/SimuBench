#![allow(dead_code)]

#[path = "../src/io/mod.rs"]
mod io;

use io::hw::{CanFrame, Frame, HardwareInterface, HwConfig, HwError, HwMode};
use io::mock::MockAdapter;
use std::fs;

fn write_test_allowlist(path: &std::path::Path) {
    let content = r#"[
    {
        "type": "can",
        "id": 419385573,
        "mask": "0x1FFFFFFF",
        "allowed_bytes": [[0, 8]],
        "max_rate_per_sec": 10,
        "description": "allow test can id"
    }
]"#;
    fs::write(path, content).expect("allowlist write");
}

#[test]
fn hw_config_parses_cli_flags() {
    let args = vec![
        "simubench".to_string(),
        "--hw-mode=live".to_string(),
        "--dry-run".to_string(),
        "--allowlist=./allowlist.json".to_string(),
        "--enable-write".to_string(),
        "--noninteractive-approved".to_string(),
    ];

    let cfg = HwConfig::from_cli_args(&args).expect("valid args");
    assert_eq!(cfg.mode, HwMode::Live);
    assert!(cfg.dry_run);
    assert_eq!(
        cfg.allowlist_path.as_deref(),
        Some(std::path::Path::new("./allowlist.json"))
    );
    // dry-run must keep writes disabled effectively
    assert!(!cfg.write_effectively_enabled());
}

#[test]
fn mock_adapter_blocks_write_by_default_and_reads_injected_frame() {
    let cfg = HwConfig::default();
    let mut ad = MockAdapter::new();
    ad.init(&cfg).expect("init ok");

    let f = Frame::Can(CanFrame {
        id: 0x18FF50E5,
        dlc: 8,
        data: [1, 2, 3, 4, 5, 6, 7, 8],
        len: 8,
        timestamp_ms: Some(1),
    });

    let err = ad.send_frame(f.clone()).expect_err("write must be blocked");
    assert!(matches!(err, HwError::WriteBlockedAllowlist));

    ad.inject_rx(f.clone());
    let got = ad.read_frame().expect("read injected frame");
    assert_eq!(got, f);
}

#[test]
fn mock_adapter_allows_write_when_policy_satisfied() {
    let temp = std::env::temp_dir().join("allowlist_io_mock.json");
    write_test_allowlist(&temp);

    let args = vec![
        "simubench".to_string(),
        "--enable-write".to_string(),
        format!("--allowlist={}", temp.display()),
        "--noninteractive-approved".to_string(),
        "--dry-run=false".to_string(),
    ];
    let cfg = HwConfig::from_cli_args(&args).expect("valid args");
    assert!(cfg.write_effectively_enabled());

    let mut ad = MockAdapter::new();
    ad.init(&cfg).expect("init ok");
    let f = Frame::Can(CanFrame {
        id: 0x18FF50E5,
        dlc: 8,
        data: [0; 8],
        len: 8,
        timestamp_ms: None,
    });
    ad.send_frame(f).expect("write allowed");
    assert_eq!(ad.tx_queue.len(), 1);
}

#[test]
fn mock_adapter_dry_run_logs_without_physical_tx() {
    let temp_allow = std::env::temp_dir().join("allowlist_io_mock_dry_run.json");
    let temp_log_dir = std::env::temp_dir().join("autobreaking_hw_logs_dry_run");
    write_test_allowlist(&temp_allow);

    let args = vec![
        "simubench".to_string(),
        "--enable-write".to_string(),
        format!("--allowlist={}", temp_allow.display()),
        "--noninteractive-approved".to_string(),
        "--dry-run".to_string(),
        format!("--log-dir={}", temp_log_dir.display()),
    ];
    let cfg = HwConfig::from_cli_args(&args).expect("valid args");
    assert!(!cfg.write_effectively_enabled());
    assert!(cfg.write_intent_enabled());

    let mut ad = MockAdapter::new();
    ad.init(&cfg).expect("init ok");
    let f = Frame::Can(CanFrame {
        id: 0x18FF50E5,
        dlc: 8,
        data: [0; 8],
        len: 8,
        timestamp_ms: None,
    });

    ad.send_frame(f).expect("dry-run should allow intent");
    assert_eq!(ad.tx_queue.len(), 0);
}

#[test]
fn mock_adapter_rate_limit_blocks_burst() {
    let temp_allow = std::env::temp_dir().join("allowlist_io_mock_rate_limit.json");
    write_test_allowlist(&temp_allow);

    let args = vec![
        "simubench".to_string(),
        "--enable-write".to_string(),
        format!("--allowlist={}", temp_allow.display()),
        "--noninteractive-approved".to_string(),
        "--dry-run=false".to_string(),
        "--rate-limit-global=1".to_string(),
        "--rate-limit-per-id=1".to_string(),
    ];
    let cfg = HwConfig::from_cli_args(&args).expect("valid args");

    let mut ad = MockAdapter::new();
    ad.init(&cfg).expect("init ok");

    let f = Frame::Can(CanFrame {
        id: 0x18FF50E5,
        dlc: 8,
        data: [0; 8],
        len: 8,
        timestamp_ms: None,
    });

    ad.send_frame(f.clone()).expect("first send ok");
    let err = ad.send_frame(f).expect_err("second send should rate-limit");
    assert!(matches!(err, HwError::RateLimited));
}

#[test]
fn allowlist_parse_mask_accepts_hex() {
    let mask = io::allowlist::parse_mask("0x1FFFFFFF").expect("mask parse");
    assert_eq!(mask, 0x1FFF_FFFF);
}

#[test]
fn replay_can_record_keeps_explicit_timestamp() {
    let f = Frame::Can(CanFrame {
        id: 0x18FF50E5,
        dlc: 8,
        data: [1, 2, 3, 4, 5, 6, 7, 8],
        len: 8,
        timestamp_ms: Some(999),
    });

    let rec = io::replay::frame_to_record(&f, "rx", None, None);
    assert_eq!(rec.ts, 999);
    assert!(rec.raw_hex.is_none());
}
