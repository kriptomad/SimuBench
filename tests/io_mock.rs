#![allow(dead_code)]

#[path = "../src/io/mod.rs"]
mod io;

use io::hw::{CanFrame, Frame, HardwareInterface, HwConfig, HwError, HwMode};
use io::mock::MockAdapter;

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
    let args = vec![
        "simubench".to_string(),
        "--enable-write".to_string(),
        "--allowlist=./allowlist.json".to_string(),
        "--noninteractive-approved".to_string(),
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
