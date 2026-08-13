# AutoBreaking V2 Change Engineering Book

Gerado em: 2026-08-13 09:56:03

## Executive Summary
| Metric | Value |
|---|---:|
| Rust files scanned | 60 |
| Functions detected | 848 |
| Critical risk functions | 66 |
| High risk functions | 52 |
| Medium risk functions | 310 |
| Low risk functions | 420 |

## Visual Roadmap
```mermaid
flowchart LR
  W1[Wave 1\nProtocol Contracts] --> W2[Wave 2\nHardware Abstraction]
  W2 --> W3[Wave 3\nReport Schema Stabilization]
  W3 --> W4[Wave 4\nPerformance and Observability]
  W4 --> W5[Wave 5\nCI and Evidence Hardening]
```

## 1. Matriz de Risco por Funcao Critica

<details>
<summary>Expand full critical and high risk matrix</summary>

| Risco | Funcao | Local | Linha | Justificativa | Testes Minimos |
|---|---|---|---:|---|---|
| critical | handle_uds_serial_request | src/bin/cat_comm_bridge.rs | 101 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | missing_for_live_flash | src/io/hw.rs | 49 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | declared_capabilities | src/io/hw.rs | 231 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | open_real_adapter | src/io/hw.rs | 298 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | probe_live_adapter | src/io/hw.rs | 324 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | append_flash_audit | src/io/live_runner.rs | 484 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | run_flash_preflight | src/io/live_runner.rs | 749 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | j1939_uds_req_id | src/io/live_runner.rs | 816 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | is_uds_response_id | src/io/live_runner.rs | 820 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | recv_can_uds_response | src/io/live_runner.rs | 1046 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | send_uds_and_wait | src/io/live_runner.rs | 1164 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | live_flash_ecm_firmware | src/io/live_runner.rs | 1286 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | uds_response_id_matches_expected_target | src/io/live_runner.rs | 1631 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | uds_response_id_rejects_wrong_pf_ps_or_sa | src/io/live_runner.rs | 1639 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | run_all_phases | src/io/production_program.rs | 77 | impacts release gate and conformance reporting | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | run_sim_identity_check | src/io/production_program.rs | 601 | impacts release gate and conformance reporting | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | build_conformance_summary | src/io/production_program.rs | 613 | impacts release gate and conformance reporting | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | run_uds_runtime_self_test | src/io/production_program.rs | 744 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | supports | src/io/vendor_cat_comm.rs | 30 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | open | src/io/vendor_cat_comm.rs | 88 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | ensure_connected | src/io/vendor_cat_comm.rs | 154 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | send_bridge_request | src/io/vendor_cat_comm.rs | 164 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | read_bridge_response | src/io/vendor_cat_comm.rs | 182 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | append_diag_record | src/io/vendor_cat_comm.rs | 201 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | init | src/io/vendor_cat_comm.rs | 233 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | read_frame | src/io/vendor_cat_comm.rs | 246 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | try_read_frame | src/io/vendor_cat_comm.rs | 261 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | send_frame | src/io/vendor_cat_comm.rs | 273 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | close | src/io/vendor_cat_comm.rs | 286 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | adapter_info | src/io/vendor_cat_comm.rs | 305 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | resolve_bridge_exe | src/io/vendor_cat_comm.rs | 318 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | map_bridge_error | src/io/vendor_cat_comm.rs | 347 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | bridge_info_from_response | src/io/vendor_cat_comm.rs | 368 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | ensure_bridge_compatibility | src/io/vendor_cat_comm.rs | 375 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | now_ms | src/io/vendor_cat_comm.rs | 413 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | engineering_oil_catalog | src/leak_physics.rs | 496 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | uds_process_sa | src/main.rs | 6236 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | uds_send_and_log | src/main.rs | 6246 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | run_uds_flash_pipeline | src/main.rs | 6278 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | tab_uds | src/main.rs | 9040 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | fmt | src/uds.rs | 42 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | new | src/uds.rs | 237 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | tick | src/uds.rs | 276 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | process | src/uds.rs | 293 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_session_control | src/uds.rs | 320 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_ecu_reset | src/uds.rs | 353 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_clear_dtc | src/uds.rs | 395 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_read_dtc | src/uds.rs | 428 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_read_data_by_id | src/uds.rs | 504 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_security_access | src/uds.rs | 554 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_write_data_by_id | src/uds.rs | 640 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_routine_control | src/uds.rs | 684 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_request_download | src/uds.rs | 733 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_transfer_data | src/uds.rs | 767 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_request_transfer_exit | src/uds.rs | 809 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | svc_tester_present | src/uds.rs | 830 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | record_dtc_set | src/uds.rs | 850 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | nrc | src/uds.rs | 879 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | log | src/uds.rs | 883 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | service_name | src/uds.rs | 905 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | dtc_to_3bytes | src/uds.rs | 926 | impacts runtime behavior and downstream callers | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | session_default_relocks_security | src/uds.rs | 939 | impacts diagnostic protocol contract and ECU session state | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | transfer_exit_rejects_incomplete_download | src/uds.rs | 949 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | bin_path | tests/vendor_bridge_e2e.rs | 9 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | unique_temp_dir | tests/vendor_bridge_e2e.rs | 22 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| critical | cat_comm_bridge_probe_and_live_phases_flash_e2e | tests/vendor_bridge_e2e.rs | 33 | impacts firmware write path, transfer integrity, and production gate outcome | cargo check ; cargo check --features vendor-windows ; cargo test --workspace ; cargo test --features vendor-windows --test vendor_bridge_e2e ; powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1 |
| high | from_path | src/io/allowlist.rs | 33 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | is_allowed | src/io/allowlist.rs | 42 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | per_rule_rate_limit | src/io/allowlist.rs | 46 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | validate | src/io/allowlist.rs | 53 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | rule_matches_frame | src/io/allowlist.rs | 95 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | parse_u32_hex_or_dec | src/io/allowlist.rs | 137 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | parse_mask | src/io/allowlist.rs | 146 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | match_hex_pattern | src/io/allowlist.rs | 150 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | can_rule_matches_mask | src/io/allowlist.rs | 182 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | serial_rule_matches_pattern | src/io/allowlist.rs | 209 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | write_intent_enabled | src/io/hw.rs | 223 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | connect_ecm | src/io/live_runner.rs | 174 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | live_preflight_ecm | src/io/live_runner.rs | 793 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | check_live_channel_policy | src/io/live_runner.rs | 1216 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | on_rate_limited | src/io/metrics.rs | 37 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | inject_disconnect | src/io/mock.rs | 32 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | clear_disconnect | src/io/mock.rs | 36 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | read_frame | src/io/mock.rs | 61 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | try_read_frame | src/io/mock.rs | 89 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | send_frame | src/io/mock.rs | 114 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | default | src/io/production_program.rs | 69 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | run_sim_baseline_check | src/io/production_program.rs | 568 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | run_sim_connectivity_check | src/io/production_program.rs | 587 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | run_sim_preflight_check | src/io/production_program.rs | 731 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | run_sim_stress_check | src/io/production_program.rs | 763 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | hex_sha | src/io/production_program.rs | 784 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | now_ms | src/io/production_program.rs | 792 | impacts release gate and conformance reporting | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | new | src/io/rate_limiter.rs | 13 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | try_take | src/io/rate_limiter.rs | 23 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | refill | src/io/rate_limiter.rs | 33 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | new | src/io/rate_limiter.rs | 49 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | check_can | src/io/rate_limiter.rs | 57 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | check_serial | src/io/rate_limiter.rs | 70 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | limiter_blocks_when_empty | src/io/rate_limiter.rs | 80 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | aggregate_group_report | src/leak_physics.rs | 73 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | export_calibration_report_json | src/leak_physics.rs | 1654 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | export_calibration_report_csv | src/leak_physics.rs | 1670 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | report_captures_pressure_band_and_safe_window | src/leak_physics.rs | 2030 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | calibration_mode_ingests_csv_and_exports_reports | src/leak_physics.rs | 2107 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | export_leak_report_json | src/lib.rs | 1053 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | export_leak_report_csv | src/lib.rs | 1060 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | export_leak_calibration_report_json | src/lib.rs | 1118 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | export_leak_calibration_report_csv | src/lib.rs | 1126 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | pick_save_path | src/main.rs | 542 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | leak_ascii_cad | src/main.rs | 6360 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | fmt | src/v2x_telematics.rs | 322 | impacts runtime behavior and downstream callers | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | write_test_allowlist | tests/io_mock.rs | 10 | impacts write authorization and operational safety | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | mock_adapter_allows_write_when_policy_satisfied | tests/io_mock.rs | 97 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | mock_adapter_dry_run_logs_without_physical_tx | tests/io_mock.rs | 125 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | mock_adapter_rate_limit_blocks_burst | tests/io_mock.rs | 157 | impacts hardware transport abstraction and live connectivity | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | allowlist_parse_mask_accepts_hex | tests/io_mock.rs | 189 | impacts write authorization and operational safety | cargo check ; cargo test --workspace ; cargo test --test io_mock |
| high | e2e_hydraulic_plus_ac_stress_generates_reports | tests/leak_system_integration.rs | 4 | impacts evidence schema consumed by automation and approvals | cargo check ; cargo test --workspace ; cargo test --test io_mock |

</details>

## 2. Se Mudar X Entao Quebra Y

<details>
<summary>Expand full X->Y impact table with line references</summary>

| X alterado | Onde | Risco | Y afetado | Linhas de referencia | Efeito esperado |
|---|---|---|---|---|---|
| handle_uds_serial_request | src/bin/cat_comm_bridge.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/bin/cat_comm_bridge.rs:101 | impacto local com potencial efeito indireto via fluxo de controle |
| missing_for_live_flash | src/io/hw.rs | critica | src/io/production_program.rs ; src/io/production_program.rs ; src/io/production_program.rs | src/io/hw.rs:49 ; src/io/production_program.rs:94 ; src/io/production_program.rs:705 ; src/io/production_program.rs:708 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| declared_capabilities | src/io/hw.rs | critica | src/io/production_program.rs | src/io/hw.rs:231 ; src/io/production_program.rs:93 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| open_real_adapter | src/io/hw.rs | critica | src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/hw.rs:298 ; src/io/live_runner.rs:141 ; src/io/live_runner.rs:182 ; src/io/live_runner.rs:277 ; src/io/live_runner.rs:786 ; src/io/live_runner.rs:804 ; src/io/live_runner.rs:1252 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| probe_live_adapter | src/io/hw.rs | critica | src/bin/simulator_cli.rs ; tests/vendor_bridge_e2e.rs | src/io/hw.rs:324 ; src/bin/simulator_cli.rs:36 ; tests/vendor_bridge_e2e.rs:60 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| append_flash_audit | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:484 | impacto local com potencial efeito indireto via fluxo de controle |
| run_flash_preflight | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:749 | impacto local com potencial efeito indireto via fluxo de controle |
| j1939_uds_req_id | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:816 | impacto local com potencial efeito indireto via fluxo de controle |
| is_uds_response_id | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:820 | impacto local com potencial efeito indireto via fluxo de controle |
| recv_can_uds_response | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:1046 | impacto local com potencial efeito indireto via fluxo de controle |
| send_uds_and_wait | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:1164 | impacto local com potencial efeito indireto via fluxo de controle |
| live_flash_ecm_firmware | src/io/live_runner.rs | critica | src/io/production_program.rs ; src/main.rs | src/io/live_runner.rs:1286 ; src/io/production_program.rs:291 ; src/main.rs:9291 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| uds_response_id_matches_expected_target | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:1631 | impacto local com potencial efeito indireto via fluxo de controle |
| uds_response_id_rejects_wrong_pf_ps_or_sa | src/io/live_runner.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:1639 | impacto local com potencial efeito indireto via fluxo de controle |
| run_all_phases | src/io/production_program.rs | critica | src/bin/simulator_cli.rs ; tests/vendor_bridge_e2e.rs | src/io/production_program.rs:77 ; src/bin/simulator_cli.rs:73 ; tests/vendor_bridge_e2e.rs:64 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| run_sim_identity_check | src/io/production_program.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:601 | impacto local com potencial efeito indireto via fluxo de controle |
| build_conformance_summary | src/io/production_program.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:613 | impacto local com potencial efeito indireto via fluxo de controle |
| run_uds_runtime_self_test | src/io/production_program.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:744 | impacto local com potencial efeito indireto via fluxo de controle |
| supports | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:30 | impacto local com potencial efeito indireto via fluxo de controle |
| open | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/hw.rs ; src/io/hw.rs ; src/io/hw.rs ; src/io/replay.rs ; src/io/replay.rs | src/io/vendor_cat_comm.rs:88 ; src/io/hw.rs:303 ; src/io/hw.rs:310 ; src/io/hw.rs:315 ; src/io/hw.rs:391 ; src/io/replay.rs:31 ; src/io/replay.rs:42 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| ensure_connected | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:154 | impacto local com potencial efeito indireto via fluxo de controle |
| send_bridge_request | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:164 | impacto local com potencial efeito indireto via fluxo de controle |
| read_bridge_response | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:182 | impacto local com potencial efeito indireto via fluxo de controle |
| append_diag_record | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:201 | impacto local com potencial efeito indireto via fluxo de controle |
| init | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/vendor_cat_comm.rs:233 ; src/io/hw.rs:287 ; src/io/hw.rs:326 ; src/io/live_runner.rs:142 ; src/io/live_runner.rs:183 ; src/io/live_runner.rs:288 ; src/io/live_runner.rs:787 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| read_frame | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/vendor_cat_comm.rs:246 ; src/io/hw.rs:288 ; src/io/live_runner.rs:148 ; src/io/live_runner.rs:213 ; src/io/live_runner.rs:302 ; src/io/live_runner.rs:702 ; src/io/live_runner.rs:897 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| try_read_frame | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/mock.rs ; src/io/serial_adapter.rs ; src/io/socketcan_adapter.rs | src/io/vendor_cat_comm.rs:261 ; src/io/hw.rs:289 ; src/io/mock.rs:89 ; src/io/serial_adapter.rs:61 ; src/io/socketcan_adapter.rs:80 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| send_frame | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/mock.rs ; src/io/serial_adapter.rs ; src/io/socketcan_adapter.rs | src/io/vendor_cat_comm.rs:273 ; src/io/hw.rs:290 ; src/io/live_runner.rs:202 ; src/io/live_runner.rs:840 ; src/io/mock.rs:114 ; src/io/serial_adapter.rs:69 ; src/io/socketcan_adapter.rs:89 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| close | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/vendor_cat_comm.rs:286 ; src/io/hw.rs:291 ; src/io/hw.rs:330 ; src/io/live_runner.rs:162 ; src/io/live_runner.rs:168 ; src/io/live_runner.rs:203 ; src/io/live_runner.rs:227 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| adapter_info | src/io/vendor_cat_comm.rs | critica | src/io/hw.rs ; src/io/hw.rs | src/io/vendor_cat_comm.rs:305 ; src/io/hw.rs:293 ; src/io/hw.rs:328 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| resolve_bridge_exe | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:318 | impacto local com potencial efeito indireto via fluxo de controle |
| map_bridge_error | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:347 | impacto local com potencial efeito indireto via fluxo de controle |
| bridge_info_from_response | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:368 | impacto local com potencial efeito indireto via fluxo de controle |
| ensure_bridge_compatibility | src/io/vendor_cat_comm.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/io/vendor_cat_comm.rs:375 | impacto local com potencial efeito indireto via fluxo de controle |
| now_ms | src/io/vendor_cat_comm.rs | critica | src/bin/cat_comm_bridge.rs ; src/bin/cat_comm_bridge.rs ; src/io/hw.rs ; src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/vendor_cat_comm.rs:413 ; src/bin/cat_comm_bridge.rs:97 ; src/bin/cat_comm_bridge.rs:362 ; src/io/hw.rs:353 ; src/io/hw.rs:400 ; src/io/live_runner.rs:200 ; src/io/live_runner.rs:373 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| engineering_oil_catalog | src/leak_physics.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/leak_physics.rs:496 | impacto local com potencial efeito indireto via fluxo de controle |
| uds_process_sa | src/main.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/main.rs:6236 | impacto local com potencial efeito indireto via fluxo de controle |
| uds_send_and_log | src/main.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/main.rs:6246 | impacto local com potencial efeito indireto via fluxo de controle |
| run_uds_flash_pipeline | src/main.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/main.rs:6278 | impacto local com potencial efeito indireto via fluxo de controle |
| tab_uds | src/main.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/main.rs:9040 | impacto local com potencial efeito indireto via fluxo de controle |
| fmt | src/uds.rs | critica | src/adas.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs | src/uds.rs:42 ; src/adas.rs:56 ; src/autonomous.rs:16 ; src/autonomous.rs:39 ; src/autonomous.rs:75 ; src/autonomous.rs:83 ; src/autonomous.rs:173 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| new | src/uds.rs | critica | src/adas.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs | src/uds.rs:237 ; src/adas.rs:67 ; src/autonomous.rs:185 ; src/autonomous.rs:190 ; src/autonomous.rs:230 ; src/autonomous.rs:545 ; src/autonomous.rs:570 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| tick | src/uds.rs | critica | src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/bin/simulator_cli.rs | src/uds.rs:276 ; src/autonomous.rs:245 ; src/autonomous.rs:546 ; src/autonomous.rs:584 ; src/autonomous.rs:612 ; src/autonomous.rs:630 ; src/bin/simulator_cli.rs:125 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| process | src/uds.rs | critica | src/can_gateway.rs ; src/io/production_program.rs ; src/io/production_program.rs ; src/io/production_program.rs ; src/io/production_program.rs ; src/io/production_program.rs | src/uds.rs:293 ; src/can_gateway.rs:210 ; src/io/production_program.rs:603 ; src/io/production_program.rs:604 ; src/io/production_program.rs:746 ; src/io/production_program.rs:747 ; src/io/production_program.rs:748 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| svc_session_control | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:320 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_ecu_reset | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:353 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_clear_dtc | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:395 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_read_dtc | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:428 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_read_data_by_id | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:504 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_security_access | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:554 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_write_data_by_id | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:640 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_routine_control | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:684 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_request_download | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:733 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_transfer_data | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:767 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_request_transfer_exit | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:809 | impacto local com potencial efeito indireto via fluxo de controle |
| svc_tester_present | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:830 | impacto local com potencial efeito indireto via fluxo de controle |
| record_dtc_set | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:850 | impacto local com potencial efeito indireto via fluxo de controle |
| nrc | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:879 | impacto local com potencial efeito indireto via fluxo de controle |
| log | src/uds.rs | critica | src/can_gateway.rs ; src/can_gateway.rs ; src/can_gateway.rs ; src/nvm.rs | src/uds.rs:883 ; src/can_gateway.rs:110 ; src/can_gateway.rs:128 ; src/can_gateway.rs:281 ; src/nvm.rs:32 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| service_name | src/uds.rs | critica | src/main.rs | src/uds.rs:905 ; src/main.rs:6252 | pode quebrar protocolo, gate de producao ou fluxo de flash |
| dtc_to_3bytes | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:926 | impacto local com potencial efeito indireto via fluxo de controle |
| session_default_relocks_security | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:939 | impacto local com potencial efeito indireto via fluxo de controle |
| transfer_exit_rejects_incomplete_download | src/uds.rs | critica | chamadas diretas nao detectadas por varredura estatica | src/uds.rs:949 | impacto local com potencial efeito indireto via fluxo de controle |
| bin_path | tests/vendor_bridge_e2e.rs | critica | chamadas diretas nao detectadas por varredura estatica | tests/vendor_bridge_e2e.rs:9 | impacto local com potencial efeito indireto via fluxo de controle |
| unique_temp_dir | tests/vendor_bridge_e2e.rs | critica | chamadas diretas nao detectadas por varredura estatica | tests/vendor_bridge_e2e.rs:22 | impacto local com potencial efeito indireto via fluxo de controle |
| cat_comm_bridge_probe_and_live_phases_flash_e2e | tests/vendor_bridge_e2e.rs | critica | chamadas diretas nao detectadas por varredura estatica | tests/vendor_bridge_e2e.rs:33 | impacto local com potencial efeito indireto via fluxo de controle |
| from_path | src/io/allowlist.rs | alta | src/io/mock.rs ; src/leak_physics.rs | src/io/allowlist.rs:33 ; src/io/mock.rs:47 ; src/leak_physics.rs:1479 | pode quebrar readiness, relatorio ou integracao de hardware |
| is_allowed | src/io/allowlist.rs | alta | src/io/mock.rs | src/io/allowlist.rs:42 ; src/io/mock.rs:133 | pode quebrar readiness, relatorio ou integracao de hardware |
| per_rule_rate_limit | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:46 | impacto local com potencial efeito indireto via fluxo de controle |
| validate | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:53 | impacto local com potencial efeito indireto via fluxo de controle |
| rule_matches_frame | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:95 | impacto local com potencial efeito indireto via fluxo de controle |
| parse_u32_hex_or_dec | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:137 | impacto local com potencial efeito indireto via fluxo de controle |
| parse_mask | src/io/allowlist.rs | alta | tests/io_mock.rs | src/io/allowlist.rs:146 ; tests/io_mock.rs:190 | pode quebrar readiness, relatorio ou integracao de hardware |
| match_hex_pattern | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:150 | impacto local com potencial efeito indireto via fluxo de controle |
| can_rule_matches_mask | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:182 | impacto local com potencial efeito indireto via fluxo de controle |
| serial_rule_matches_pattern | src/io/allowlist.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/allowlist.rs:209 | impacto local com potencial efeito indireto via fluxo de controle |
| write_intent_enabled | src/io/hw.rs | alta | src/io/mock.rs ; tests/io_mock.rs | src/io/hw.rs:223 ; src/io/mock.rs:125 ; tests/io_mock.rs:140 | pode quebrar readiness, relatorio ou integracao de hardware |
| connect_ecm | src/io/live_runner.rs | alta | src/io/production_program.rs ; src/main.rs | src/io/live_runner.rs:174 ; src/io/production_program.rs:171 ; src/main.rs:2655 | pode quebrar readiness, relatorio ou integracao de hardware |
| live_preflight_ecm | src/io/live_runner.rs | alta | src/io/production_program.rs | src/io/live_runner.rs:793 ; src/io/production_program.rs:271 | pode quebrar readiness, relatorio ou integracao de hardware |
| check_live_channel_policy | src/io/live_runner.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/live_runner.rs:1216 | impacto local com potencial efeito indireto via fluxo de controle |
| on_rate_limited | src/io/metrics.rs | alta | src/io/mock.rs | src/io/metrics.rs:37 ; src/io/mock.rs:149 | pode quebrar readiness, relatorio ou integracao de hardware |
| inject_disconnect | src/io/mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/mock.rs:32 | impacto local com potencial efeito indireto via fluxo de controle |
| clear_disconnect | src/io/mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/mock.rs:36 | impacto local com potencial efeito indireto via fluxo de controle |
| read_frame | src/io/mock.rs | alta | src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/mock.rs:61 ; src/io/hw.rs:288 ; src/io/live_runner.rs:148 ; src/io/live_runner.rs:213 ; src/io/live_runner.rs:302 ; src/io/live_runner.rs:702 ; src/io/live_runner.rs:897 | pode quebrar readiness, relatorio ou integracao de hardware |
| try_read_frame | src/io/mock.rs | alta | src/io/hw.rs ; src/io/serial_adapter.rs ; src/io/socketcan_adapter.rs ; src/io/vendor_cat_comm.rs | src/io/mock.rs:89 ; src/io/hw.rs:289 ; src/io/serial_adapter.rs:61 ; src/io/socketcan_adapter.rs:80 ; src/io/vendor_cat_comm.rs:261 | pode quebrar readiness, relatorio ou integracao de hardware |
| send_frame | src/io/mock.rs | alta | src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs ; src/io/serial_adapter.rs ; src/io/socketcan_adapter.rs ; src/io/socketcan_adapter.rs | src/io/mock.rs:114 ; src/io/hw.rs:290 ; src/io/live_runner.rs:202 ; src/io/live_runner.rs:840 ; src/io/serial_adapter.rs:69 ; src/io/socketcan_adapter.rs:89 ; src/io/socketcan_adapter.rs:124 | pode quebrar readiness, relatorio ou integracao de hardware |
| default | src/io/production_program.rs | alta | src/autonomous.rs ; src/bin/cat_comm_bridge.rs ; src/bin/cat_comm_bridge.rs ; src/boot_sequence.rs ; src/camera.rs ; src/camera.rs | src/io/production_program.rs:69 ; src/autonomous.rs:184 ; src/bin/cat_comm_bridge.rs:59 ; src/bin/cat_comm_bridge.rs:184 ; src/boot_sequence.rs:178 ; src/camera.rs:142 ; src/camera.rs:246 | pode quebrar readiness, relatorio ou integracao de hardware |
| run_sim_baseline_check | src/io/production_program.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:568 | impacto local com potencial efeito indireto via fluxo de controle |
| run_sim_connectivity_check | src/io/production_program.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:587 | impacto local com potencial efeito indireto via fluxo de controle |
| run_sim_preflight_check | src/io/production_program.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:731 | impacto local com potencial efeito indireto via fluxo de controle |
| run_sim_stress_check | src/io/production_program.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:763 | impacto local com potencial efeito indireto via fluxo de controle |
| hex_sha | src/io/production_program.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/production_program.rs:784 | impacto local com potencial efeito indireto via fluxo de controle |
| now_ms | src/io/production_program.rs | alta | src/bin/cat_comm_bridge.rs ; src/bin/cat_comm_bridge.rs ; src/io/hw.rs ; src/io/hw.rs ; src/io/live_runner.rs ; src/io/live_runner.rs | src/io/production_program.rs:792 ; src/bin/cat_comm_bridge.rs:97 ; src/bin/cat_comm_bridge.rs:362 ; src/io/hw.rs:353 ; src/io/hw.rs:400 ; src/io/live_runner.rs:200 ; src/io/live_runner.rs:373 | pode quebrar readiness, relatorio ou integracao de hardware |
| new | src/io/rate_limiter.rs | alta | src/adas.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs | src/io/rate_limiter.rs:13 ; src/adas.rs:67 ; src/autonomous.rs:185 ; src/autonomous.rs:190 ; src/autonomous.rs:230 ; src/autonomous.rs:545 ; src/autonomous.rs:570 | pode quebrar readiness, relatorio ou integracao de hardware |
| try_take | src/io/rate_limiter.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/rate_limiter.rs:23 | impacto local com potencial efeito indireto via fluxo de controle |
| refill | src/io/rate_limiter.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/rate_limiter.rs:33 | impacto local com potencial efeito indireto via fluxo de controle |
| new | src/io/rate_limiter.rs | alta | src/adas.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs | src/io/rate_limiter.rs:49 ; src/adas.rs:67 ; src/autonomous.rs:185 ; src/autonomous.rs:190 ; src/autonomous.rs:230 ; src/autonomous.rs:545 ; src/autonomous.rs:570 | pode quebrar readiness, relatorio ou integracao de hardware |
| check_can | src/io/rate_limiter.rs | alta | src/io/mock.rs | src/io/rate_limiter.rs:57 ; src/io/mock.rs:141 | pode quebrar readiness, relatorio ou integracao de hardware |
| check_serial | src/io/rate_limiter.rs | alta | src/io/mock.rs | src/io/rate_limiter.rs:70 ; src/io/mock.rs:142 | pode quebrar readiness, relatorio ou integracao de hardware |
| limiter_blocks_when_empty | src/io/rate_limiter.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/io/rate_limiter.rs:80 | impacto local com potencial efeito indireto via fluxo de controle |
| aggregate_group_report | src/leak_physics.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/leak_physics.rs:73 | impacto local com potencial efeito indireto via fluxo de controle |
| export_calibration_report_json | src/leak_physics.rs | alta | src/lib.rs | src/leak_physics.rs:1654 ; src/lib.rs:1123 | pode quebrar readiness, relatorio ou integracao de hardware |
| export_calibration_report_csv | src/leak_physics.rs | alta | src/lib.rs | src/leak_physics.rs:1670 ; src/lib.rs:1131 | pode quebrar readiness, relatorio ou integracao de hardware |
| report_captures_pressure_band_and_safe_window | src/leak_physics.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/leak_physics.rs:2030 | impacto local com potencial efeito indireto via fluxo de controle |
| calibration_mode_ingests_csv_and_exports_reports | src/leak_physics.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/leak_physics.rs:2107 | impacto local com potencial efeito indireto via fluxo de controle |
| export_leak_report_json | src/lib.rs | alta | src/main.rs ; tests/leak_system_integration.rs | src/lib.rs:1053 ; src/main.rs:1063 ; tests/leak_system_integration.rs:46 | pode quebrar readiness, relatorio ou integracao de hardware |
| export_leak_report_csv | src/lib.rs | alta | src/main.rs ; tests/leak_system_integration.rs | src/lib.rs:1060 ; src/main.rs:1057 ; tests/leak_system_integration.rs:49 | pode quebrar readiness, relatorio ou integracao de hardware |
| export_leak_calibration_report_json | src/lib.rs | alta | src/main.rs | src/lib.rs:1118 ; src/main.rs:1142 | pode quebrar readiness, relatorio ou integracao de hardware |
| export_leak_calibration_report_csv | src/lib.rs | alta | src/main.rs | src/lib.rs:1126 ; src/main.rs:1130 | pode quebrar readiness, relatorio ou integracao de hardware |
| pick_save_path | src/main.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/main.rs:542 | impacto local com potencial efeito indireto via fluxo de controle |
| leak_ascii_cad | src/main.rs | alta | chamadas diretas nao detectadas por varredura estatica | src/main.rs:6360 | impacto local com potencial efeito indireto via fluxo de controle |
| fmt | src/v2x_telematics.rs | alta | src/adas.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs ; src/autonomous.rs | src/v2x_telematics.rs:322 ; src/adas.rs:56 ; src/autonomous.rs:16 ; src/autonomous.rs:39 ; src/autonomous.rs:75 ; src/autonomous.rs:83 ; src/autonomous.rs:173 | pode quebrar readiness, relatorio ou integracao de hardware |
| write_test_allowlist | tests/io_mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | tests/io_mock.rs:10 | impacto local com potencial efeito indireto via fluxo de controle |
| mock_adapter_allows_write_when_policy_satisfied | tests/io_mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | tests/io_mock.rs:97 | impacto local com potencial efeito indireto via fluxo de controle |
| mock_adapter_dry_run_logs_without_physical_tx | tests/io_mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | tests/io_mock.rs:125 | impacto local com potencial efeito indireto via fluxo de controle |
| mock_adapter_rate_limit_blocks_burst | tests/io_mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | tests/io_mock.rs:157 | impacto local com potencial efeito indireto via fluxo de controle |
| allowlist_parse_mask_accepts_hex | tests/io_mock.rs | alta | chamadas diretas nao detectadas por varredura estatica | tests/io_mock.rs:189 | impacto local com potencial efeito indireto via fluxo de controle |
| e2e_hydraulic_plus_ac_stress_generates_reports | tests/leak_system_integration.rs | alta | chamadas diretas nao detectadas por varredura estatica | tests/leak_system_integration.rs:4 | impacto local com potencial efeito indireto via fluxo de controle |

</details>

## 3. Roadmap de Refatoracao Segura por Ondas

### Wave 1 - Blindagem de Contratos de Protocolo
- Escopo: src/io/live_runner.rs, src/io/vendor_cat_comm.rs, src/uds.rs
- Mudancas: centralizar validacao de SID positivo, tipar NRC, consolidar parse de DIDs
- Checklist de testes:
  - [ ] cargo check
  - [ ] cargo test --workspace
  - [ ] cargo test --features vendor-windows --test vendor_bridge_e2e

### Wave 2 - Refino de Abstracao de Hardware
- Escopo: src/io/hw.rs, src/io/socketcan_adapter.rs, src/io/serial_adapter.rs
- Mudancas: contratos de capability com traits menores, erros padronizados por camada
- Checklist de testes:
  - [ ] cargo check --features vendor-windows
  - [ ] cargo test --workspace
  - [ ] cargo test --test io_mock

### Wave 3 - Estabilizacao de Schema de Relatorio
- Escopo: src/io/production_program.rs, src/bin/simulator_cli.rs, docs/ECM-Data.md
- Mudancas: versionamento de schema, compat adapters para consumidores antigos
- Checklist de testes:
  - [ ] cargo check
  - [ ] cargo test --workspace
  - [ ] powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_windows_template_e2e.ps1

### Wave 4 - Performance e Observabilidade
- Escopo: src/main.rs, src/observability.rs, src/io/metrics.rs
- Mudancas: reduzir jitter de render e padronizar eventos de telemetria
- Checklist de testes:
  - [ ] cargo check
  - [ ] cargo test --workspace

### Wave 5 - Harden de CI e Evidencia
- Escopo: .github/workflows/*.yml, scripts/*.ps1, scripts/*.sh
- Mudancas: gates obrigatorios por risco, artefatos de report/hash sempre anexados
- Checklist de testes:
  - [ ] cargo check
  - [ ] cargo test --workspace
  - [ ] cargo test --features vendor-windows --test vendor_bridge_e2e

## Cobertura e Limites
- Esta edicao prioriza funcoes de alto impacto e rastreabilidade de chamadas por varredura estatica.
- Chamadas dinamicas e dispatch indireto podem nao aparecer na tabela X->Y.