# ECM-Data Production Runbook (Windows-first)

## 0. Executive summary

AutoBreaking includes a production-grade ECM live integration layer with safe-by-default writes, operator-gated actions, audit logging, replay support, and test adapters. Windows operation is first-class through a Cat Comm integration seam and COM serial fallback.

## 1. Goals and non-goals

Goals:

1. Reliable asynchronous read and controlled write policy for live ECM communications.
2. Safe-by-default write posture with deny-unless-explicit gates.
3. Full audit trail via JSONL logs and startup policy evidence.
4. CI and testability without hardware using mock adapters.

Non-goals (current phase):

1. Bundling proprietary vendor SDK binaries in repository.
2. Full proprietary parser coverage without approved captures/specs.

## 2. Hardware adapter model

The adapter contract is centralized in [src/io/hw.rs](src/io/hw.rs).

Implementations:

1. Serial adapter for COM links: [src/io/serial_adapter.rs](src/io/serial_adapter.rs)
2. SocketCAN adapter for Linux: [src/io/socketcan_adapter.rs](src/io/socketcan_adapter.rs)
3. Cat Comm template (Windows): [src/io/vendor_cat_comm.rs](src/io/vendor_cat_comm.rs)
4. Mock adapter for tests: [src/io/mock.rs](src/io/mock.rs)

Selection order:

1. vendor_name=cat_comm (Windows + feature vendor-windows)
2. can-if
3. serial-port

## 3. Safety controls

Write must satisfy all controls:

1. enable-write true
2. allowlist path present and valid
3. noninteractive-approved true
4. dry-run false

Code references:

1. [src/io/hw.rs](src/io/hw.rs)
2. [src/io/allowlist.rs](src/io/allowlist.rs)
3. [src/io/rate_limiter.rs](src/io/rate_limiter.rs)

## 4. Allowlist schema and enforcement

Example file:

1. [allowlist.example.json](allowlist.example.json)

Current rule capabilities:

1. CAN id + optional mask
2. payload windows with allowed_bytes
3. serial hex patterns with wildcard byte pairs
4. optional per-rule max_rate_per_sec

Recommended hardening:

1. signed allowlist plus detached signature
2. validation at startup/reload
3. operator identity trace in audit stream

## 5. Rate limiter sizing

Two-level limiter model:

1. Global token bucket
2. Per-CAN-ID token bucket

Recommended conservative profile:

1. global rate: 100 tx/s
2. global burst: 300 tokens
3. per-id rate: 1 tx/s
4. per-id burst: 5 tokens

Token bucket equation:

$$tokens(t) = min(C, tokens(t_0) + r (t - t_0))$$

## 6. Log and storage sizing

Live outputs:

1. startup audit gzip jsonl
2. ecm_rx jsonl
3. ecm_snapshot jsonl
4. exported CSV snapshots from ECM-Live tab

Sizing assumptions:

1. CAN RX line ~120 bytes
2. At 1000 frames/s:
1. per day raw: about 10.4 GB
2. 30-day raw: about 312 GB
3. gzip reduction commonly 4x to 8x

## 7. Metrics model

Current in-process counters:

1. [src/io/metrics.rs](src/io/metrics.rs)

Recommended Prometheus extension:

1. hw_rx_frames_total by transport
2. hw_tx_frames_total by transport and allowed verdict
3. hw_read_errors_total and hw_write_errors_total
4. hw_rate_limited_total
5. hw_last_rx_timestamp

Cardinality guidance:

1. Avoid frame-id labels in primary timeseries.
2. Use aggregated source labels only.

## 8. Failure modes and mitigations

1. COM permission denied:
Validate driver install and process privileges.
2. Device disconnect:
Use reconnect backoff and operator alerts.
3. Bus-off or transceiver error:
Stop writes and require controlled recovery sequence.
4. Parser corruption:
Use resync boundaries and retain raw forensic logs.
5. Allowlist regression:
Require staged rollout: dry-run first, then controlled write.
6. Vendor SDK crash:
Run vendor SDK in bridge subprocess with watchdog restart.

## 9. Windows Cat Comm runbook

Preconditions:

1. bench isolation and operator authorization
2. emergency kill-switch available
3. validated COM/vendor driver path

Checks:

```powershell
Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Caption
pnputil /enum-drivers | findstr /i "Cat"
```

Safe flow:

1. launch with dry-run
2. Detect then Connect then Retrieve Data in ECM-Live tab
3. verify live snapshot updates and jsonl outputs
4. export CSV and review trends
5. only then move to dry-run=false with explicit approvals

## 10. CI and test matrix

Workflow:

1. [/.github/workflows/integration-mock.yml](.github/workflows/integration-mock.yml)

Coverage:

1. formatting
2. clippy target tests
3. io mock integration suite

Local command set:

```powershell
cargo fmt
cargo clippy --test io_mock
cargo test --test io_mock
cargo check
```

## 11. Security checklist

1. default deny writes
2. least privilege runtime account
3. protected allowlist storage and ACLs
4. append-only audit logs
5. metrics endpoint restricted to trusted network path
6. no secrets in repo or plain-text configs

## 12. Upgrade roadmap

1. implement Cat Comm real bridge bindings
2. add signed allowlist verification
3. add Prometheus exporter endpoint
4. parser extension with real captures
5. optional HIL Windows runner workflow
