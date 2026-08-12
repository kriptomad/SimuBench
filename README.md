# SimuBench (formerly AutoBreaking prototype)

SimuBench is now a production-focused heavy machinery ECU bench platform in Rust, with a Windows-first ECM live path. It is no longer an ABS-only test project: it combines full-system simulation, real hardware acquisition workflows, safety-gated write controls, auditable logs, replay tooling, and testable adapter architecture.

## 1. Executive summary

This project provides a hardware-agnostic interface for live ECM telemetry and controlled writes. Windows operators can use Cat Comm integration mode via a vendor adapter template, while serial and Linux SocketCAN paths remain available. Write operations are denied by default and require explicit enablement, allowlist scope, and operator approval. In parallel, SimuBench keeps a rich multi-ECU simulation and diagnostics UI for development, validation, and operator training.

## 2. Windows-first quickstart

Build:

```powershell
cargo build --release --features "vendor-windows"
```

Live over serial (recommended baseline before vendor SDK integration):

```powershell
cargo run --release --features "vendor-windows" -- --hw-mode=live --serial-port=COM3 --serial-baud=115200 --dry-run
```

Live with Cat Comm template mode:

```powershell
cargo run --release --features "vendor-windows" -- --hw-mode=live --vendor-name=cat_comm --dry-run
```

Simulation mode:

```powershell
cargo run -- --hw-mode=sim
```

## 3. ECM-Live Data workflow

The ECM live workflow runs in a dedicated tab and never auto-starts.

1. Detect: scan for candidate ECM source addresses.
2. Connect: perform handshake and verify response path.
3. Retrieve Data: stream live frames and decode real-time snapshot fields.
4. Stop: terminate live stream.
5. Export CSV: export rolling snapshot history for analysis.

The tab also includes a rolling post-analysis dashboard with min/avg/max RPM plus coolant and oil pressure extrema.

## 4. Adapter architecture

Adapters are selected in priority order by runtime flags:

1. Windows vendor adapter when `--vendor-name=cat_comm` is set and feature `vendor-windows` is enabled.
2. Linux SocketCAN when `--can-if=<iface>` is provided.
3. Serial adapter when `--serial-port=<port>` is provided.

Key files:

1. [src/io/hw.rs](src/io/hw.rs)
2. [src/io/serial_adapter.rs](src/io/serial_adapter.rs)
3. [src/io/socketcan_adapter.rs](src/io/socketcan_adapter.rs)
4. [src/io/vendor_cat_comm.rs](src/io/vendor_cat_comm.rs)

## 5. Cat Comm vendor binding status

Current `cat_comm` path is a fail-closed template by design. It documents the integration seam and blocks live operations until vendor SDK or bridge wiring is added.

Recommended production pattern:

1. Run vendor SDK access in an isolated bridge process.
2. Communicate with Rust via named pipes or stdio RPC.
3. Restart bridge on crash without killing the main app.

## 6. Safety model

Write path requires all conditions:

1. `--enable-write`
2. `--allowlist=<path>`
3. `--noninteractive-approved`
4. `--dry-run=false`

Without all four, physical writes are blocked.

Policy and startup audit logic:

1. [src/io/hw.rs](src/io/hw.rs)
2. [src/io/allowlist.rs](src/io/allowlist.rs)
3. [src/io/rate_limiter.rs](src/io/rate_limiter.rs)

## 7. Logging, replay, and evidence

Runtime artifacts include JSONL/GZIP startup audit and ECM live streams.

1. Append-only audit session logs.
2. RX records and snapshot records in live mode.
3. Replay conversion support for forensic analysis.

Reference:

1. [src/io/replay.rs](src/io/replay.rs)
2. [src/io/live_runner.rs](src/io/live_runner.rs)

## 8. Failure modes and mitigations

1. Permission denied on COM or driver:
Install vendor driver correctly and validate access rights.
2. Device disconnect or cable power glitch:
Use explicit reconnect strategy and keep emergency power cutoff accessible.
3. CAN bus-off or transceiver faults:
Stop writes, surface operator alerts, and require controlled recovery.
4. Parser desync on serial streams:
Keep raw logging and perform parser resync with bounded retries.
5. Allowlist misconfiguration:
Use dry-run first, change control, and signed allowlist process in production.

## 9. Metrics and observability

Internal hardware metrics counters are tracked in:

1. [src/io/metrics.rs](src/io/metrics.rs)

Recommended production extension:

1. expose Prometheus endpoint;
2. avoid high-cardinality labels;
3. alert on read/write error spikes and rate-limit bursts.

## 10. CI and test strategy

Mock integration workflow:

1. [/.github/workflows/integration-mock.yml](.github/workflows/integration-mock.yml)

Local validation:

```powershell
cargo fmt
cargo clippy --test io_mock
cargo test --test io_mock
cargo check
```

## 11. CLI flags

Live mode flags:

1. `--hw-mode=sim|live`
2. `--vendor-name=cat_comm`
3. `--serial-port=<port>`
4. `--serial-baud=<baud>`
5. `--can-if=<interface>`
6. `--enable-write`
7. `--allowlist=<path>`
8. `--noninteractive-approved`
9. `--dry-run` or `--dry-run=false`
10. `--rate-limit-global=<n>`
11. `--rate-limit-per-id=<n>`
12. `--log-dir=<path>`

## 12. Production runbook

Full long-form runbook, calculations, security controls, and rollout checklist:

1. [docs/ECM-Data.md](docs/ECM-Data.md)

## 13. Current limitations

1. Cat Comm template still needs real SDK bridge wiring.
2. Proprietary Caterpillar parser coverage requires validated captures/specs.
3. Prometheus endpoint is documented but not yet wired into runtime.

## 14. Next implementation targets

1. Wire Cat Comm bridge and recovery watchdog.
2. Add signed allowlist verification.
3. Add Prometheus exporter endpoint and alert rules.
4. Expand parser decoding with real captures.
