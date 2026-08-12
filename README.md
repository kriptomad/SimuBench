# SimuBench

SimuBench is a full heavy-machinery ECU bench platform in Rust.

It combines three major capabilities in one system:

1. High-fidelity multi-ECU simulation and diagnostics bench.
2. Real ECM live communication workflow (Detect, Connect, Retrieve, Stop, Export CSV).
3. Safety-first hardware I/O layer with allowlist, rate limiting, dry-run controls, and auditable logs.

This is not an ABS-only project anymore. ABS/ESP/TCS is one subsystem inside a broader platform that includes powertrain, transmission, hydraulics, CAN network, UDS diagnostics, autonomous sensor stack, V2X/telematics, leak physics, and operator-focused desktop workflows.

## What the platform does

### 1. Multi-ECU real-time simulation

Simulates a complete heavy-machinery electronic architecture with interacting ECUs:

1. ECM, TCM, BCM, ICM, HCM, ABS/ESP/TCS, VCM.
2. Boot sequence, ignition states, node online/offline behavior.
3. Fault injection catalog and DTC-driven diagnostics behavior.
4. Cross-module state propagation through CAN/J1939 and network management.

Core orchestrator and module exports are defined in [src/lib.rs](src/lib.rs).

### 2. CAN/J1939 network and diagnostics

Implements protocol and network behavior for realistic bench analysis:

1. J1939 frame flow and decode primitives.
2. Multi-bus CAN simulation with health/error counters.
3. UDS diagnostic server flows for service-level testing.
4. OSEK-style network management and NVM persistence hooks.

Relevant modules:

1. [src/j1939.rs](src/j1939.rs)
2. [src/can_gateway.rs](src/can_gateway.rs)
3. [src/can_network.rs](src/can_network.rs)
4. [src/uds.rs](src/uds.rs)
5. [src/network_mgmt.rs](src/network_mgmt.rs)
6. [src/nvm.rs](src/nvm.rs)

### 3. Autonomous and sensing stack

The bench includes perception/control subsystems for AD scenarios:

1. GPS, IMU, radar, lidar, and camera simulation.
2. Sensor fusion and autonomous controller integration.
3. V2X and telematics behavior in the same runtime loop.

Relevant modules:

1. [src/gps.rs](src/gps.rs)
2. [src/imu.rs](src/imu.rs)
3. [src/radar.rs](src/radar.rs)
4. [src/lidar.rs](src/lidar.rs)
5. [src/camera.rs](src/camera.rs)
6. [src/autonomous.rs](src/autonomous.rs)
7. [src/v2x_telematics.rs](src/v2x_telematics.rs)

### 4. Leak physics and engineering analysis

Includes hydraulic leak modeling and report generation support for engineering use:

1. Circuit-level leak simulation.
2. Scenario prediction workflow.
3. Export support for analysis/reporting.

Module:

1. [src/leak_physics.rs](src/leak_physics.rs)

### 5. ECM live data workflow

In the desktop app, ECM live communication runs in a dedicated tab and is operator-triggered only.

Workflow:

1. Detect candidate ECMs.
2. Connect and verify response path.
3. Retrieve Data in real time.
4. Stop acquisition explicitly.
5. Export live history to CSV.

The tab also provides a rolling post-analysis summary (min/avg/max and key extrema).

Key modules:

1. [src/main.rs](src/main.rs)
2. [src/io/live_runner.rs](src/io/live_runner.rs)
3. [src/io/ecm_params.rs](src/io/ecm_params.rs)

## Desktop application capabilities

Main GUI binary:

1. [src/main.rs](src/main.rs)

Main tabs include:

1. Cluster.
2. CAN Bus monitor.
3. Events.
4. ECU network.
5. Engine.
6. Faults.
7. Boot.
8. Implements.
9. Params.
10. Sensors.
11. Autonomous.
12. V2X.
13. UDS.
14. ECM Live.
15. Leak Lab.
16. Plots.

## Hardware I/O architecture

I/O abstraction and policy layer is under [src/io](src/io):

1. [src/io/hw.rs](src/io/hw.rs): config, trait contract, CLI parsing, startup audit log.
2. [src/io/serial_adapter.rs](src/io/serial_adapter.rs): real serial transport.
3. [src/io/socketcan_adapter.rs](src/io/socketcan_adapter.rs): Linux SocketCAN transport.
4. [src/io/vendor_cat_comm.rs](src/io/vendor_cat_comm.rs): Windows-first Cat Comm template.
5. [src/io/allowlist.rs](src/io/allowlist.rs): write authorization rules.
6. [src/io/rate_limiter.rs](src/io/rate_limiter.rs): global and per-ID throttling.
7. [src/io/replay.rs](src/io/replay.rs): JSONL conversion/replay support.
8. [src/io/metrics.rs](src/io/metrics.rs): hardware metrics model.
9. [src/io/mock.rs](src/io/mock.rs): mock adapter for CI and tests.

Adapter selection priority:

1. Windows vendor adapter when vendor is requested and feature is enabled.
2. CAN interface when provided.
3. Serial port when provided.

## Safety model (write control)

Physical write is blocked unless all gates are satisfied:

1. enable-write.
2. allowlist path provided and valid.
3. noninteractive approved.
4. dry-run explicitly disabled.

Safe-by-default behavior:

1. default mode is read/sim safe.
2. dry-run prevents physical TX while preserving audit evidence.
3. startup policy is recorded to log artifacts.

## Logging, replay, and observability

The platform supports evidence and post-analysis flows:

1. startup policy logs (JSONL/GZIP).
2. live RX/snapshot records.
3. replay-friendly conversions.
4. in-process metrics structures for operational counters.

Related modules:

1. [src/io/replay.rs](src/io/replay.rs)
2. [src/io/live_runner.rs](src/io/live_runner.rs)
3. [src/io/metrics.rs](src/io/metrics.rs)
4. [src/observability.rs](src/observability.rs)

## Binaries

This repository currently ships two binaries:

1. auto_breaking: desktop ECU bench app.
2. simulator_cli: headless CLI simulation runner.

CLI binary source:

1. [src/bin/simulator_cli.rs](src/bin/simulator_cli.rs)

Cargo default-run is configured so plain cargo run starts auto_breaking.

## Build and run

### Desktop app (default)

```powershell
cargo run -- --hw-mode=sim
```

### Explicit binaries

```powershell
cargo run --bin auto_breaking -- --hw-mode=sim
cargo run --bin simulator_cli -- --seed 7 --steps 20000 --model reduced
```

### Windows-first live build

```powershell
cargo build --release --features "vendor-windows"
```

### Live over serial

```powershell
cargo run --release --features "vendor-windows" -- --hw-mode=live --serial-port=COM3 --serial-baud=115200 --dry-run
```

### Live with Cat Comm template mode

```powershell
cargo run --release --features "vendor-windows" -- --hw-mode=live --vendor-name=cat_comm --dry-run
```

## Runtime flags for live mode

Supported live flags include:

1. --hw-mode=sim|live
2. --vendor-name=cat_comm
3. --serial-port
4. --serial-baud
5. --can-if
6. --enable-write
7. --allowlist
8. --noninteractive-approved
9. --dry-run or --dry-run=false
10. --rate-limit-global
11. --rate-limit-per-id
12. --log-dir

## Testing and quality

Main integration test for I/O safety behavior:

1. [tests/io_mock.rs](tests/io_mock.rs)

Local validation commands:

```powershell
cargo fmt
cargo clippy --workspace --all-targets
cargo clippy --test io_mock
cargo test --locked --workspace
cargo test --test io_mock
cargo check
```

## CI workflows

CI and quality workflows are under [.github/workflows](.github/workflows):

1. [ci.yml](.github/workflows/ci.yml): formatting, clippy, and workspace tests.
2. [integration-mock.yml](.github/workflows/integration-mock.yml): focused mock integration tests on Linux and Windows (including vendor-windows build check).
3. [fuzz-smoke.yml](.github/workflows/fuzz-smoke.yml): fuzz smoke run.
4. [nightly-montecarlo.yml](.github/workflows/nightly-montecarlo.yml): scheduled simulation workload.

## Repository map

Top-level code domains:

1. [src/lib.rs](src/lib.rs): simulation orchestrator and module exports.
2. [src/main.rs](src/main.rs): desktop bench UI and operator workflows.
3. [src/io](src/io): live hardware abstraction and safety controls.
4. [src/bin](src/bin): CLI runner.
5. [tests](tests): integration tests.
6. [docs](docs): runbooks and technical documentation.
7. [MANUAL_TECNICO.html](MANUAL_TECNICO.html): full technical manual with code break-down.

## Current status and limitations

Current state:

1. Full simulation platform is functional.
2. ECM-Live workflow is integrated in UI.
3. Serial and SocketCAN adapters are implemented.
4. Cat Comm adapter path exists as a fail-closed template.

Known limitations:

1. Cat Comm requires vendor SDK or bridge wiring for full production communications.
2. Proprietary parser expansion depends on validated captures/specs.
3. Prometheus exporter endpoint is planned but not yet fully exposed.

## Documentation

Additional documentation:

1. [docs/ECM-Data.md](docs/ECM-Data.md): production runbook and operational guidance.
2. [MANUAL_TECNICO.html](MANUAL_TECNICO.html): deep technical manual and code break-down.

