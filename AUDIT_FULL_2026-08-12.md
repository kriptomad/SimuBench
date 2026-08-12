# Full Technical Audit - AutoBreaking

Date: 2026-08-12
Scope: full codebase audit with static review + test/lint/build + deterministic runtime scenarios
Mode: read-only audit with evidence collection and prioritized remediation plan

## 1. What was executed

- `cargo check --workspace`
- `cargo clippy --workspace --all-targets`
- `cargo test --workspace`
- `cargo run --bin simulator_cli -- --seed {1,7,42,99,123456} --steps 12000`
- `cargo run --bin simulator_cli -- --replay sample.can --verify`
- 3 parallel deep audits via Explore agent:
  - Core logic and physics modules
  - Integration/protocol/API modules
  - UI/GUI/UX modules

## 2. Execution evidence summary

- Build: PASS
- Tests: PASS (all workspace tests green)
- Clippy: PASS with warnings only (no build-blocking defects)
- CLI deterministic scenarios: PASS across all seeds tested

Scenario snapshots (12000 steps each):

- seed=1: can_errors=0, can_health=0.9415, fuel_pct=84.1967
- seed=7: can_errors=0, can_health=0.9415, fuel_pct=84.1897
- seed=42: can_errors=0, can_health=0.9415, fuel_pct=84.1958
- seed=99: can_errors=0, can_health=0.9415, fuel_pct=84.1894
- seed=123456: can_errors=0, can_health=0.9415, fuel_pct=84.1981

Replay path:

- `replay_file=sample.can verify=true status=ok`

## 3. Severity summary

Legend:

- CONFIRMED: manually re-checked in source by this audit run
- AGENT-FLAGGED: raised by subagent and needs targeted confirmation test before merge

Totals:

- Critical: 0 CONFIRMED, 3 AGENT-FLAGGED
- High: 2 CONFIRMED, 8 AGENT-FLAGGED
- Medium: 5 CONFIRMED, 12 AGENT-FLAGGED

## 4. Confirmed findings (highest priority)

### High

1) Handshake acceptance in live ECM connect is broad and can accept unrelated CAN response source as success.

- File: `src/io/live_runner.rs`
- Function: `connect_ecm`
- Risk: false positive in live connection readiness.
- Action: enforce PGN/SA response validation against expected request set.

2) Background live thread errors are only printed to stderr and not propagated to structured health state.

- File: `src/io/live_runner.rs`
- Function: `start_retrieve_data`
- Risk: silent degraded live data with stale snapshot.
- Action: add shared health/error state and surface to UI + observability.

### Medium

3) Autonomous ACC integral can remain accumulated across traffic state transitions.

- File: `src/autonomous.rs`
- Function: `run_acc`
- Risk: transient overshoot when lead target disappears/reappears.
- Action: reset or decay integral when lead is infinite/unreliable.

4) Clippy indicates maintainability debt in UI control flow.

- File: `src/main.rs`
- Warnings: `match_single_binding`, `useless_format`
- Risk: low runtime impact, higher maintenance cost.
- Action: clean up for readability and safer future edits.

5) UI command burst risk on high-frequency sliders (throttle/brake update cadence).

- File: `src/main.rs`
- Risk: extra queue pressure under sustained drag.
- Action: introduce small debounce/rate-limit for command enqueue.

6) CAN connect path validation should be hardened with timeout/retry reason metadata.

- File: `src/io/live_runner.rs`
- Risk: poor diagnosability in field failures.
- Action: classify failures by stage and expose code/reason.

7) UI state synchronization remains complex and fragile in parameter pages.

- File: `src/main.rs`
- Risk: stale values when dirty-state toggles are incomplete.
- Action: centralize sync policy per tab with explicit apply/cancel model.

## 5. Agent-flagged findings requiring targeted confirmation

These were reported by deep subagents and should be validated with focused reproduction before direct fix:

- Potential ESP correction logic mismatch under oversteer/understeer branch behavior.
- Potential AD tab feedback gaps for ACC/LKA action acknowledgments.
- Potential Leak Lab validation gaps for manual/custom parameter invariants.
- Potential performance hot spots in trace/event rendering under heavy data volume.

## 6. Module-by-module status

Core simulation and vehicle dynamics:

- `src/lib.rs`: PASS runtime checks; integration-heavy, medium complexity risk.
- `src/sim_core.rs`: PASS tests.
- `src/engine.rs`: no critical flags in this audit.
- `src/transmission.rs`: no critical flags in this audit.
- `src/ecu_ecm.rs`: stable under tests; monitor calibration realism.
- `src/ecu_tcm.rs`: recently revised; speed regression tests pass.
- `src/ecu_abs.rs`: no confirmed critical defect; add scenario tests for ESP edge cases.
- `src/ecu_vcm.rs`: no critical flags in this run.
- `src/chassis.rs`: no confirmed critical defect in this run.
- `src/implement.rs`: integration stable in tests.
- `src/leak_physics.rs`: broad coverage and integration tests pass.

Autonomy and perception:

- `src/autonomous.rs`: functional path active; medium risk in ACC integral handling.
- `src/adas.rs`: no critical flags in this run.
- `src/sensors.rs`: no critical flags in this run.
- `src/radar.rs`: no critical flags in this run.
- `src/camera.rs`: no critical flags in this run.
- `src/lidar.rs`: no critical flags in this run.
- `src/gps.rs`: no critical flags in this run.
- `src/imu.rs`: no critical flags in this run.

Network, protocol, diagnostics, storage:

- `src/j1939.rs`: tests pass.
- `src/can_bus.rs`: no critical flags in this run.
- `src/can_gateway.rs`: no critical flags in this run.
- `src/can_network.rs`: tests pass.
- `src/network_mgmt.rs`: no critical flags in this run.
- `src/uds.rs`: prior hard fix present; tests pass.
- `src/io/*`: HIGH priority hardening in live runner path.
- `src/nvm.rs`: medium hardening opportunity for wear-limit enforcement policy.
- `src/observability.rs`: medium integration opportunity to wire structured health events.

UI/GUI/UX:

- `src/main.rs`: functionally broad and complex; medium maintainability/performance risks.
- `src/widgets.rs`: no critical flags in this run.

## 7. API and integration audit conclusions

- Physical simulation API surface is coherent and test-backed.
- Live adapter path is the highest production risk area.
- UDS and CAN protocol paths are generally robust after recent fixes, but need deeper contract/fault tests for production confidence.

## 8. Recommended remediation roadmap

Phase 1 (urgent, 1-2 days):

- Harden `connect_ecm` response validation in `src/io/live_runner.rs`.
- Add live feed health propagation (thread status, last error, stale window).
- Add focused tests for live connect failure modes.

Phase 2 (short, 2-4 days):

- Add ACC integral reset/decay policy tests in `src/autonomous.rs`.
- Add ESP branch behavior scenario tests in `src/ecu_abs.rs`.
- Clean top clippy warnings in `src/main.rs`.

Phase 3 (stabilization, 1 week):

- UI performance pass for heavy lists/trace panels.
- Structured observability wiring across live I/O and protocol failure points.
- Add contract/fault injection test matrix for `src/io/*` and `src/uds.rs`.

## 9. Final audit verdict

- Current state: functionally healthy and test-stable.
- Production risk concentration: live hardware integration path (`src/io/live_runner.rs`) and large-state UI operability in `src/main.rs`.
- Confidence level: high for compile/test stability, medium for field integration robustness without the hardening actions above.