# Changelog

## [Unreleased]
### Added
- CI workflows for lint/unit/nightly Monte Carlo/fuzz smoke.
- Observability module with structured JSON events and simulation metrics.
- Incremental solver architecture module (`sim_core`) with integrator traits.
- Property-based tests for physical bounds and leak invariants.
- Monte Carlo, replay, and integration scripts under `scripts/`.
- Fuzz harness scaffold for J1939/CAN frame parser.
- Benchmark harness for simulation steps (`benches/solver_bench.rs`).
- Agent runbook and PR checklist template.

### Changed
- CAN network now supports health scoring, per-bus injections, and snapshot exports.
- J1939 registry expanded with ADAS/fusion/energy PGNs and builders.
