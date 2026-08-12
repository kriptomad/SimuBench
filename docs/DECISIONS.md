# Architecture Decisions

## ADR-001: Incremental refactor with API preservation
- We introduce internal traits and helper modules before changing public APIs.
- Reason: reduce regression risk and keep UI and integration tests stable.

## ADR-002: Feature flags for high-risk changes
- Flags: `advanced_observability`, `new_integrator`.
- Reason: safe rollout/rollback without large reverts.

## ADR-003: Test matrix-first workflow
- Every refactor commit must include unit or integration coverage.
- Nightly Monte Carlo and fuzz smoke run in CI to catch long-tail failures.

## ADR-004: Structured telemetry baseline
- Logs use JSON event format with correlation IDs.
- Metrics baseline includes step duration, steps completed, errors, replay failures.
