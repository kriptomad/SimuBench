# OSS Integration Matrix for AutoBreaking

## Objective

Turn verified open-source protocol stacks and tools into production-ready integration targets for the current AutoBreaking runtime, with explicit safety, licensing, and implementation gates.

Scope mapped to current modules:

- src/io/live_runner.rs
- src/io/vendor_cat_comm.rs
- src/io/production_program.rs
- src/io/hw.rs

## Selection Rules (hard gates)

1. License must be compatible with commercial/distribution constraints.
2. Stack must expose explicit behavior for UDS 0x27/0x34/0x36/0x37 or ISO-TP flow control semantics.
3. Integration must preserve existing write gates:
   - enable_write + allowlist + noninteractive_approved + !dry_run.
4. No regression on current strict phase runner behavior.

## Candidate Matrix

| Candidate | Type | License | Maturity | Protocol coverage | OS fit | Integration effort | Main risks | Recommendation |
|---|---|---|---|---|---|---|---|---|
| driftregion/iso14229 | C library | MIT | Medium/High | UDS server/client core, 0x27, 0x34, 0x36, 0x37 semantics | Cross-platform (C), transport-dependent | Medium | C FFI boundary, ABI discipline | Adopt as semantic reference and optional conformance harness |
| linux-can/can-utils | Linux CLI/tools + headers | Mixed permissive/GPL dual in many files | High | CAN + ISO-TP operational tooling (isotpsend, isotprecv, isotpdump, candump), J1939 tools | Linux-focused | Low/Medium | Linux-only runtime, GPL contamination if code copied | Use as external tooling/interoperability reference, do not copy source |
| pylessard/python-can-isotp | Python ISO-TP stack | MIT | High | ISO-TP timing, FC blocksize/stmin/wftmax, error handling | Cross-platform Python | Low (reference), High (runtime embed) | Python runtime dependency if embedded | Use as behavioral oracle for test vectors and timing assertions |
| pylessard/python-udsoncan | Python UDS client/services | MIT | High | UDS services and strict response handling, including 0x27/0x34/0x36/0x37 patterns | Cross-platform Python | Low (reference), High (runtime embed) | Python runtime dependency if embedded | Use as protocol behavior reference, not runtime dependency |
| socketcan-rs/socketcan-rs | Rust crate | MIT/Apache-2.0 | Medium/High | SocketCAN read/write abstractions (CAN path) | Linux-focused | Medium | Linux-only adapter path | Candidate for replacing/customizing socketcan adapter internals |
| hardbyte/python-can | Python CAN abstraction | LGPL-3.0 | High | Multi-adapter CAN APIs, filtering models, send/recv patterns | Cross-platform Python | Low (reference), High (runtime embed) | LGPL obligations, Python runtime | Use as adapter design reference only |
| cantools/cantools | Python DBC tooling | MIT | High | DBC decode/encode and message schema workflows | Cross-platform Python | Medium | Python runtime + schema governance | Optional offline validation/engineering tooling only |

## Module-to-OSS Mapping

## src/io/live_runner.rs

Use OSS references to tighten transport and diagnostic semantics:

- ISO-TP behavior parity:
  - fc stmin, blocksize, wftmax handling patterns from python-can-isotp.
  - timeout taxonomy parity for flow control and consecutive frame windows.
- UDS transaction strictness:
  - 0x27 seed/key sequence controls from iso14229 + udsoncan behavior.
  - 0x34/0x36/0x37 state machine and NRC gate rules.
- J1939/CAN observability:
  - can-utils style operational traces as external validation during integration tests.

Targeted outcomes:

1. Deterministic block counter and sequence checks during transfer.
2. Explicit FC timeout and wrong-sequence failure categories in report evidence.
3. Stronger parity between preflight and flash-state transitions.

## src/io/vendor_cat_comm.rs

Use OSS as protocol discipline references for the template bridge:

- Borrow transport option vocabulary from ISO-TP tool ecosystem:
  - stmin, bs, wftmax, padding mode, tx/rx separation semantics.
- Add bridge capability negotiation concept:
  - version and transport capability fields inspired by robust adapter APIs.

Targeted outcomes:

1. Bridge init handshake returns capabilities and limits.
2. Host applies strict compatibility checks before enabling flash path.
3. Better error kind granularity mapping to existing HwError categories.

## src/io/production_program.rs

Use OSS conformance references to harden phase evidence:

- Add conformance checks for UDS service outcomes (0x27/0x34/0x36/0x37).
- Add explicit blocked/failed distinction for transport-timeout vs policy-gate failures.

Targeted outcomes:

1. Per-phase evidence includes protocol reason class.
2. JSON report includes conformance_summary section.
3. Strict mode fails on semantic mismatch, not only transport failure.

## src/io/hw.rs

Use OSS coverage findings to guide adapter capability model:

- Define capability flags:
  - can_raw
  - isotp
  - j1939
  - uds_flash
  - vendor_bridge
- Drive startup policy and phase gates by declared capability profile.

Targeted outcomes:

1. Predictable failures when required capability is absent.
2. Reduced false-start runs in production phases.

## License and Compliance Notes

Do:

1. Treat can-utils and other GPL-affected projects as interoperability references and external tools.
2. Keep source-level implementation in Rust under this repository's own licensing strategy.
3. Keep copied material to zero; only behavior-level alignment and independent implementation.

Do not:

1. Copy/paste implementation code from GPL files into this codebase.
2. Introduce Python runtime as a hard dependency in production path unless approved.

## Priority Adoption Plan

## Wave 1 (immediate, low risk)

1. Add protocol conformance checklist to phase report generation in production_program.
2. Add ISO-TP timing and FC diagnostics fields to live_runner flash summary.
3. Add vendor bridge capability negotiation fields in vendor_cat_comm init/ping flow.

## Wave 2 (short-term)

1. Introduce adapter capability struct in hw and enforce it in phase gates.
2. Add repeatable interoperability scripts using can-utils on Linux benches.
3. Create golden UDS/ISO-TP test vectors from OSS behavior references.

## Wave 3 (optional)

1. Evaluate socketcan-rs deeper integration for Linux adapter internals.
2. Add offline DBC validation workflow using cantools-equivalent process (tooling side).

## Concrete Backlog Items

1. live_runner: add FlashTransportDiagnostics struct:
   - fc_blocksize_seen
   - fc_stmin_seen
   - wait_frame_count
   - sequence_error_count
   - flowcontrol_timeout_count
2. production_program: add conformance_summary in ProgramReport with per-service pass/fail.
3. hw: add capability declaration and gate checks before live flash preflight.
4. vendor_cat_comm: extend bridge protocol with protocol_version and capability list.
5. docs: append runbook section for Linux can-utils validation workflow.

## Verification Gates for First Integration Patch Set

1. cargo check must pass.
2. cargo test --workspace must pass.
3. Existing strict phase CLI run must still produce overall_passed=true in baseline path.
4. New report fields must be present and non-empty when flash path runs.

## External References (evaluated)

- https://github.com/driftregion/iso14229
- https://github.com/linux-can/can-utils
- https://github.com/pylessard/python-can-isotp
- https://github.com/pylessard/python-udsoncan
- https://github.com/socketcan-rs/socketcan-rs
- https://github.com/hardbyte/python-can
- https://github.com/cantools/cantools
