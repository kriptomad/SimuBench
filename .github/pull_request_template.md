## Problem
Describe the issue and impact.

## Changes
- [ ] Refactor/API-preserve
- [ ] Tests
- [ ] Docs/Changelog
- [ ] Observability

## How to test
```bash
cargo fmt -- --check
cargo clippy -- -D warnings
cargo test
```

## Artifacts
- Bench logs:
- Replay/Monte Carlo:

## Acceptance checklist
- [ ] Changelog entry included
- [ ] Unit tests for changed logic
- [ ] Integration tests where applicable
- [ ] No public API breakage (or compatibility shim included)
- [ ] Performance impact documented
- [ ] Rollback path documented (feature flag or revert)
