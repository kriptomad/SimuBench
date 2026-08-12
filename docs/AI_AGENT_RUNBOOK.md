# AI Agent Refactor Runbook

## Branch strategy
- Base branch: `feat/auto-refactor-runner`
- Agent branch: `feat/ai-auto-refactor`

## Agent cycle
1. Select 5-10 files max.
2. Ask for 1-3 atomic changes per cycle.
3. Require tests and changelog updates.
4. Validate locally: `cargo fmt -- --check`, `cargo clippy -- -D warnings`, `cargo test`.

## Prompt template
"You are a senior Rust engineer for embedded simulation systems. Analyze up to 10 files and produce:
1) max 5 bullets of findings,
2) unified diff in 1-2 logical commits,
3) tests (unit or property-based),
4) exact verification commands.
Do not break public API without compatibility shim + migration notes."

## PR acceptance checklist
- [ ] Changelog updated
- [ ] Unit tests added/updated
- [ ] Integration tests added/updated when cross-module
- [ ] `cargo fmt -- --check` passes
- [ ] `cargo clippy -- -D warnings` passes
- [ ] `cargo test` passes
- [ ] Benchmarks included if performance-sensitive
- [ ] Feature flag used for risky rollout
