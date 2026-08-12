#!/usr/bin/env bash
set -euo pipefail

if ! command -v cargo-fuzz >/dev/null 2>&1; then
  cargo install cargo-fuzz
fi

# Keep CI runtime bounded.
cargo fuzz run j1939_frame_from_raw -- -max_total_time=120
