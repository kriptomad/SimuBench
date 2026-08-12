#!/usr/bin/env bash
set -euo pipefail

artifact=${1:-}
if [[ -z "$artifact" ]]; then
  echo "usage: $0 <artifact_log>"
  exit 1
fi

cargo build --release --bin simulator_cli
./target/release/simulator_cli --replay "$artifact" --verify
