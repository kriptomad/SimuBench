#!/usr/bin/env bash
set -euo pipefail

seeds=${1:-5}
model=${2:-reduced}
out_dir=${3:-artifacts/monte}
mkdir -p "$out_dir"

cargo build --release --bin simulator_cli

for i in $(seq 1 "$seeds"); do
  seed=$(( (RANDOM<<16) ^ i ))
  log_file="$out_dir/monte_${model}_${seed}.log"
  ./target/release/simulator_cli --seed "$seed" --model "$model" --steps 10000 2>&1 | tee "$log_file" || true
done

echo "Monte Carlo run complete: $out_dir"
