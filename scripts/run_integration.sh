#!/usr/bin/env bash
set -euo pipefail

cargo test --test leak_system_integration
