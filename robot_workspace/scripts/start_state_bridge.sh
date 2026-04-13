#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${1:-$(dirname "$0")/../config/workspace.yaml}"
vt-franka-workspace state-bridge --config "$CONFIG_PATH"

