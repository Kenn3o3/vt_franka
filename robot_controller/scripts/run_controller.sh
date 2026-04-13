#!/usr/bin/env bash
set -euo pipefail

CONFIG_PATH="${1:-$(dirname "$0")/../config/controller.yaml}"
vt-franka-controller run --config "$CONFIG_PATH"

