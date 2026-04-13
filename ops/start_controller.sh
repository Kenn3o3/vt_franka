#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vt-franka-controller run --config "$ROOT_DIR/robot_controller/config/controller.yaml"

