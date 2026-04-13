#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vt-franka-workspace teleop --config "$ROOT_DIR/robot_workspace/config/workspace.yaml"

