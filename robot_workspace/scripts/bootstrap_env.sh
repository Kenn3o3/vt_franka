#!/usr/bin/env bash
set -euo pipefail

conda env create -f "$(dirname "$0")/../environment.yml" || true
echo "Activate with: conda activate vt-franka-workspace"

