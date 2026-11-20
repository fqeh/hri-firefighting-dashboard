#!/usr/bin/env bash
# Run libsdformat itself, not merely an XML parser.
set -euo pipefail
if [[ -n "${1:-}" ]]; then
  asset_root="$1"
else
  asset_root="$(ros2 pkg prefix --share hri_sim)"
fi
export SDF_PATH="$asset_root/models:${SDF_PATH:-}"
for file in "$asset_root"/worlds/*.sdf "$asset_root"/models/*/model.sdf; do
  echo "Validating $file"
  gz sdf -k "$file"
done
