#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
build_dir="${1:-build-local}"
if [[ -f /opt/intel/openvino_2024.6.0/setupvars.sh ]]; then
  set +u  # Intel's environment script uses optional unset variables.
  source /opt/intel/openvino_2024.6.0/setupvars.sh
  set -u
fi
cmake -S . -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DFPA_BUILD_HARDWARE=ON -DBUILD_TESTING=ON
cmake --build "$build_dir" -j2
ctest --test-dir "$build_dir" --output-on-failure
printf 'Build/tests finished. No live camera, gimbal or firing was started.\n'
