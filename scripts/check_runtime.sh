#!/usr/bin/env bash
# Read-only dependency/help checks. Never install packages or open hardware.
set -euo pipefail
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
binary_dir="${1:-bin}"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  printf 'This prebuilt package requires Linux x86_64. Build from source on this platform.\n' >&2
  exit 1
fi
if [[ -f /opt/intel/openvino_2024.6.0/setupvars.sh ]]; then
  set +u  # Intel's environment script uses optional unset variables.
  source /opt/intel/openvino_2024.6.0/setupvars.sh
  set -u
fi
for task in 1 2 3; do
  binary="$binary_dir/task_$task"
  [[ "$binary" = /* ]] || binary="$project_root/$binary"
  if [[ ! -x "$binary" ]]; then
    printf 'Missing/executable bit unset: %s. Extract the .tar.gz on Linux, or rebuild.\n' "$binary" >&2
    exit 1
  fi
  if ! dependencies="$(ldd "$binary" 2>&1)"; then
    printf '%s\n' "$dependencies" >&2
    exit 1
  fi
  case "$dependencies" in
    *'not found'*) printf 'INCOMPATIBLE dependencies for %s:\n%s\nDo not replace robot libraries; rebuild in its existing environment.\n' "$binary" "$dependencies" >&2; exit 1 ;;
  esac
  "$binary" --help >/dev/null
  printf 'PASS: task_%s shared libraries and help entry.\n' "$task"
done
printf 'This check does NOT validate GPU plugins, camera access, calibration, serial protocol or real tracking.\n'
