#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "usage: $0 CONFIG_YAML [OUTPUT_DIR]" >&2
  exit 64
fi

config_yaml=$1
output_dir=${2:-}

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
fast_calib_root=${FAST_CALIB_ROOT:-$(cd "${script_dir}/.." && pwd)}
cd "$fast_calib_root"

if [[ -z "$output_dir" ]]; then
  output_dir=$(python3 - "$config_yaml" <<'PY'
from pathlib import Path
import sys
for raw_line in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines():
    line = raw_line.split("#", 1)[0].strip()
    if line.startswith("output_path:"):
        print(line.split(":", 1)[1].strip().strip('"'))
        break
PY
)
fi

set +u
source /opt/ros/humble/setup.bash
workspace_setup=${ROS_WORKSPACE_SETUP:-}
if [[ -n "$workspace_setup" ]]; then
  source "$workspace_setup"
elif [[ -f "${fast_calib_root}/install/setup.bash" ]]; then
  source "${fast_calib_root}/install/setup.bash"
elif [[ -f "${fast_calib_root}/../../install/setup.bash" ]]; then
  source "${fast_calib_root}/../../install/setup.bash"
else
  echo "Cannot find workspace setup.bash. Build the package first or set ROS_WORKSPACE_SETUP." >&2
  exit 69
fi
set -u

clean_ld=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
set +e
env LD_LIBRARY_PATH="$clean_ld" ros2 launch fast_calib calib.launch.py \
  rviz:=false \
  params_file:="$(realpath "$config_yaml")" &
launch_pid=$!
fast_calib_status=0
for _ in $(seq 1 120); do
  if [[ -n "$output_dir" && -f "$output_dir/calib_result.txt" ]]; then
    kill -INT "$launch_pid" 2>/dev/null || true
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid"
    fast_calib_status=$?
    launch_pid=""
    break
  fi
  sleep 1
done
if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$launch_pid"
  fast_calib_status=$?
fi
set -e

if [[ -n "$output_dir" && -f "$output_dir/calib_result.txt" && "$fast_calib_status" -eq 130 ]]; then
  fast_calib_status=0
fi

if [[ -n "$output_dir" && -f "$output_dir/aligned_cloud.ply" ]]; then
  set +e
  python3 scripts/diagnose_board_plane.py \
    --cloud "$output_dir/aligned_cloud.ply" \
    --config "$config_yaml" \
    --out "$output_dir/diagnostics"
  diagnostic_status=$?
  set -e
else
  diagnostic_status=2
fi

if [[ ! -f "$output_dir/calib_result.txt" ]]; then
  echo "calibration result missing: $output_dir/calib_result.txt" >&2
  if [[ "$fast_calib_status" -eq 0 ]]; then
    fast_calib_status=3
  fi
fi

if [[ "$diagnostic_status" -ne 0 && "$fast_calib_status" -eq 0 ]]; then
  fast_calib_status="$diagnostic_status"
fi

exit "$fast_calib_status"
