#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: interactive_calibration_workflow.sh SCENE_NAME [DURATION_S]

Capture one image + MID360 bag, generate a static accumulated cloud, open RViz
with four draggable LiDAR hole spheres, save the edited sphere centers, and run
manual-center FAST-Calib.

After RViz opens:
  1. Set the RViz tool to Interact.
  2. Drag the four colored spheres onto the four calibration-board holes.
  3. In another terminal, run:
       ros2 service call /save_lidar_hole_markers std_srvs/srv/Trigger {}
  4. Return to this terminal and press Enter.
EOF
}

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 64
fi

scene_name=$1
duration_s=${2:-25}
fast_calib_root=${FAST_CALIB_ROOT:-/home/vision/FAST-Calib}
cd "$fast_calib_root"

data_dir="/home/vision/FAST-Calib/calib_data/${scene_name}"
output_dir="/home/vision/FAST-Calib/output/${scene_name}"
config_path="/home/vision/FAST-Calib/config/qr_params_${scene_name}.yaml"
static_cloud="${output_dir}/filtered_cloud.ply"
centers_file="${output_dir}/manual_lidar_holes.yaml"
rviz_file="${output_dir}/manual_lidar_hole_editor.rviz"
manual_output_dir="/home/vision/FAST-Calib/output/${scene_name}_manual_four_holes"

set +u
source /opt/ros/humble/setup.bash
if [[ -f /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash ]]; then
  source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
fi
source install/setup.bash
set -u

echo "Capturing scene: ${scene_name}"
set +e
scripts/capture_and_run_scene.sh "$scene_name" "$duration_s"
capture_status=$?
set -e

if [[ ! -f "$config_path" ]]; then
  echo "Scene config was not generated: $config_path" >&2
  exit "$capture_status"
fi

if [[ ! -f "$static_cloud" ]]; then
  cat >&2 <<EOF
Static cloud was not generated: $static_cloud

This usually means the camera/ArUco target was not detected, or FAST-Calib
failed before LiDAR board extraction. Check:
  $output_dir
EOF
  exit "$capture_status"
fi

mkdir -p "$manual_output_dir"

cat >"$rviz_file" <<EOF
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.35
      Cell Size: 0.5
      Class: rviz_default_plugins/Grid
      Color: 90; 90; 90
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: XY Grid
      Plane: XY
      Plane Cell Count: 24
      Reference Frame: livox_frame
      Value: true
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 1.0
      Name: XYZ Axes
      Radius: 0.03
      Reference Frame: livox_frame
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 210; 210; 210
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Name: Static Accumulated Cloud
      Position Transformer: XYZ
      Queue Size: 1
      Selectable: true
      Size (Pixels): 2
      Size (m): 0.01
      Style: Points
      Topic:
        Value: /static_accumulated_cloud
      Use Fixed Frame: true
      Value: true
    - Class: rviz_default_plugins/InteractiveMarkers
      Enabled: true
      Name: Draggable Hole Spheres
      Show Axes: true
      Show Descriptions: true
      Update Topic: /manual_lidar_holes/update
      Value: true
  Enabled: true
  Global Options:
    Background Color: 18; 18; 18
    Fixed Frame: livox_frame
    Frame Rate: 10
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.4
      Focal Point:
        X: 3.56
        Y: -0.18
        Z: 0.1
      Name: Current View
      Pitch: 0.28
      Target Frame: livox_frame
      Yaw: 2.7
    Saved: ~
Window Geometry:
  Height: 900
  Width: 1400
EOF

editor_pid=""
rviz_pid=""
cleanup() {
  if [[ -n "$rviz_pid" ]] && kill -0 "$rviz_pid" 2>/dev/null; then
    kill "$rviz_pid" 2>/dev/null || true
  fi
  if [[ -n "$editor_pid" ]] && kill -0 "$editor_pid" 2>/dev/null; then
    kill "$editor_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "Starting interactive LiDAR hole editor..."
python3 scripts/interactive_lidar_hole_editor.py \
  --cloud "$static_cloud" \
  --output "$centers_file" \
  --initial-centers "$centers_file" \
  --rate 0.2 >"${output_dir}/interactive_lidar_hole_editor.log" 2>&1 &
editor_pid=$!

sleep 1

echo "Opening RViz: $rviz_file"
rviz2 -d "$rviz_file" >"${output_dir}/rviz2_manual_lidar_holes.log" 2>&1 &
rviz_pid=$!

cat <<EOF

RViz is open.

Move the four colored spheres onto the four physical holes in the static cloud.
When done, save the positions from another terminal:

  cd /home/vision/FAST-Calib
  source /opt/ros/humble/setup.bash
  ros2 service call /save_lidar_hole_markers std_srvs/srv/Trigger {}

Saved centers file:
  $centers_file

EOF

read -r -p "After saving the four spheres, press Enter here to run calibration..."

set +e
ros2 service call /save_lidar_hole_markers std_srvs/srv/Trigger {} >/dev/null 2>&1
set -e

if [[ ! -f "$centers_file" ]]; then
  echo "Manual centers file does not exist: $centers_file" >&2
  exit 74
fi

echo "Running manual-center calibration..."
clean_ld=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
env LD_LIBRARY_PATH="$clean_ld" ros2 run fast_calib manual_lidar_centers_calib \
  --ros-args \
  --params-file "$config_path" \
  -p manual_lidar_centers_path:="$centers_file" \
  -p output_path:="$manual_output_dir"

echo
echo "Manual calibration output:"
echo "  $manual_output_dir"
echo "Result:"
echo "  $manual_output_dir/calib_result.txt"
