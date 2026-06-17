# Current FAST-Calib ROS2 Setup

This checkout uses the ROS2 fork because this machine has ROS2 Humble and no ROS1 runtime on the host.

## Source

- Repository: `https://github.com/ichangjian/FAST-Calib-ROS2`
- Commit: `707df94bf7a4acdcb806affed7a9e41ee179518c`
- Local path: `/home/vision/FAST-Calib`

## Host ROS Environment

- ROS distribution: `humble`
- Build command:

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Current Parameters

Default config: `/home/vision/FAST-Calib/config/qr_params.yaml`

Configured from `/home/vision/moving_scaning_hku`:

- LiDAR topic: `/livox/lidar`
- Camera topic in source project notes: `/left_camera/image`
- Camera intrinsics source: `/home/vision/moving_scaning_hku/experiments/extrinsic_calibration/fast_calib_camera_intrinsics_from_hik.yaml`
- Target metadata source: `/home/vision/moving_scaning_hku/experiments/extrinsic_calibration/target_measurement_phase1_46_provisional.yaml`
- ArUco dictionary: `DICT_4X4_50`
- ArUco board order for FAST-Calib: `[0, 1, 3, 2]` for top-left, top-right, bottom-right, bottom-left
- Circle center spacing semantics: `delta_width_circles=0.500` and `delta_height_circles=0.400` are full C1-C2/C1-C4 spacings. The FAST-Calib camera-side source divides these values by 2 internally to place circle centers at `±0.250 m` and `±0.200 m`.
- Livox MID360 config: `/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json`

The target metadata is provisional diagnostic metadata. Do not write FAST-Calib output into FAST-LIVO2 production config until the acceptance checks in `/home/vision/moving_scaning_hku/experiments/extrinsic_calibration/result_review_checklist.md` pass.

## Expected Input Pair

The ROS2 fork reads one ROS2 bag directory containing `sensor_msgs/msg/PointCloud2` and one still image:

```text
/home/vision/FAST-Calib/calib_data/current_static_pair/
  metadata.yaml
  *.db3
  image.png
```

Default output path:

```text
/home/vision/FAST-Calib/output/current_static_pair/
```

## Run

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
source install/setup.bash
ros2 launch fast_calib calib.launch.py rviz:=false
```

If `/opt/MVS/lib` is present in `LD_LIBRARY_PATH`, filter it when running FAST-Calib because MVS ships an older `libusb` that can break PCL:

```bash
CLEAN_LD=$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
env LD_LIBRARY_PATH="$CLEAN_LD" ros2 launch fast_calib calib.launch.py rviz:=false
```

For a copied scene-specific config:

```bash
ros2 launch fast_calib calib.launch.py \
  rviz:=false \
  params_file:=/absolute/path/to/scene_qr_params.yaml
```

The helper below runs FAST-Calib with `/opt/MVS/lib` removed from `LD_LIBRARY_PATH`, then runs the offline board-plane hole diagnostic if `aligned_cloud.ply` exists:

```bash
/home/vision/FAST-Calib/scripts/run_fast_calib_scene.sh \
  /home/vision/FAST-Calib/config/qr_params_current_static_pair_long.yaml
```

## Current Runtime Status

As of the latest check on 2026-06-12, no active ROS2 device nodes were left running. `ros2 topic list -t` only returned `/parameter_events` and `/rosout`.

Start the MID360 in the `PointCloud2` format expected by this fork:

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
source install/setup.bash
ros2 launch fast_calib mid360_pointcloud2_launch.py
```

This launch uses `/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json`. The copy keeps the same MID360 network settings from `/home/vision/moving_scaning_hku/ros2_livox_ws/src/livox_ros_driver2/config/MID360_config.json`, but stores `extrinsic_parameter.x/y/z` as integers because this Livox driver reads them with `GetInt()`.

In another terminal, verify the topic type:

```bash
ros2 topic list -t | grep /livox/lidar
```

Expected:

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

Use ROS2 recording for the LiDAR input expected by FAST-Calib:

```bash
ros2 bag record /livox/lidar \
  -o /home/vision/FAST-Calib/calib_data/current_static_pair/lidar_bag
```

Export or place the corresponding debayered still image at:

```text
/home/vision/FAST-Calib/calib_data/current_static_pair/image.png
```

The installed helper can grab one Hikvision PNG directly without a ROS camera node:

```bash
ros2 run fast_calib grab_hikvision_png \
  /home/vision/FAST-Calib/calib_data/current_static_pair/image.png \
  DA3217436 \
  30000 \
  15
```

After physically repositioning the target, the capture/run path can be reduced to one command. Use a new scene name so prior diagnostic data is not overwritten:

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
source install/setup.bash

scripts/capture_and_run_scene.sh current_static_pair_after_reposition 25
```

The installed equivalent is:

```bash
ros2 run fast_calib capture_and_run_scene.sh current_static_pair_after_reposition 25
```

This script:

- starts the MID360 `PointCloud2` driver if `/livox/lidar` is not already available,
- captures `/home/vision/FAST-Calib/calib_data/<scene>/image.png`,
- records `/home/vision/FAST-Calib/calib_data/<scene>/lidar_bag`,
- writes `/home/vision/FAST-Calib/config/qr_params_<scene>.yaml`,
- runs FAST-Calib and the board-plane diagnostic,
- returns failure if `calib_result.txt` is missing.

## Current Live Test

On 2026-06-09 a live diagnostic pair was captured:

- Image: `/home/vision/FAST-Calib/calib_data/current_static_pair/image.png`
- LiDAR bag: `/home/vision/FAST-Calib/calib_data/current_static_pair/lidar_bag`
- Bag content: 79 messages on `/livox/lidar`, type `sensor_msgs/msg/PointCloud2`, duration about 7.8 s.
- Image size: 1440 x 1080.

FAST-Calib successfully loaded the image and ROS2 bag, detected 4 ArUco/camera centers, and segmented the target board plane after tightening the ROI to:

```yaml
x_min: 1.90
x_max: 3.90
y_min: -1.90
y_max: -1.65
z_min: -0.40
z_max: 1.35
```

No accepted extrinsic was produced from this pair. The LiDAR side did not yield 4 reliable circular hole centers; the run exits with:

```text
Expected 4 LiDAR circle centers
```

This is a diagnostic capture/run, not a calibration result.

A second longer pair was captured to reduce MID360 sparsity:

- Image: `/home/vision/FAST-Calib/calib_data/current_static_pair_long/image.png`
- LiDAR bag: `/home/vision/FAST-Calib/calib_data/current_static_pair_long/lidar_bag`
- Bag content: 359 messages on `/livox/lidar`, type `sensor_msgs/msg/PointCloud2`, duration about 35.8 s.
- Scene config: `/home/vision/FAST-Calib/config/qr_params_current_static_pair_long.yaml`

Run command:

```bash
CLEAN_LD=$(printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
env LD_LIBRARY_PATH="$CLEAN_LD" ros2 launch fast_calib calib.launch.py \
  rviz:=false \
  params_file:=/home/vision/FAST-Calib/config/qr_params_current_static_pair_long.yaml
```

The long pair also detects the 4 ArUco/camera centers and segments the board plane, but it still does not produce a valid extrinsic. The LiDAR hole extraction found only partial candidates instead of 4 circular centers. Treat every file in `/home/vision/FAST-Calib/output/current_static_pair*` as diagnostic output only.

The offline board-plane diagnostic confirms this:

```bash
python3 /home/vision/FAST-Calib/scripts/diagnose_board_plane.py \
  --cloud /home/vision/FAST-Calib/output/current_static_pair_long/aligned_cloud.ply \
  --config /home/vision/FAST-Calib/config/qr_params_current_static_pair_long.yaml \
  --out /home/vision/FAST-Calib/output/current_static_pair_long/diagnostics
```

Current long-pair result:

```text
candidate_empty_regions=6
radius_filtered_candidates=0
verdict=fail
```

Diagnostic outputs:

```text
/home/vision/FAST-Calib/output/current_static_pair_long/diagnostics/board_plane_hole_diagnostic.txt
/home/vision/FAST-Calib/output/current_static_pair_long/diagnostics/board_plane_hole_diagnostic.png
```

On 2026-06-12 another live pair was captured after restarting the MID360 driver:

- Image: `/home/vision/FAST-Calib/calib_data/current_static_pair_20260612/image.png`
- LiDAR bag: `/home/vision/FAST-Calib/calib_data/current_static_pair_20260612/lidar_bag`
- Bag content: 249 messages on `/livox/lidar`, type `sensor_msgs/msg/PointCloud2`, duration about 24.8 s.
- Scene config: `/home/vision/FAST-Calib/config/qr_params_current_static_pair_20260612.yaml`

## MID360 Network Change

On 2026-06-17 the MID360 static IP was changed from `192.168.1.3` to `192.168.1.30`.

Reason: `wlo1` uses `192.168.1.3/24` for the user's remote desktop/Wi-Fi path. Keeping the sensor on the same address conflicts with the host and can break RDP.

The change was made through an isolated Linux network namespace and macvlan on `enp2s0`, using `/home/vision/FAST-Calib/tools/set_livox_ip.cpp`, so the host Wi-Fi address and default route were not modified. The runtime Livox driver config now uses:

```text
/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json
  host cmd/point/imu IP: 192.168.1.50
  lidar IP:             192.168.1.30
```

Verification after reboot:

```text
ping 192.168.1.30: success
ros2 launch fast_calib mid360_pointcloud2_launch.py: publishes /livox/lidar as sensor_msgs/msg/PointCloud2
```

Do not remove `192.168.1.3/24` from `wlo1`; that address belongs to the user's Wi-Fi/RDP environment.

## Black Background Capture, 2026-06-17

After adding the black board behind the calibration target, a new live pair was captured:

- Scene name: `current_static_pair_blackboard_20260617_r2`
- Image: `/home/vision/FAST-Calib/calib_data/current_static_pair_blackboard_20260617_r2/image.png`
- LiDAR bag: `/home/vision/FAST-Calib/calib_data/current_static_pair_blackboard_20260617_r2/lidar_bag`
- Bag content: 249 messages on `/livox/lidar`, type `sensor_msgs/msg/PointCloud2`, duration about 24.8 s.
- Scene config: `/home/vision/FAST-Calib/config/qr_params_current_static_pair_blackboard_20260617_r2.yaml`
- Output: `/home/vision/FAST-Calib/output/current_static_pair_blackboard_20260617_r2`

FAST-Calib detected the 4 camera-side ArUco/circle centers and produced a `calib_result.txt`, but this result is rejected:

```text
RMSE: 0.2319 m
Pcl norm: about 1.52 m
diagnostic verdict: fail
candidate_empty_regions: 0
radius_filtered_candidates: 0
```

The rejected result is:

```text
Rcl: [ -0.122357,  -0.935215,  -0.332267,
        0.129012,   0.316955,  -0.939625,
        0.984066,  -0.157836,   0.081872]
Pcl: [ -1.309119,   0.339494,   0.697328]
```

Do not copy this result into FAST-LIVO2 production config. The black background improved the image-side visibility, but the LiDAR-side board-plane diagnostic still did not find four radius-compatible empty circular regions with the expected rectangle geometry.

After turning off the lights, another pair was captured:

- Scene name: `current_static_pair_blackboard_lights_off_20260617`
- Image: `/home/vision/FAST-Calib/calib_data/current_static_pair_blackboard_lights_off_20260617/image.png`
- LiDAR bag: `/home/vision/FAST-Calib/calib_data/current_static_pair_blackboard_lights_off_20260617/lidar_bag`
- Bag content: 248 messages on `/livox/lidar`, type `sensor_msgs/msg/PointCloud2`, duration about 24.7 s.
- Scene config: `/home/vision/FAST-Calib/config/qr_params_current_static_pair_blackboard_lights_off_20260617.yaml`
- Output: `/home/vision/FAST-Calib/output/current_static_pair_blackboard_lights_off_20260617`

This run did not produce `calib_result.txt`. It was an improvement over the lights-on black-background run on the LiDAR diagnostic, but still failed:

```text
edge clusters: 4
candidate_empty_regions: 3
radius_filtered_candidates: 1
verdict: fail
reason: no four radius-compatible empty regions matched the target rectangle geometry
```

FAST-Calib rejected the LiDAR side with:

```text
Expected 4 LiDAR circle centers, got 0
```

After verifying the MID360 live axes in RViz, the target board was confirmed to be in positive `X`, centered near `Y=0`, with positive `Z` upward. The earlier diagnostic ROI using `y_min=-1.90` and `y_max=-1.65` was therefore looking at the wrong lateral region.

The default ROI in `/home/vision/FAST-Calib/config/qr_params.yaml` was updated to:

```yaml
x_min: 2.20
x_max: 4.40
y_min: -0.80
y_max: 0.80
z_min: -0.60
z_max: 1.50
```

Offline reruns on `/home/vision/FAST-Calib/calib_data/current_static_pair_adjusted_20260617/lidar_bag` with `Y` centered around zero improved the LiDAR-side interpretation. FAST-Calib consistently fitted two real target-hole circles:

```text
candidate_center_origin 3.57492  0.07246  0.11297, fitted_radius 0.109236
candidate_center_origin 3.55353 -0.43381  0.13000, fitted_radius 0.114494
```

That confirms the corrected axis/ROI is hitting the board body. It still does not produce calibration because only two of the four holes are detected in the current bag.

A full recapture with the corrected `Y≈0` default ROI was run as:

- Scene name: `current_static_pair_y0_full_20260617`
- Image: `/home/vision/FAST-Calib/calib_data/current_static_pair_y0_full_20260617/image.png`
- LiDAR bag: `/home/vision/FAST-Calib/calib_data/current_static_pair_y0_full_20260617/lidar_bag`
- Bag content: 249 messages on `/livox/lidar`, type `sensor_msgs/msg/PointCloud2`, duration about 24.8 s.
- Scene config: `/home/vision/FAST-Calib/config/qr_params_current_static_pair_y0_full_20260617.yaml`
- Output: `/home/vision/FAST-Calib/output/current_static_pair_y0_full_20260617`

This full recapture again fitted only two target-hole circles:

```text
candidate_center_origin 3.57454  0.07096 0.11873, fitted_radius 0.110922
candidate_center_origin 3.55390 -0.43180 0.14098, fitted_radius 0.114809
Expected 4 LiDAR circle centers, got 0
```

No `calib_result.txt` was produced.
- Output path: `/home/vision/FAST-Calib/output/current_static_pair_20260612`

Run command:

```bash
/home/vision/FAST-Calib/scripts/run_fast_calib_scene.sh \
  /home/vision/FAST-Calib/config/qr_params_current_static_pair_20260612.yaml
```

FAST-Calib loaded the new image and bag and detected 4 ArUco/camera centers, but the LiDAR side still failed:

```text
Plane cloud size: 2603
Extracted 2145 edge points.
Number of edge clusters: 1
fitted_radius 0.213331 expected_radius 0.12
Expected 4 LiDAR circle centers, got 0
```

The offline board-plane diagnostic for the same run also failed:

```text
candidate_empty_regions=1
radius_filtered_candidates=1
verdict=fail
calibration result missing: /home/vision/FAST-Calib/output/current_static_pair_20260612/calib_result.txt
```

The diagnostic was rechecked with the same full-spacing semantics as FAST-Calib source (`0.500 m x 0.400 m` between hole centers), so this is not a half-spacing/full-spacing mismatch.

No valid extrinsic has been produced for the 2026-06-12 capture.

On 2026-06-17 the target was rechecked with a black backing board behind the circular holes:

- Image captured successfully: `/home/vision/FAST-Calib/calib_data/current_static_pair_blackboard_20260617/image.png`
- The camera view shows the four holes mostly covered by the black backing board, which is an improvement over the 2026-06-12 scene.
- A full LiDAR capture could not start because the host Wi-Fi interface `wlo1` currently owns `192.168.1.3/24`, while `/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json` also configures the MID360 LiDAR IP as `192.168.1.3`.
- Evidence:

```text
ip route get 192.168.1.3
local 192.168.1.3 dev lo table local src 192.168.1.3
```

The capture script now fails fast with status `78` when this IP conflict is present. Release the conflicting local address before rerunning the full capture:

```bash
sudo ip addr del 192.168.1.3/24 dev wlo1
```

Then rerun with a fresh scene name, for example:

```bash
ros2 run fast_calib capture_and_run_scene.sh current_static_pair_blackboard_20260617_r2 25
```

## Physical Scene Requirement

The remaining blocker is the physical capture geometry, not ROS2 transport or file loading. In the current camera view, the circular holes show nearby background such as wall/floor/rails through the openings. The LiDAR therefore receives returns through the holes instead of seeing clean empty gaps, which breaks FAST-Calib's circle-hole extraction.

Before recapturing:

- Move the target so all four circular holes look into open space with no close reflective surface behind them.
- Keep floor, rails, wall edges, and people out of the hole openings.
- If open space is not possible, place a dark absorptive backing far enough behind the target that it does not form strong returns near the board plane.
- Keep the target centered in both the MID360 and camera fields of view, preferably closer than the current sparse setup while still fully visible.
- Record a static bag after the target and both sensors are stationary.

After repositioning, recapture the image and bag, then rerun the same scene config or copy it to a new scene-specific YAML with updated paths.

The preferred recapture command after repositioning is:

```bash
/home/vision/FAST-Calib/scripts/capture_and_run_scene.sh current_static_pair_after_reposition 25
```
