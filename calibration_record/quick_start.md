# 下次重新标定快速流程

## 1. 准备环境

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source /home/vision/moving_scaning_hku/ros2_livox_ws/install/setup.bash
source install/setup.bash
```

如果运行 FAST-Calib 时遇到 `libusb_set_option` 之类错误，先清理 MVS SDK 带来的旧 libusb：

```bash
CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
```

## 2. 启动 MID360

```bash
ros2 launch fast_calib mid360_pointcloud2_launch.py
```

确认：

```bash
ros2 topic list -t | grep /livox/lidar
```

期望：

```text
/livox/lidar [sensor_msgs/msg/PointCloud2]
```

## 3. 采集一个新场景

推荐每次换一个 scene 名，避免覆盖旧结果：

```bash
cd /home/vision/FAST-Calib
scripts/capture_and_run_scene.sh <scene_name> 25
```

例如：

```bash
scripts/capture_and_run_scene.sh current_static_pair_new 25
```

脚本会完成：

- 抓一张 Hikvision 图像；
- 录制 `/livox/lidar` bag；
- 生成 `config/qr_params_<scene_name>.yaml`；
- 运行 FAST-Calib；
- 运行离线诊断。

## 4. 如果自动 LiDAR 检测失败

常见现象：

```text
Expected 4 LiDAR circle centers, got 2
```

这次最终就是自动检测只抓到上排两个孔。处理方法是：

1. 用累计静态点云显示，不播放实时 bag。
2. 在 RViz 里人工确认四个孔。
3. 用 `manual_lidar_centers_calib` 手工四孔版本出外参。

现在推荐直接使用可视化交互流程：

```bash
scripts/interactive_calibration_workflow.sh <scene_name> 25
```

它会自动采集、生成静态点云、打开 RViz 四球编辑器，并在保存球心后继续出外参。详细说明见 `calibration_record/interactive_workflow.md`。

启动静态点云 + 孔位显示：

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
python3 scripts/publish_static_cloud_with_holes.py \
  --cloud output/final_success_20260617/filtered_cloud.ply \
  --rate 0.2
```

另一个终端打开 RViz：

```bash
source /opt/ros/humble/setup.bash
rviz2 -d /home/vision/FAST-Calib/rviz_cfg/static_accumulated_holes.rviz
```

本次确认的四个 LiDAR 孔中心：

```text
upper +Y: x=3.57454, y= 0.07096, z= 0.11873
upper -Y: x=3.55390, y=-0.43180, z= 0.14098
lower +Y: x=3.54801, y= 0.06100, z=-0.27500
lower -Y: x=3.52758, y=-0.45020, z=-0.25500
```

## 5. 用四个手工孔位出外参

手工四孔工具已经加入工程：

```text
tools/manual_lidar_centers_calib.cpp
```

编译：

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_calib --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

运行示例：

```bash
mkdir -p output/<scene_name>_manual_four_holes

CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)

env LD_LIBRARY_PATH="$CLEAN_LD" ros2 run fast_calib manual_lidar_centers_calib \
  --ros-args \
  --params-file config/qr_params_<scene_name>.yaml \
  -p output_path:=/home/vision/FAST-Calib/output/<scene_name>_manual_four_holes
```

输出文件：

```text
output/<scene_name>_manual_four_holes/calib_result.txt
output/<scene_name>_manual_four_holes/colored_cloud.pcd
output/<scene_name>_manual_four_holes/colored_cloud.ply
output/<scene_name>_manual_four_holes/qr_detect.png
```

## 6. 验收标准

至少检查：

- `calib_result.txt` 存在；
- 四孔 SVD RMSE 尽量在厘米以内，本次是 `0.004935 m`；
- `colored_cloud.pcd` / `colored_cloud.ply` 能生成；
- RViz 中四个孔位球和点云空洞对应；
- 不要使用明显异常结果，例如平移 Z 达到数米的结果。
