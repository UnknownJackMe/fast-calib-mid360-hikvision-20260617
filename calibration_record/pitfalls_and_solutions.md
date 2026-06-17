# 标定踩坑和解决方案

## 1. LiDAR IP 和 RDP/Wi-Fi 冲突

现象：

- 用户通过 RDP 连接电脑，突然连接不稳定或连不上；
- MID360 原 IP 与主机 Wi-Fi/RDP 地址冲突。

原因：

- 主机 `wlo1` 使用 `192.168.1.3/24`；
- MID360 之前也使用 `192.168.1.3`；
- 这会影响远程桌面路径，不能为了 LiDAR 改坏 Wi-Fi。

解决：

- 将 MID360 改为 `192.168.1.30`；
- 主机有线 Livox 侧使用 `192.168.1.50`；
- 不要改 Wi-Fi/RDP 的 `wlo1` 地址；
- 使用 `/home/vision/FAST-Calib/config/livox_mid360_fast_calib.json`。

## 2. MID360 driver 输出类型必须是 PointCloud2

现象：

- FAST-Calib 读取 bag 时需要 `sensor_msgs/msg/PointCloud2`；
- Livox 原始自定义消息不能直接给当前 FAST-Calib 流程用。

解决：

使用本工程里的 launch：

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

## 3. MVS SDK 的旧 libusb 会破坏 PCL

现象：

运行 FAST-Calib 或手工四孔工具时报错：

```text
symbol lookup error: /lib/x86_64-linux-gnu/libpcl_io.so.1.12: undefined symbol: libusb_set_option
```

原因：

- Hikvision MVS SDK 的 `/opt/MVS/lib/64/libusb-1.0.so.0` 被优先加载；
- 系统 PCL 需要系统版本的 libusb。

解决：

运行 FAST-Calib 前过滤 `/opt/MVS/lib`：

```bash
CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)
env LD_LIBRARY_PATH="$CLEAN_LD" ros2 run fast_calib manual_lidar_centers_calib ...
```

`scripts/run_fast_calib_scene.sh` 已经做了这件事。

## 4. 反光和背景会影响 LiDAR 孔洞识别

现象：

- 后面黑色背景板可能反光；
- 开灯时点云里背景/板面边界干扰更明显；
- 自动 LiDAR 圆洞检测只能找到部分孔。

处理：

- 关灯后重新采集；
- 尽量避免背景板强反光；
- 让标定板完整、稳定地出现在 MID360 可见范围内；
- 采集 25s 左右，积累足够点云。

## 5. 坐标方向不要搞反

本次现场确认的 LiDAR 坐标关系：

- 标定板在 LiDAR `+X` 方向；
- 标定板中心附近 `Y≈0`；
- 左右跨在 `+Y/-Y` 之间；
- 往上是 `+Z`。

这对检查 ROI 和人工孔位非常重要。

## 6. ROI 要覆盖完整标定板

现象：

- ROI 太窄时只剩一部分板面；
- 自动检测只能抓到上排两个孔；
- 用户在 RViz 里能看见四个孔，但算法没有检测到。

本次最终 ROI：

```yaml
x_min: 2.20
x_max: 4.40
y_min: -0.80
y_max: 0.80
z_min: -0.60
z_max: 1.50
```

注意：

- 盲目扩大 ROI 会引入背景和假圆；
- 本次 `xy_wide_b` 产生了可疑结果，`RMSE=0.626 m` 且 `Pcl z=8.397 m`，不要使用。

## 7. 自动 LiDAR 圆洞检测只找到两个孔

现象：

最新有效数据 `current_static_pair_y0_full_20260617` 中自动检测只找到：

```text
hole +Y: x=3.57454, y= 0.07096, z=0.11873
hole -Y: x=3.55390, y=-0.43180, z=0.14098
```

原因：

- FAST-Calib 的 LiDAR 流程是先提取板面，再提边缘，再拟合 2D 圆；
- 下排两个孔在原始点云里能看见，但边缘/聚类/拟合没有稳定通过；
- 严格按 `z=-0.13±0.1` 搜时，候选顶在下边界，说明下排孔中心更低。

解决：

把 Z 搜索往下扩展后，人工确认下排两个候选：

```text
lower +Y: x=3.54801, y= 0.06100, z=-0.27500
lower -Y: x=3.52758, y=-0.45020, z=-0.25500
```

四个点几何关系：

- 上排左右间距约 `0.503 m`；
- 下排左右间距约 `0.512 m`；
- 上下间距约 `0.394 m` 到 `0.396 m`；
- 与标定板 `0.500 m x 0.400 m` 基本一致。

## 8. RDP 下不要实时播放点云

现象：

- RDP 远程桌面显示实时点云时非常卡；
- `ros2 bag play -l` 循环播放 `/livox/lidar` 会持续推高频点云。

解决：

- 停止 bag 播放；
- 用累计后的静态点云 `filtered_cloud.ply`；
- RViz 只显示 `/static_accumulated_cloud` 和 `/detected_holes`；
- 发布频率设为 `0.2 Hz`。

命令：

```bash
python3 scripts/publish_static_cloud_with_holes.py \
  --cloud output/final_success_20260617/filtered_cloud.ply \
  --rate 0.2
```

RViz 配置：

```text
rviz_cfg/static_accumulated_holes.rviz
```

## 9. RViz 里 Marker 看不见

现象：

- 用户看不到两个球；
- 半透明球和点云颜色混在一起；
- 没点云或 topic 不对。

解决：

- 球改成全不透明实心球；
- 球直径使用 `0.35 m` 方便在 RDP 中看；
- 点云 topic 使用 `/static_accumulated_cloud`；
- Marker topic 使用 `/detected_holes`；
- Fixed Frame 使用 `livox_frame`。

检查：

```bash
ros2 topic list -t | grep -E 'static_accumulated_cloud|detected_holes'
ros2 topic echo --once /detected_holes
```

## 10. 参数文件节点名要匹配

现象：

手工四孔工具第一次运行时没有读取 scene config，退回默认 image path：

```text
Loading the image /home/chunran/calib_ws/src/fast_calib/data/image.png failed
```

原因：

YAML 顶层节点是：

```yaml
fast_calib:
  ros__parameters:
```

工具节点名如果不是 `fast_calib`，ROS2 不会把这些参数加载进去。

解决：

`tools/manual_lidar_centers_calib.cpp` 使用节点名：

```cpp
auto node = std::make_shared<rclcpp::Node>("fast_calib");
```

## 11. 不要信明显异常的外参结果

本次有一个自动/宽 ROI 的可疑结果：

```text
/home/vision/FAST-Calib/output/current_static_pair_y0_full_20260617_xy_wide_b/calib_result.txt
RMSE: 0.626 m
Pcl z: 8.397513
```

这个结果不符合现场几何，不能用。

最终采用结果：

```text
/home/vision/FAST-Calib/output/final_success_20260617/calib_result.txt
RMSE: 0.004935 m
```

