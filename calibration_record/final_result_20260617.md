# 2026-06-17 最终标定结果

## 数据来源

```text
Image:
/home/vision/FAST-Calib/calib_data/final_success_20260617/image.png

LiDAR bag:
/home/vision/FAST-Calib/calib_data/final_success_20260617/lidar_bag

Config:
/home/vision/FAST-Calib/config/qr_params_final_success_20260617.yaml

Automatic FAST-Calib output:
/home/vision/FAST-Calib/output/final_success_20260617

Manual four-hole output:
/home/vision/FAST-Calib/output/final_success_20260617
```

## 为什么采用手工四孔

自动 LiDAR 检测在这组数据上只稳定检测到上排两个孔：

```text
upper +Y: x=3.57454, y= 0.07096, z=0.11873
upper -Y: x=3.55390, y=-0.43180, z=0.14098
```

RViz 中累计静态点云可以看到四个孔。根据现场确认和离线点云搜索，补充下排两个孔：

```text
lower +Y: x=3.54801, y= 0.06100, z=-0.27500
lower -Y: x=3.52758, y=-0.45020, z=-0.25500
```

四个孔几何关系符合标定板尺寸：

```text
width:  about 0.50 m
height: about 0.40 m
```

因此最终外参使用 `tools/manual_lidar_centers_calib.cpp` 计算。

## 最终外参

结果文件：

```text
/home/vision/FAST-Calib/output/final_success_20260617/calib_result.txt
```

内容：

```yaml
# FAST-LIVO2 calibration format
cam_model: Pinhole
cam_width: 1440
cam_height: 1080
scale: 1.0
cam_fx: 1794.97
cam_fy: 1793.23
cam_cx: 706.1
cam_cy: 553.756
cam_d0: -0.0628587
cam_d1: 0.0902734
cam_d2: -0.000450262
cam_d3: 0.00100354

Rcl: [  0.006071,  -0.999079,   0.042474,
        0.013104,  -0.042391,  -0.999015,
        0.999896,   0.006622,   0.012835]
Pcl: [  0.022384,  -0.085765,   0.002566]
```

四孔 SVD 配准 RMSE：

```text
0.004935 m
```

## 生成的检查文件

```text
/home/vision/FAST-Calib/output/final_success_20260617/calib_result.txt
/home/vision/FAST-Calib/output/final_success_20260617/colored_cloud.pcd
/home/vision/FAST-Calib/output/final_success_20260617/colored_cloud.ply
/home/vision/FAST-Calib/output/final_success_20260617/qr_detect.png
```

`colored_cloud.pcd` 信息：

```text
POINTS 261904
FIELDS x y z rgb
```

## 本次使用的四个 LiDAR 孔位

```text
upper +Y: x=3.57454, y= 0.07096, z= 0.11873
upper -Y: x=3.55390, y=-0.43180, z= 0.14098
lower +Y: x=3.54801, y= 0.06100, z=-0.27500
lower -Y: x=3.52758, y=-0.45020, z=-0.25500
```

## 复现命令

```bash
cd /home/vision/FAST-Calib
source /opt/ros/humble/setup.bash
source install/setup.bash

mkdir -p output/final_success_20260617

CLEAN_LD=$(printf '%s' "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS/lib' | paste -sd:)

env LD_LIBRARY_PATH="$CLEAN_LD" ros2 run fast_calib manual_lidar_centers_calib \
  --ros-args \
  --params-file config/qr_params_final_success_20260617.yaml \
  -p output_path:=/home/vision/FAST-Calib/output/final_success_20260617
```

## 不要使用的结果

以下结果是宽 ROI 或误检导致的异常结果，不要作为最终外参：

```text
/home/vision/FAST-Calib/output/current_static_pair_y0_full_20260617_xy_wide_b/calib_result.txt
RMSE: 0.626 m
Pcl z: 8.397513
```

