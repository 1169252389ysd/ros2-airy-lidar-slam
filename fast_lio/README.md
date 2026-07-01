# FAST-LIO ROS2 — RoboSense Airy 96线 完整部署

基于 FAST-LIO2 的 ROS2 Humble 移植，专门适配 **RoboSense Airy (RSAIRY) 96线** 机械式激光雷达及其内置 IMU。

## 代码来源

本仓库在原作基础上做了 Airy 雷达适配和 ROS2 部署修复，主要引用：

| 仓库 | 贡献 |
|------|------|
| [hku-mars/FAST-LIO](https://github.com/hku-mars/FAST-LIO) | 原始 FAST-LIO/FAST-LIO2 算法 |
| [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | ROS2 Humble 移植 |
| [Ilyes-Origamist/robosense_fast_lio](https://github.com/Ilyes-Origamist/robosense_fast_lio) (RS-Airy 分支) | Airy 雷达预处理、坐标系对齐 |
| 本仓库 | Airy 96线完整部署：地图累积修复、NED→FLU 外参方案、参数精调 |

## 硬件

- 激光雷达：RoboSense Airy (RSAIRY) 96线（360° 机械旋转式）
- IMU：Airy 内置 IMU（NED 坐标系）
- 系统：Ubuntu 22.04 + ROS2 Humble

## 1. 依赖

```bash
# ROS2 Humble
# PCL >= 1.8（apt 默认版本即可）
# Eigen >= 3.3.4（apt 默认版本即可）

# RoboSense 激光雷达驱动
cd ~/ros2_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
# 编译前修改 CMakeLists.txt：
#   set(POINT_TYPE XYZIRT)
#   set(ENABLE_IMU_DATA_PARSE ON)
cd ~/ros2_ws
colcon build --packages-select rslidar_sdk
```

## 2. 编译

```bash
cd ~/fastlio_ws/src
git clone <本仓库地址> --recursive
cd ~/fastlio_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_lio --symlink-install
source install/setup.bash
```

## 3. 运行

**先启雷达驱动，再启 FAST-LIO：**

```bash
# 终端 1：雷达驱动
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rslidar_sdk humble_start.py

# 终端 2：FAST-LIO
source /opt/ros/humble/setup.bash
source ~/fastlio_ws/install/setup.bash
ros2 launch fast_lio mapping_robosenseAiry.launch.py
```

启动后**保持静止 3 秒以上**，等终端出现 `IMU Initial Done` 再开始移动。

## 4. 关键配置

`config/robosenseAiry.yaml`：

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_type` | 5 | RSAIRY 专用 handler |
| `scan_line` | 96 | Airy 96线 |
| `timestamp_unit` | 0 | Airy timestamp 单位是秒（非微秒） |
| `extrinsic_R` | Ilyes 标定值 | 同时处理 NED→FLU 坐标系转换 + 物理安装偏差 |
| `point_filter_num` | 3 | 每 3 个点保留 1 个 |
| `deskew_en` | true | 运动补偿（去畸变） |

## 5. 修改要点

相较于原始 FAST-LIO ROS2 版本，本项目做了以下关键修改：

- `preprocess.h/cpp` — 新增 `RSAIRY=5` 雷达类型和 `robosenseAiry_handler`，支持 XYZIRT（调用 M1 handler 正确使用逐点 timestamp）和 XYZI（体素降采样）双模式
- `laserMapping.cpp` — 地图累积修复（`pcl_wait_pub` 每帧累积 + 1Hz 发布到 `/Laser_map`）、外参在线估计支持
- `IMU_Processing.hpp` — `deskew_en` 运动补偿可配置开关
- `config/robosenseAiry.yaml` — Airy 96线完整参数（IMU 协方差、高度过滤、面匹配距离等）

## 6. 保存地图

```bash
ros2 service call /map_save std_srvs/srv/Trigger
# 保存到 yaml 中 map_file_path 指定的路径
```

## 7. RViz 话题

| 话题 | 内容 |
|------|------|
| `/cloud_registered` | 当前帧扫描（世界坐标系） |
| `/Laser_map` | 累积的完整地图 |
| `/Odometry` | 里程计位姿 |
| `/path` | 运动路径 |

## 致谢

原始算法：[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](https://arxiv.org/abs/2107.06829) — Xu Wei, Cai Yixi, He Dongjiao, Lin Jiarong, Zhang Fu (HKU MARS Lab)

## 许可证

BSD 3-Clause（与原始 FAST-LIO 一致）
