# FAST-LIO ROS2 — RoboSense Airy 96线

基于 [FAST-LIO2](https://github.com/hku-mars/FAST-LIO) 的 ROS2 Humble 移植，专门适配 **RoboSense Airy (RSAIRY) 96线** 360° 机械式激光雷达及内置 IMU。

## 核心贡献：Airy 外参标定

Airy 内置 IMU 输出 **NED 坐标系**（X前-Y右-Z下），而 FAST-LIO 内部使用 **FLU 坐标系**（X前-Y左-Z上），且 LiDAR 与 IMU 芯片存在约 90° 的物理安装偏转。`extrinsic_R` 必须同时补偿这两个差异。

外参标定标准文档见：[IMU 外参标定标准](../docs/imu_extrinsic_calibration_standard.docx)。

详见 RoboSense 官方文档：[链接待补充]

| | LiDAR 坐标系 | IMU 坐标系 |
|---|-------------|-----------|
| X 轴 | 前 | 前 |
| Y 轴 | 右 | 右 |
| Z 轴 | 上 | **下**（NED，重力方向） |

`extrinsic_R` 约等于 **R_z(-89.8°) × R_x(179.7°)**，即 IMU 芯片相对于 LiDAR 有约 -90° yaw + Z 轴翻转。这也是为什么直接使用单位矩阵做外参会失败——它忽略了物理安装偏转和坐标系差异。

> 换一台新 Airy 设备部署时，先启用 `extrinsic_est_en: true` 跑几分钟做在线标定，确认 `ext_roll ≈ 180°, ext_yaw ≈ -90°` 即说明该设备与当前标定值一致。

## 代码来源

| 仓库 | 作者 | 贡献 |
|------|------|------|
| [hku-mars/FAST-LIO](https://github.com/hku-mars/FAST-LIO) | HKU MARS Lab | 原始 FAST-LIO/FAST-LIO2 算法 |
| [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | [Ericsii](https://github.com/Ericsii) | ROS2 Humble 移植 |
| [Ilyes-Origamist/robosense_fast_lio](https://github.com/Ilyes-Origamist/robosense_fast_lio) (RS-Airy) | [Ilyes-Origamist](https://github.com/Ilyes-Origamist) | Airy handler、外参标定值、坐标系方案 |
| [RuanJY/robosense_fast_lio](https://github.com/RuanJY/robosense_fast_lio) | [RuanJY](https://github.com/RuanJY) | Airy 点云预处理适配 |
| 本仓库 | — | 96线部署完整文档、外参原理、地图累积修复、参数精调 |

## 硬件

- 激光雷达：RoboSense Airy (RSAIRY) 96线
- IMU：Airy 内置 IMU
- 系统：Ubuntu 22.04 + ROS2 Humble

## 1. 依赖安装

```bash
# PCL、Eigen 用系统默认版本即可

# 创建工作区
mkdir -p ~/slam_ws/src
cd ~/slam_ws/src

# Livox ROS2 驱动（编译依赖，代码中引用其消息类型）
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# RoboSense 激光雷达驱动
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
# 修改 rslidar_sdk/CMakeLists.txt：
#   set(POINT_TYPE XYZIRT)
#   set(ENABLE_IMU_DATA_PARSE ON)

# FAST-LIO（本仓库）
git clone <本仓库地址>

cd ~/slam_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 2. 编译

```bash
cd ~/slam_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. 运行

**必须先启动雷达驱动：**

```bash
# 终端 1：雷达驱动
source /opt/ros/humble/setup.bash && source ~/slam_ws/install/setup.bash
ros2 launch rslidar_sdk humble_start.py

# 终端 2：FAST-LIO
source /opt/ros/humble/setup.bash && source ~/slam_ws/install/setup.bash
ros2 launch fast_lio mapping_robosenseAiry.launch.py
```

⚠️ 启动后**保持静止 ≥3 秒**，等终端出现 `IMU Initial Done` 再移动。否则重力估计错误，建图失败。

## 4. 关键参数

`config/robosenseAiry.yaml`：

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_type` | 5 | RSAIRY handler |
| `scan_line` | 96 | Airy 96线 |
| `timestamp_unit` | 0 | Airy timestamp 是**秒**（容易踩坑！） |
| `extrinsic_R` | 标定矩阵 | 同时处理 NED→FLU + 安装偏转 |
| `extrinsic_est_en` | false | 在线标定开关（新设备建议先开） |
| `point_filter_num` | 3 | 每 3 点保留 1 点 |
| `deskew_en` | true | 运动补偿（去畸变） |

### 外参在线标定

```
yaml 设置 extrinsic_est_en: true + runtime_pos_log_enable: true
跑几分钟后查看：
tail -f ~/fastlio_ws/src/FAST_LIO_ROS2/Log/mat_pre.txt
日志中 ext_roll/ext_pitch/ext_yaw 为实时估计的欧拉角（度）
```

## 5. 主要修改

- `preprocess.h/cpp` — `RSAIRY=5` 雷达类型 + `robosenseAiry_handler`：XYZIRT → M1 handler，XYZI → 体素降采样
- `laserMapping.cpp` — 地图累积修复（每帧累积 + 1Hz 发布 `/Laser_map`），外参在线估计启用
- `IMU_Processing.hpp` — `deskew_en` 可配置开关
- `launch/mapping_robosenseAiry.launch.py` — Airy 专用启动文件 + 显示 TF 补偿

## 6. 保存地图

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

## 7. RViz 话题

| 话题 | 内容 |
|------|------|
| `/cloud_registered` | 当前帧（世界坐标系） |
| `/Laser_map` | 累积地图 |
| `/Odometry` | 里程计 |
| `/path` | 运动路径 |

## 致谢

[FAST-LIO2](https://arxiv.org/abs/2107.06829) — Xu Wei, Cai Yixi, He Dongjiao, Lin Jiarong et al. (HKU MARS Lab)

## 许可证

BSD 3-Clause
