# ros2-airy-lidar-slam

基于 ROS2 Humble + RoboSense Airy 96线激光雷达的 SLAM 算法集合。

## 算法

| 目录 | 算法 | 说明 |
|------|------|------|
| [fast_lio/](fast_lio/) | FAST-LIO2 | 紧耦合 LiDAR-IMU 里程计（EKF 迭代卡尔曼滤波） |

## 核心贡献

- **Airy 外参标定**：NED→FLU 坐标系对齐 + LiDAR-IMU 物理安装偏差，`extrinsic_R ≈ R_z(-89.8°) × R_x(179.7°)`
- 外参标定方法文档化，新设备部署可复现

## 硬件

- 激光雷达：RoboSense Airy (RSAIRY) 96线（360° 机械式）
- IMU：Airy 内置 IMU（NED 坐标系）
- 系统：Ubuntu 22.04 + ROS2 Humble

## 代码来源

| 仓库 | 作者 | 贡献 |
|------|------|------|
| [hku-mars/FAST-LIO](https://github.com/hku-mars/FAST-LIO) | HKU MARS Lab | 原始算法 |
| [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2) | Ericsii | ROS2 移植 |
| [Ilyes-Origamist/robosense_fast_lio](https://github.com/Ilyes-Origamist/robosense_fast_lio) | Ilyes-Origamist | Airy handler + 外参标定 |

## 快速开始

详见各子目录下的 README。

## 许可证

BSD 3-Clause
