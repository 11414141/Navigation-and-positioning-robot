# 移动机器人建图、定位与巡检导航项目

## 项目简介

面向室外园区和农田等场景，构建了基于 LiDAR、RTK 与 ROS 2 的机器人建图、定位和多点巡检导航流程。系统支持将采集到的 Livox Mid-360 点云离线建图，生成二维导航地图，并完成语义地点标记、路线生成与自动巡检。

## 系统流程

```text
Livox Mid-360 点云与 IMU
          |
          v
     FAST-LIO2 离线建图
          |
          v
  PCD 处理与二维地图生成
          |
          +-------------------+
          |                   |
          v                   v
     RTK 定位导航          AMCL 定位导航
          |                   |
          +---------+---------+
                    v
           语义地点与多点巡检
```

## 主要工作

- 建立 Mid-360 点云、IMU、底盘里程计与 RTK 状态的采集和检查流程。
- 配置 FAST-LIO2 离线回放建图，输出三维 PCD 地图和轨迹。
- 实现地图地面切片、噪点过滤和 PCD 到 Nav2 二维占据栅格地图转换。
- 建立 RTK 锚点、地图坐标对齐、语义地点记录和地点路线生成流程。
- 集成 RTK Nav2 与 AMCL Nav2 两套相互独立的导航方案。
- 提供巡检路线执行、bag 完整性检查、定位链路诊断和 RViz 可视化辅助工具。

## 技术栈

`ROS 2 Humble` · `C++` · `Python` · `Livox Mid-360` · `FAST-LIO2` · `PCL` · `Nav2` · `RTK/GNSS` · `RViz2`

## 项目代码

- [FAST-LIO2 与 Mid-360 配置](workspace/fast_lio)
- [Livox 驱动](workspace/livox_ros_driver2)
- [点云转换与地图工具](workspace/livox_nav_tools)
- [PCD 到二维地图转换](workspace/pcd2pgm)
- [建图、定位、路线与巡检脚本](scripts/rtk_tools)

## 说明

公开仓库不包含现场地图、原始 rosbag、语义坐标、RTK/NTRIP 凭据和硬件私有配置。相关内容已在发布前脱敏，以保护现场数据和设备安全。
