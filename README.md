# Navigation and Positioning Robot

## 中文项目概览

这是一个面向室外移动机器人的建图、定位与巡检导航软件项目，使用 Livox Mid-360、ROS 2、FAST-LIO2、RTK 和 Nav2 构建从数据采集到多点巡检的完整软件流程。

招聘方可先阅读：[中文项目首页](PROJECT_OVERVIEW_CN.md)。

ROS 2 Humble robot-side source snapshot for Livox Mid-360 acquisition, FAST-LIO2 offline mapping, PCD-to-2D-map conversion, and site-based Nav2 workflow helpers.

This repository is a public, privacy-reviewed export made from a working robot. It is not a complete plug-and-play image: hardware-specific calibration, network settings, GNSS/NTRIP credentials, field maps, semantic places, and recorded bags are intentionally excluded.

## Included

- `workspace/fast_lio`: FAST-LIO2 source with the local Mid-360 integration changes already present in the robot workspace.
- `workspace/livox_ros_driver2`: Livox ROS Driver 2 source. Use `config/MID360_config.example.json` as a starting point for the local network configuration.
- `workspace/livox_nav_tools`: local Livox CustomMsg to PointCloud2/LaserScan conversion tools.
- `workspace/pcd2pgm`: PCD to Nav2 occupancy-map converter, with local changes retained.
- `scripts/rtk_tools`: acquisition, offline mapping, RTK place, route, RTK Nav2, and AMCL Nav2 helper scripts. Private GNSS launch configuration is required locally and is not provided here.
- `docs`: workflow, privacy boundary, and local-change records.
- `environment`: tested platform summary.

## Not Included

- NTRIP usernames, passwords, tokens, caster addresses, or private GNSS launch files.
- SSH keys, Wi-Fi/campus-network configuration, system netplan files, serial-device rules, or personal credentials.
- Field-site directories, semantic waypoint coordinates, maps, PCD files, rosbag data, trajectories, and runtime logs.
- ROS `build`, `install`, and `log` outputs.

## Tested Baseline

- Ubuntu 22.04.5 LTS, ARM64 (`aarch64`)
- ROS 2 Humble
- GCC 11.4.0, CMake 3.22.1, Python 3.10.12
- Livox Mid-360 and FAST-LIO2

See [docs/workflow.md](docs/workflow.md) before building. See [SECURITY.md](SECURITY.md) before contributing issue reports or configuration examples.

## License and Upstream Projects

This repository contains source snapshots from third-party projects, retaining their original license files. In particular, FAST-LIO and pcd2pgm have their own upstream licenses. Review each package's license before redistribution or commercial use.
