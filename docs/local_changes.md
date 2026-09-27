# Local Changes and Upstream Baselines

## FAST-LIO2

- Upstream: `https://github.com/Ericsii/FAST_LIO.git`
- Branch: `ros2`
- Base commit: `2fffc570a25d0df172720bac034fbdb6a13d2162`
- Locally modified tracked files: `CMakeLists.txt`, `config/mid360.yaml`, `launch/mapping.launch.py`, `rviz/fastlio.rviz`, `src/IMU_Processing.hpp`, `src/laserMapping.cpp`, `src/preprocess.cpp`.
- Local added source retained: `src/livox_custom_to_pointcloud2.cpp`.

The original workspace also contained `.backup` copies of two source files. They are intentionally excluded from the public export.

## pcd2pgm

- Upstream: `https://github.com/LihanChen2004/pcd2pgm.git`
- Branch: `main`
- Base commit: `4602ebc53da00bddcd03053915af0e2875dbd580`
- Locally modified files: `config/pcd2pgm.yaml`, `include/pcd2pgm/pcd2pgm.hpp`, `src/pcd2pgm.cpp`.

## Local Integration Packages

- `workspace/livox_nav_tools` is local integration code for converting Livox CustomMsg into PointCloud2 and LaserScan.
- `scripts/rtk_tools` is a local workflow layer for collection, offline mapping, map processing, semantic places, RTK Nav2, and AMCL Nav2.
