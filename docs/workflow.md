# Build and Workflow

This export captures source and helper scripts from a ROS 2 Humble robot. It cannot be run unchanged on another robot because calibration, package names, network configuration, and GNSS credentials are deployment-specific.

## Source layout

Copy the packages needed by your deployment into a colcon workspace `src` directory, then resolve dependencies and build. The four included packages are independent source snapshots, not a pre-built workspace.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/robot_ws/src
# Copy or symlink the required packages from workspace/ into ~/robot_ws/src.
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Livox Mid-360

Create a local `MID360_config.json` from `workspace/livox_ros_driver2/config/MID360_config.example.json`. Set the host NIC and sensor addresses for the local isolated LiDAR network. Do not commit that local file when it contains deployment-specific addresses.

## FAST-LIO2 Offline Mapping

The source is in `workspace/fast_lio`; its active local configuration is `config/mid360.yaml`. The helper `scripts/rtk_tools/run_fastlio_offline_from_bag.sh` documents the robot-side offline replay flow. Supply your own input bag and adjust package/workspace paths for the target machine.

## RTK Workflows

The public scripts do not include NTRIP configuration. Before running an RTK helper, create and protect a private ROS launch file, then set:

```bash
export GNSS_PRIVATE_LAUNCH=/absolute/path/to/zed_f9p_ntrip_private.launch.py
```

Use only credentials you own or are authorized to use. Site maps and semantic waypoints are local data and remain outside this repository.
