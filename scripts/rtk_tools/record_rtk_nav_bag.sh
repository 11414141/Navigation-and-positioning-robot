#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-full}"
if [ "$MODE" != "full" ] && [ "$MODE" != "fastlio" ] && [ "$MODE" != "glim" ]; then
  echo "用法:"
  echo "  $0 full      # 推荐：同时录FAST-LIO2、GLIM、RTK/Nav2复现所需话题"
  echo "  $0 fastlio   # 只录FAST-LIO2建图和RTK/里程计核心话题"
  echo "  $0 glim      # 只录GLIM常用PointCloud2、IMU和RTK/里程计核心话题"
  exit 2
fi

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

source_if_exists /home/wheeltec/rtk_tools/site_common.sh

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash

DATE_STR="$(date +%Y%m%d)"
TIME_STR="$(date +%Y%m%d_%H%M%S)"
if [ -n "${SITE_DIR:-}" ]; then
  ensure_site_dirs "$SITE_DIR"
  BAG_ROOT="${BAG_ROOT:-$(site_bags_dir "$SITE_DIR")/rtk_nav_${DATE_STR}}"
else
  BAG_ROOT="${BAG_ROOT:-/home/wheeltec/bags/rtk_nav_${DATE_STR}}"
fi
mkdir -p "$BAG_ROOT"

case "$MODE" in
  full)
    BAG_NAME="rtk_nav_full_${TIME_STR}"
    TOPICS=(
      /livox/lidar
      /livox/lidar_points
      /livox/lidar_points_nav
      /livox/lidar_points_glim
      /scan
      /livox/imu
      /odom
      /odom_combined
      /odometry/global
      /odometry/gps
      /imu/data_raw
      /gps/fix
      /gps/fix_filtered
      /nmea_sentence
      /gnss/gpgga
      /gps/vel
      /gps/nmea/vel
      /gps/pose
      /heading
      /heading_deg
      /time_reference
      /rtk/status
      /tf
      /tf_static
      /map
      /global_costmap/costmap
      /global_costmap/costmap_updates
      /local_costmap/costmap
      /local_costmap/costmap_updates
      /plan
      /cmd_vel
      /cmd_vel_nav
      /parameter_events
      /rosout
      /diagnostics
    )
    ;;
  fastlio)
    BAG_NAME="fastlio_custommsg_${TIME_STR}"
    TOPICS=(
      /livox/lidar
      /livox/imu
      /odom
      /odom_combined
      /imu/data_raw
      /gps/fix
      /gps/fix_filtered
      /rtk/status
      /tf
      /tf_static
      /parameter_events
      /rosout
      /diagnostics
    )
    ;;
  glim)
    BAG_NAME="glim_pointcloud2_${TIME_STR}"
    TOPICS=(
      /livox/lidar_points_glim
      /scan
      /livox/imu
      /odom
      /odom_combined
      /imu/data_raw
      /gps/fix
      /gps/fix_filtered
      /rtk/status
      /tf
      /tf_static
      /parameter_events
      /rosout
      /diagnostics
    )
    ;;
esac

OUT_PATH="${OUT_PATH:-${BAG_ROOT}/${BAG_NAME}}"
QOS_FILE="/home/wheeltec/rtk_tools/record_rtk_nav_bag_qos.yaml"
CACHE_SIZE="${CACHE_SIZE:-1073741824}"
MAX_BAG_DURATION="${MAX_BAG_DURATION:-0}"

echo "========== RTK/Nav 一键录包 =========="
echo "mode: $MODE"
echo "output: $OUT_PATH"
echo "qos: $QOS_FILE"
echo "cache_size: $CACHE_SIZE"
echo "max_bag_duration: $MAX_BAG_DURATION"
echo
echo "录制话题:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "按 Ctrl+C 正常停止录制。不要直接断电，否则bag可能损坏。"
echo

exec ros2 bag record \
  -o "$OUT_PATH" \
  --max-cache-size "$CACHE_SIZE" \
  --max-bag-duration "$MAX_BAG_DURATION" \
  --qos-profile-overrides-path "$QOS_FILE" \
  "${TOPICS[@]}"
