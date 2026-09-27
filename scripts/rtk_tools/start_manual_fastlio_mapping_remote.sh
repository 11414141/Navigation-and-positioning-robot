#!/usr/bin/env bash
set -euo pipefail

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
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

SITE_NAME=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --site)
      SITE_NAME="${2:-}"
      shift 2
      ;;
    *)
      echo "错误: 未知参数 $1" >&2
      exit 2
      ;;
  esac
done

if [ -n "$SITE_NAME" ]; then
  SITE_DIR="$(site_dir_from_name "$SITE_NAME")"
  ensure_site_dirs "$SITE_DIR"
  export SITE_DIR
fi

FASTLIO_START_DELAY="${FASTLIO_START_DELAY:-3}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-true}"
SUPPORT_PIDS=()

check_existing_nodes() {
  local node_list conflicts
  set +e
  node_list="$(ros2 node list 2>/dev/null)"
  local status=$?
  set -e
  if [ "$status" -ne 0 ]; then
    echo "警告: 无法查询 ros2 node list，跳过残留节点检查。"
    return 0
  fi

  conflicts="$(printf '%s\n' "$node_list" | grep -E '^/(livox_lidar_publisher|laser_mapping|livox_custom_to_pointcloud2)$' || true)"
  if [ -n "$conflicts" ]; then
    echo "错误: 检测到旧的雷达/FAST-LIO2相关节点仍在运行，请先停止当前任务。"
    echo "$conflicts" | sed 's/^/  /'
    exit 1
  fi
}

start_support_nodes() {
  echo "启动远程手动FAST-LIO2建图前置节点:"
  echo "  base_serial, robot_description, Livox MID360s, livox_frame静态TF"

  ros2 launch turn_on_wheeltec_robot base_serial.launch.py &
  SUPPORT_PIDS+=("$!")

  ros2 launch turn_on_wheeltec_robot robot_mode_description.launch.py &
  SUPPORT_PIDS+=("$!")

  ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
  SUPPORT_PIDS+=("$!")

  ros2 run tf2_ros static_transform_publisher \
    --x 0.14 --y 0.0 --z 0.16 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_footprint --child-frame-id livox_frame &
  SUPPORT_PIDS+=("$!")

  echo "等待雷达/IMU稳定 ${FASTLIO_START_DELAY}s 后启动FAST-LIO2..."
  sleep "$FASTLIO_START_DELAY"
}

stop_support_nodes() {
  if [ "${#SUPPORT_PIDS[@]}" -gt 0 ]; then
    echo
    echo "停止远程手动建图前置节点..."
    local pid
    for pid in "${SUPPORT_PIDS[@]}"; do
      kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${SUPPORT_PIDS[@]}"; do
      wait "$pid" 2>/dev/null || true
    done
    SUPPORT_PIDS=()
  fi
}

trap 'stop_support_nodes' EXIT

check_existing_nodes
start_support_nodes

echo
echo "启动FAST-LIO2建图。远程停止任务后会自动执行: 3D PCD -> 切片 -> 2D Nav2地图。"
echo

FASTLIO_START_DELAY=0 RVIZ="$FASTLIO_RVIZ" /home/wheeltec/rtk_tools/start_fastlio_mapping_auto_2d.sh &
FASTLIO_PID=$!
if [ -n "$SITE_NAME" ]; then
  (
    sleep "${AUTO_HOME_DELAY:-5}"
    echo "自动标定起始点为充电站:"
    /home/wheeltec/rtk_tools/myrobot_places.py record-origin --site "$SITE_NAME" --name "充电站" || true
  ) &
fi
wait "$FASTLIO_PID"

if [ -n "${SITE_DIR:-}" ] && [ ! -f "$(site_current_yaml "$SITE_DIR")" ]; then
  echo
  echo "当前地点尚无Nav2全局地图，自动将首次建图候选地图更新为全局地图..."
  /home/wheeltec/rtk_tools/myrobot_site_manager.py update-candidate --site "$SITE_NAME"
fi
