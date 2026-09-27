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

resolve_image_path() {
  local yaml_file="$1"
  local image_line
  image_line="$(grep -E '^image:' "$yaml_file" | head -1 | sed 's/^image:[[:space:]]*//')"
  if [[ "$image_line" = /* ]]; then
    printf '%s\n' "$image_line"
  else
    printf '%s/%s\n' "$(dirname "$yaml_file")" "$image_line"
  fi
}

install_current_map() {
  local candidate_yaml="$1"
  local current_dir="/home/wheeltec/maps/nav2_current"
  if [ -n "${SITE_DIR:-}" ]; then
    current_dir="$SITE_DIR/maps/nav2_current"
  fi
  local current_yaml="$current_dir/current.yaml"
  local current_pgm="$current_dir/current.pgm"
  local candidate_image

  candidate_image="$(resolve_image_path "$candidate_yaml")"
  if [ ! -f "$candidate_image" ]; then
    echo "错误: 候选地图图像不存在: $candidate_image" >&2
    return 1
  fi

  mkdir -p "$current_dir"
  cp "$candidate_image" "$current_pgm"
  cp "$candidate_yaml" "$current_yaml"
  sed -i 's|^image:.*|image: current.pgm|' "$current_yaml"
}

check_existing_nodes() {
  local node_list
  local conflicts

  set +e
  node_list="$(ros2 node list 2>/dev/null)"
  local status=$?
  set -e

  if [ "$status" -ne 0 ]; then
    echo "警告: 无法查询 ros2 node list，跳过残留节点检查。"
    echo "      如果稍后 map_server 或 GNSS 串口报错，请先确认没有旧launch仍在运行。"
    return 0
  fi

  conflicts="$(printf '%s\n' "$node_list" | grep -E \
    '^/(map_server|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager_rtk_navigation|zed_f9p_ntrip_node|rtk_quality_filter|ekf_filter_node_odom|ekf_filter_node_map|navsat_transform|livox_custom_to_pointcloud2|livox_lidar_publisher)$' || true)"

  if [ -n "$conflicts" ]; then
    echo "错误: 检测到旧的RTK/Nav2相关节点仍在运行，先不要重复启动。"
    echo
    echo "$conflicts" | sed 's/^/  /'
    echo
    echo "请先到之前启动这些节点的终端按 Ctrl+C，或重启ROS相关终端/小车系统。"
    echo "旧 map_server active 会导致 lifecycle configure 失败；旧 GNSS 节点会导致串口 multiple access。"
    exit 1
  fi
}

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash

if [ -n "${SITE_DIR:-}" ]; then
  ensure_site_dirs "$SITE_DIR"
fi

check_existing_nodes

if [ -n "${SITE_DIR:-}" ]; then
  CURRENT_YAML="$(site_current_yaml "$SITE_DIR")"
  LATEST_FILE="$(site_latest_candidate_file "$SITE_DIR")"
else
  CURRENT_YAML="/home/wheeltec/maps/nav2_current/current.yaml"
  LATEST_FILE="/home/wheeltec/maps/nav2_latest_candidate.txt"
fi
FALLBACK_YAML="/home/wheeltec/ws_2dmap/maps/verygood_map.yaml"
CANDIDATE_YAML=""

if [ -f "$LATEST_FILE" ]; then
  CANDIDATE_YAML="$(cat "$LATEST_FILE")"
fi

echo "========== 启动RTK Nav2前地图检查 =========="
echo "当前Nav2全局地图:"
if [ -f "$CURRENT_YAML" ]; then
  echo "  $CURRENT_YAML"
else
  echo "  尚未创建，将使用备用地图: $FALLBACK_YAML"
fi
echo
echo "最新候选建图结果:"
if [ -n "$CANDIDATE_YAML" ] && [ -f "$CANDIDATE_YAML" ]; then
  echo "  $CANDIDATE_YAML"
else
  echo "  未找到"
fi
echo

if [ -n "$CANDIDATE_YAML" ] && [ -f "$CANDIDATE_YAML" ]; then
  read -r -p "是否更新全局地图为最新候选地图？[y/N] " answer
  case "$answer" in
    y|Y|yes|YES)
      install_current_map "$CANDIDATE_YAML"
      echo "已更新当前全局地图: $CURRENT_YAML"
      ;;
    *)
      echo "保持当前全局地图不变。"
      ;;
  esac
else
  echo "没有候选地图，跳过更新询问。"
fi

MAP_TO_USE="$CURRENT_YAML"
if [ ! -f "$MAP_TO_USE" ]; then
  MAP_TO_USE="$FALLBACK_YAML"
fi

if [ ! -f "$MAP_TO_USE" ]; then
  echo "错误: 没有可用地图，无法启动Nav2: $MAP_TO_USE" >&2
  exit 1
fi

echo
echo "启动Nav2，使用地图:"
echo "  $MAP_TO_USE"
echo

exec ros2 launch wheeltec_nav2 wheeltec_rtk_nav2.launch.py map:="$MAP_TO_USE" "$@"
