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

usage() {
  cat <<'EOF'
用法:
  start_amcl_nav2_for_site.sh --site 工作地点

说明:
  独立 AMCL Nav2 启动脚本。
  使用工作地点 maps/nav2_current/current.yaml。
  不启动 RTK、NTRIP、RTK map TF，也不依赖 rtk_map_transform.yaml。
  启动后需要在 RViz 使用 2D Pose Estimate 手动初始化位姿。

环境变量:
  AMCL_NAV2_PARAMS=/path/to/params.yaml  覆盖 AMCL Nav2 参数文件
  START_LIVOX=true                       是否启动 Livox 和 /scan 转换
EOF
}

SITE_NAME=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --site)
      SITE_NAME="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "错误: 未知参数 $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$SITE_NAME" ]; then
  echo "错误: 必须指定 --site" >&2
  usage >&2
  exit 2
fi

source_if_exists /home/wheeltec/rtk_tools/site_common.sh
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

SITE_DIR="$(site_dir_from_name "$SITE_NAME")"
MAP_FILE="$(site_current_yaml "$SITE_DIR")"
AMCL_NAV2_PARAMS="${AMCL_NAV2_PARAMS:-/home/wheeltec/rtk_tools/config/amcl_nav2_params.yaml}"
START_LIVOX="${START_LIVOX:-true}"

if [ ! -f "$MAP_FILE" ]; then
  echo "错误: 工作地点没有当前全局地图: $MAP_FILE" >&2
  exit 1
fi

if [ ! -f "$AMCL_NAV2_PARAMS" ]; then
  echo "错误: AMCL Nav2 参数文件不存在: $AMCL_NAV2_PARAMS" >&2
  exit 1
fi

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
  conflicts="$(
    printf '%s\n' "$node_list" |
      grep -E '^/(map_server|amcl|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|nav2_container|lifecycle_manager|lifecycle_manager_navigation|lifecycle_manager_localization|lifecycle_manager_rtk_navigation|myrobot_rtk_map_tf_publisher|zed_f9p_ntrip_node|rtk_quality_filter|navsat_transform|ekf_filter_node|ekf_filter_node_map|ekf_filter_node_odom|wheeltec_robot|wheeltec_robot_node|robot_state_publisher|joint_state_publisher|waypoint_cycle|pointcloud_to_laserscan|livox_custom_to_pointcloud2|livox_custom_to_pointcloud2_amcl|livox_lidar_publisher|static_transform_publisher_livox|static_transform_publisher_livox_amcl)$' || true
  )"
  if [ -n "$conflicts" ]; then
    echo "错误: 检测到已有 Nav2/RTK/雷达相关节点在运行，不能同时启动 AMCL Nav2。" >&2
    echo "$conflicts" | sed 's/^/  /' >&2
    echo "请先停止当前导航/RTK任务后再启动 AMCL Nav2。" >&2
    exit 1
  fi
}

resolve_image_path() {
  local yaml_file="$1"
  local image_line
  image_line="$(grep -E '^image:' "$yaml_file" | head -1 | sed 's/^image:[[:space:]]*//')"
  if [ -z "$image_line" ]; then
    echo "错误: 地图yaml缺少 image 字段: $yaml_file" >&2
    return 1
  fi
  if [[ "$image_line" = /* ]]; then
    printf '%s\n' "$image_line"
  else
    printf '%s/%s\n' "$(dirname "$yaml_file")" "$image_line"
  fi
}

prepare_ascii_nav2_map() {
  local source_yaml="$1"
  local site_name="$2"
  local image_file safe_id ascii_dir
  image_file="$(resolve_image_path "$source_yaml")"
  if [ ! -f "$image_file" ]; then
    echo "错误: 地图图像不存在: $image_file" >&2
    return 1
  fi

  safe_id="$(printf '%s' "${site_name}_amcl" | sha1sum | awk '{print $1}')"
  ascii_dir="/home/wheeltec/myrobot_nav2_maps/amcl_site_${safe_id}"
  mkdir -p "$ascii_dir"
  cp "$image_file" "$ascii_dir/current.pgm"
  cp "$source_yaml" "$ascii_dir/current.yaml"
  sed -i 's|^image:.*|image: current.pgm|' "$ascii_dir/current.yaml"
  printf '%s/current.yaml\n' "$ascii_dir"
}

check_existing_nodes
NAV2_MAP_FILE="$(prepare_ascii_nav2_map "$MAP_FILE" "$SITE_NAME")"

echo "启动 AMCL Nav2:"
echo "  site       : $SITE_NAME"
echo "  source map : $MAP_FILE"
echo "  nav2 map   : $NAV2_MAP_FILE"
echo "  params     : $AMCL_NAV2_PARAMS"
echo "  start_livox: $START_LIVOX"
echo
echo "注意: AMCL 模式不使用 RTK 自动定位。启动后请在 RViz 使用 2D Pose Estimate。"
echo

ros2 launch /home/wheeltec/rtk_tools/myrobot_amcl_nav2.launch.py \
  map:="$NAV2_MAP_FILE" \
  params_file:="$AMCL_NAV2_PARAMS" \
  start_livox:="$START_LIVOX"
