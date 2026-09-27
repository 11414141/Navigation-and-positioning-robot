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
  start_nav2_for_site.sh --site 工作地点

说明:
  远程控制专用非交互Nav2启动脚本。
  只使用该工作地点 maps/nav2_current/current.yaml，不进行交互询问。

环境变量:
  AUTO_RTK_INITIALPOSE=false      Nav2启动后自动发布一次RTK初始位姿；当前RTK模式未启动AMCL，默认关闭
  RTK_INITIALPOSE_DELAY=12        启动Nav2后等待秒数
  RTK_INITIALPOSE_PERIOD=0        >0时周期发布RTK初始位姿，默认关闭
  RTK_INITIALPOSE_MAX_COUNT=1     周期发布最大次数，0为无限
  AUTO_RTK_MAP_TF=true            使用当前地图rtk_map_transform.yaml发布 map->odom_combined
  RTK_MAP_TF_YAW_SOURCE=transform map->odom_combined yaw来源：transform 或 zero
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
NAV2_MAP_FILE=""

if [ ! -f "$MAP_FILE" ]; then
  echo "错误: 工作地点没有当前全局地图: $MAP_FILE" >&2
  exit 1
fi

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

  safe_id="$(printf '%s' "$site_name" | sha1sum | awk '{print $1}')"
  ascii_dir="/home/wheeltec/myrobot_nav2_maps/site_${safe_id}"
  mkdir -p "$ascii_dir"
  cp "$image_file" "$ascii_dir/current.pgm"
  cp "$source_yaml" "$ascii_dir/current.yaml"
  sed -i 's|^image:.*|image: current.pgm|' "$ascii_dir/current.yaml"
  printf '%s/current.yaml\n' "$ascii_dir"
}

NAV2_MAP_FILE="$(prepare_ascii_nav2_map "$MAP_FILE" "$SITE_NAME")"
AUTO_RTK_INITIALPOSE="${AUTO_RTK_INITIALPOSE:-false}"
RTK_INITIALPOSE_DELAY="${RTK_INITIALPOSE_DELAY:-12}"
RTK_INITIALPOSE_PERIOD="${RTK_INITIALPOSE_PERIOD:-0}"
RTK_INITIALPOSE_MAX_COUNT="${RTK_INITIALPOSE_MAX_COUNT:-1}"
RTK_INITIALPOSE_XY_STDDEV="${RTK_INITIALPOSE_XY_STDDEV:-0.5}"
RTK_INITIALPOSE_YAW_STDDEV="${RTK_INITIALPOSE_YAW_STDDEV:-0.7}"
AUTO_RTK_MAP_TF="${AUTO_RTK_MAP_TF:-true}"
RTK_MAP_TF_DELAY="${RTK_MAP_TF_DELAY:-3}"
RTK_MAP_TF_RATE="${RTK_MAP_TF_RATE:-20}"
RTK_MAP_TF_TIME_OFFSET="${RTK_MAP_TF_TIME_OFFSET:-0.2}"
RTK_MAP_TF_POSITION_ALPHA="${RTK_MAP_TF_POSITION_ALPHA:-0.5}"
RTK_MAP_TF_YAW_SOURCE="${RTK_MAP_TF_YAW_SOURCE:-transform}"
RTK_MAP_TF_MAX_STDDEV="${RTK_MAP_TF_MAX_STDDEV:-0.3}"
RTK_MAP_TF_PID=""

echo "远程启动Nav2:"
echo "  site: $SITE_NAME"
echo "  map : $MAP_FILE"
echo "  nav2: $NAV2_MAP_FILE"
echo "  auto_rtk_initialpose: $AUTO_RTK_INITIALPOSE"
echo "  auto_rtk_map_tf: $AUTO_RTK_MAP_TF"
echo

ros2 launch wheeltec_nav2 wheeltec_rtk_nav2.launch.py map:="$NAV2_MAP_FILE" &
NAV2_PID=$!

cleanup() {
  if [ -n "$RTK_MAP_TF_PID" ]; then
    kill -INT "$RTK_MAP_TF_PID" 2>/dev/null || true
    wait "$RTK_MAP_TF_PID" 2>/dev/null || true
  fi
  kill -INT "$NAV2_PID" 2>/dev/null || true
  wait "$NAV2_PID" 2>/dev/null || true
}
trap cleanup INT TERM

if [ "$AUTO_RTK_MAP_TF" = "true" ]; then
  if [ ! -f "$SITE_DIR/maps/nav2_current/rtk_map_transform.yaml" ]; then
    echo "警告: 缺少当前地图RTK变换，不能启动RTK地图TF发布器: $SITE_DIR/maps/nav2_current/rtk_map_transform.yaml"
  else
    (
      sleep "$RTK_MAP_TF_DELAY"
      exec /home/wheeltec/rtk_tools/publish_rtk_map_tf.py \
        --site "$SITE_NAME" \
        --publish-rate "$RTK_MAP_TF_RATE" \
        --transform-time-offset "$RTK_MAP_TF_TIME_OFFSET" \
        --position-alpha "$RTK_MAP_TF_POSITION_ALPHA" \
        --yaw-source "$RTK_MAP_TF_YAW_SOURCE" \
        --max-horizontal-stddev "$RTK_MAP_TF_MAX_STDDEV"
    ) &
    RTK_MAP_TF_PID=$!
  fi
fi

if [ "$AUTO_RTK_INITIALPOSE" = "true" ]; then
  (
    sleep "$RTK_INITIALPOSE_DELAY"
    if [ "$RTK_INITIALPOSE_PERIOD" != "0" ] && [ "$RTK_INITIALPOSE_PERIOD" != "0.0" ]; then
      /home/wheeltec/rtk_tools/publish_rtk_initialpose.py \
        --site "$SITE_NAME" \
        --xy-stddev "$RTK_INITIALPOSE_XY_STDDEV" \
        --yaw-stddev "$RTK_INITIALPOSE_YAW_STDDEV" \
        --period "$RTK_INITIALPOSE_PERIOD" \
        --max-count "$RTK_INITIALPOSE_MAX_COUNT" || \
        echo "警告: RTK周期重定位initialpose发布失败。"
    else
      /home/wheeltec/rtk_tools/publish_rtk_initialpose.py \
        --site "$SITE_NAME" \
        --xy-stddev "$RTK_INITIALPOSE_XY_STDDEV" \
        --yaw-stddev "$RTK_INITIALPOSE_YAW_STDDEV" || \
        echo "警告: RTK initialpose发布失败，可手动使用RViz 2D Pose Estimate。"
    fi
  ) &
fi

wait "$NAV2_PID"
