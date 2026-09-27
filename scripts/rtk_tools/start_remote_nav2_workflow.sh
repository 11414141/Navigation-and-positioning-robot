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
  start_remote_nav2_workflow.sh --site 工作地点 --scan-mode none|realtime_fastlio|bag_offline

说明:
  远程控制专用非交互巡检工作流，对应本机 start_site_rtk_nav_workflow.sh 的
  “已有全局地图 -> 选择扫描建图模式 -> 启动Nav2巡检”分支。
EOF
}

SITE_NAME=""
SCAN_MODE="none"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --site)
      SITE_NAME="${2:-}"
      shift 2
      ;;
    --scan-mode)
      SCAN_MODE="${2:-none}"
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
  exit 2
fi

source_if_exists /home/wheeltec/rtk_tools/site_common.sh
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

SITE_DIR="$(site_dir_from_name "$SITE_NAME")"
ensure_site_dirs "$SITE_DIR"
MAP_FILE="$(site_current_yaml "$SITE_DIR")"
NAV2_MAP_FILE=""
FASTLIO_DELAY_PID=""
BAG_PID=""
RECORDED_BAG_PATH=""
FASTLIO_MAP_FILE=""
FASTLIO_START_DELAY="${FASTLIO_START_DELAY:-3}"

if [ ! -f "$MAP_FILE" ]; then
  echo "错误: 当前工作地点没有全局地图，无法启动Nav2巡检: $MAP_FILE" >&2
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

prepare_realtime_fastlio_session() {
  local session_id session_dir raw_dir
  session_id="$(date +%Y%m%d_%H%M%S)_nav_realtime"
  session_dir="$(site_fastlio_sessions_dir "$SITE_DIR")/$session_id"
  raw_dir="$session_dir/raw"
  mkdir -p "$raw_dir"
  FASTLIO_MAP_FILE="$raw_dir/fast_lio_full.pcd"
  export FASTLIO_MAP_FILE
}

run_realtime_fastlio() {
  echo "等待雷达/IMU稳定 ${FASTLIO_START_DELAY}s 后启动实时FAST-LIO2..."
  sleep "$FASTLIO_START_DELAY"
  echo "启动实时FAST-LIO2建图:"
  echo "  raw_pcd: $FASTLIO_MAP_FILE"
  ros2 launch fast_lio mapping.launch.py \
    config_file:="${CONFIG_FILE:-mid360.yaml}" \
    map_file_path:="$FASTLIO_MAP_FILE" \
    convert_livox_points:=false \
    rviz:="${FASTLIO_RVIZ:-false}"
}

record_bag_during_nav2() {
  local date_str time_str bag_root
  date_str="$(date +%Y%m%d)"
  time_str="$(date +%Y%m%d_%H%M%S)"
  bag_root="$(site_bags_dir "$SITE_DIR")/rtk_nav_${date_str}"
  RECORDED_BAG_PATH="${bag_root}/rtk_nav_full_${time_str}"
  mkdir -p "$bag_root"
  export RECORDED_BAG_PATH
  echo "启动bag录制:"
  echo "  $RECORDED_BAG_PATH"
  OUT_PATH="$RECORDED_BAG_PATH" SITE_DIR="$SITE_DIR" /home/wheeltec/rtk_tools/record_rtk_nav_bag.sh full &
  BAG_PID=$!
}

cleanup() {
  if [ -n "${FASTLIO_DELAY_PID:-}" ]; then
    kill -INT "$FASTLIO_DELAY_PID" 2>/dev/null || true
    wait "$FASTLIO_DELAY_PID" 2>/dev/null || true
    if [ -n "${FASTLIO_MAP_FILE:-}" ] && [ -f "$FASTLIO_MAP_FILE" ]; then
      echo "实时FAST-LIO2已停止，生成候选2D地图..."
      SITE_DIR="$SITE_DIR" /home/wheeltec/rtk_tools/build_nav2_map_from_fastlio_pcd.sh "$FASTLIO_MAP_FILE"
    fi
  fi
  if [ -n "${BAG_PID:-}" ]; then
    echo "停止bag录制..."
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
    if [ -d "${RECORDED_BAG_PATH:-}" ]; then
      echo "开始用bag离线FAST-LIO2建图..."
      SITE_DIR="$SITE_DIR" /home/wheeltec/rtk_tools/run_fastlio_offline_from_bag.sh "$RECORDED_BAG_PATH"
    fi
  fi
}
trap cleanup EXIT

case "$SCAN_MODE" in
  none)
    echo "扫描建图模式: 不更新扫描建图"
    ;;
  realtime_fastlio)
    prepare_realtime_fastlio_session
    run_realtime_fastlio &
    FASTLIO_DELAY_PID=$!
    ;;
  bag_offline)
    record_bag_during_nav2
    ;;
  *)
    echo "错误: 无效扫描建图模式: $SCAN_MODE" >&2
    exit 2
    ;;
esac

echo "========== 启动Nav2巡检 =========="
echo "site: $SITE_NAME"
echo "map : $MAP_FILE"
echo "nav2: $NAV2_MAP_FILE"
echo
ros2 launch wheeltec_nav2 wheeltec_rtk_nav2.launch.py map:="$NAV2_MAP_FILE"
