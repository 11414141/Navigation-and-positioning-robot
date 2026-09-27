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

usage() {
  cat <<'EOF'
用法:
  start_fastlio_mapping_auto_2d.sh

环境变量可选:
  RVIZ=true|false
  CONFIG_FILE=mid360.yaml
  Z_MIN=0.03
  Z_MAX=1.0
  VOXEL_SIZE=0.0
  RADIUS=0.1
  MIN_NEIGHBORS=4
  MAP_RESOLUTION=0.03
  FASTLIO_START_DELAY=3

流程:
  1. 启动 FAST-LIO2 建图并保存完整3D PCD
  2. 按 Ctrl+C 结束建图
  3. 自动执行 3D PCD -> 切片 PCD -> Nav2 2D地图
  4. 只生成候选地图，不自动替换当前Nav2全局地图
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

if [ -n "${SITE_DIR:-}" ]; then
  ensure_site_dirs "$SITE_DIR"
fi

SESSION_ID="$(date +%Y%m%d_%H%M%S)"
if [ -n "${SITE_DIR:-}" ]; then
  SESSION_DIR="$(site_fastlio_sessions_dir "$SITE_DIR")/${SESSION_ID}"
else
  SESSION_DIR="/home/wheeltec/maps/fast_lio_sessions/${SESSION_ID}"
fi
RAW_DIR="${SESSION_DIR}/raw"
MAP_FILE="${RAW_DIR}/fast_lio_full.pcd"
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"
RVIZ="${RVIZ:-true}"
FASTLIO_START_DELAY="${FASTLIO_START_DELAY:-3}"

mkdir -p "$RAW_DIR"

echo "========== FAST-LIO2 建图 + 自动2D地图后处理 =========="
echo "session_dir: $SESSION_DIR"
echo "raw_pcd: $MAP_FILE"
echo "config_file: $CONFIG_FILE"
echo "rviz: $RVIZ"
echo
echo "建图运行中请正常移动小车。结束建图时，在本终端按 Ctrl+C。"
echo "Ctrl+C 后会自动执行: 3D PCD -> 切片 -> 2D Nav2地图。"
echo
echo "等待雷达/IMU稳定 ${FASTLIO_START_DELAY}s 后启动FAST-LIO2..."
sleep "$FASTLIO_START_DELAY"
echo

set +e
ros2 launch fast_lio mapping.launch.py \
  config_file:="$CONFIG_FILE" \
  map_file_path:="$MAP_FILE" \
  rviz:="$RVIZ"
FASTLIO_STATUS=$?
set -e

if [ "$FASTLIO_STATUS" -ne 0 ] && [ "$FASTLIO_STATUS" -ne 130 ] && [ "$FASTLIO_STATUS" -ne 143 ]; then
  echo "错误: FAST-LIO2异常退出，状态码 $FASTLIO_STATUS。跳过2D地图后处理。" >&2
  exit "$FASTLIO_STATUS"
fi

if [ ! -f "$MAP_FILE" ]; then
  echo "错误: FAST-LIO2结束后未找到完整3D地图: $MAP_FILE" >&2
  echo "请检查 pcd_save.pcd_save_en 是否开启，以及 FAST-LIO2 是否收到雷达/IMU数据。" >&2
  exit 1
fi

echo
echo "FAST-LIO2已结束，开始自动生成Nav2候选2D地图。"
echo

/home/wheeltec/rtk_tools/build_nav2_map_from_fastlio_pcd.sh "$MAP_FILE"
