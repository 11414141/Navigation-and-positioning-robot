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
  run_fastlio_offline_from_bag.sh /path/to/rosbag_dir

环境变量可选:
  SITE_DIR=/home/wheeltec/field_sites/农场A
  CONFIG_FILE=mid360.yaml
  RVIZ=false
  BAG_RATE=0.5
  START_DELAY=10
  FASTLIO_STOP_TIMEOUT=60
  RECORD_TRAJECTORY=true
  SLICE_MODE=adaptive
  REL_Z_MIN=0.06
  REL_Z_MAX=1.00
  FILTER_NEAR_PATH=true
  PATH_CLEAR_RADIUS=0.45
  COLUMN_FILTER=true
  MAP_SPECKLE_FILTER=true
  AUTO_RTK_ANCHORS=true
  RTK_ANCHOR_MIN_COUNT=3
  RTK_ANCHOR_SPACING=5.0
  RTK_ANCHOR_MAX_TIME_DIFF=0.2
  RTK_ANCHOR_MAX_RMS=1.0
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

BAG_PATH="${1:-}"
if [ -z "$BAG_PATH" ] || [ ! -d "$BAG_PATH" ]; then
  echo "错误: bag目录不存在: ${BAG_PATH:-未指定}" >&2
  exit 1
fi

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

if [ -n "${SITE_DIR:-}" ]; then
  ensure_site_dirs "$SITE_DIR"
fi

SESSION_ID="$(date +%Y%m%d_%H%M%S)_offline"
if [ -n "${SITE_DIR:-}" ]; then
  SESSION_DIR="$(site_fastlio_sessions_dir "$SITE_DIR")/${SESSION_ID}"
else
  SESSION_DIR="/home/wheeltec/maps/fast_lio_sessions/${SESSION_ID}"
fi
RAW_DIR="$SESSION_DIR/raw"
TRAJECTORY_DIR="$SESSION_DIR/trajectory"
MAP_FILE="$RAW_DIR/fast_lio_full.pcd"
TRAJECTORY_CSV="$TRAJECTORY_DIR/fastlio_odometry.csv"
CONFIG_FILE="${CONFIG_FILE:-mid360.yaml}"
RVIZ="${RVIZ:-false}"
BAG_RATE="${BAG_RATE:-0.5}"
START_DELAY="${START_DELAY:-10}"
FASTLIO_STOP_TIMEOUT="${FASTLIO_STOP_TIMEOUT:-60}"
RECORD_TRAJECTORY="${RECORD_TRAJECTORY:-true}"
AUTO_RTK_ANCHORS="${AUTO_RTK_ANCHORS:-true}"
RTK_ANCHOR_MIN_COUNT="${RTK_ANCHOR_MIN_COUNT:-3}"
RTK_ANCHOR_SPACING="${RTK_ANCHOR_SPACING:-5.0}"
RTK_ANCHOR_MAX_TIME_DIFF="${RTK_ANCHOR_MAX_TIME_DIFF:-0.2}"
RTK_ANCHOR_MAX_RMS="${RTK_ANCHOR_MAX_RMS:-1.0}"
RTK_ANCHOR_MAX_COUNT="${RTK_ANCHOR_MAX_COUNT:-30}"
RTK_ANCHOR_MAX_HORIZONTAL_STDDEV="${RTK_ANCHOR_MAX_HORIZONTAL_STDDEV:-0.2}"

mkdir -p "$RAW_DIR" "$TRAJECTORY_DIR"

echo "========== rosbag离线FAST-LIO2建图 =========="
echo "bag: $BAG_PATH"
echo "session_dir: $SESSION_DIR"
echo "raw_pcd: $MAP_FILE"
echo "config_file: $CONFIG_FILE"
echo "bag_rate: $BAG_RATE"
echo "start_delay: $START_DELAY"
echo "fastlio_stop_timeout: $FASTLIO_STOP_TIMEOUT"
echo "record_trajectory: $RECORD_TRAJECTORY"
echo "trajectory_csv: $TRAJECTORY_CSV"
echo "auto_rtk_anchors: $AUTO_RTK_ANCHORS"
echo

ros2 launch fast_lio mapping.launch.py \
  use_sim_time:=true \
  config_file:="$CONFIG_FILE" \
  map_file_path:="$MAP_FILE" \
  convert_livox_points:=false \
  rviz:="$RVIZ" &
FASTLIO_PID=$!

TRAJECTORY_PID=""
if [ "$RECORD_TRAJECTORY" = "true" ]; then
  /home/wheeltec/rtk_tools/record_fastlio_trajectory.py --output "$TRAJECTORY_CSV" --topic /Odometry &
  TRAJECTORY_PID=$!
fi

sleep "$START_DELAY"

set +e
ros2 bag play "$BAG_PATH" --clock --rate "$BAG_RATE"
BAG_STATUS=$?
set -e

echo "bag播放结束，停止FAST-LIO2以触发最终PCD保存..."
if [ -n "$TRAJECTORY_PID" ]; then
  echo "停止FAST-LIO2轨迹记录..."
  kill -INT "$TRAJECTORY_PID" 2>/dev/null || true
  wait "$TRAJECTORY_PID" 2>/dev/null || true
fi
kill -INT "$FASTLIO_PID" 2>/dev/null || true
stop_deadline=$((SECONDS + FASTLIO_STOP_TIMEOUT))
while kill -0 "$FASTLIO_PID" 2>/dev/null && [ "$SECONDS" -lt "$stop_deadline" ]; do
  sleep 1
done
if kill -0 "$FASTLIO_PID" 2>/dev/null; then
  echo "警告: FAST-LIO2在 ${FASTLIO_STOP_TIMEOUT}s 内未退出，发送SIGTERM继续后处理..."
  kill -TERM "$FASTLIO_PID" 2>/dev/null || true
  sleep 3
fi
if kill -0 "$FASTLIO_PID" 2>/dev/null; then
  echo "警告: FAST-LIO2仍未退出，发送SIGKILL继续后处理..."
  kill -KILL "$FASTLIO_PID" 2>/dev/null || true
fi
wait "$FASTLIO_PID" 2>/dev/null || true

if [ "$BAG_STATUS" -ne 0 ]; then
  echo "警告: ros2 bag play退出状态码为 $BAG_STATUS"
fi

if [ ! -f "$MAP_FILE" ]; then
  echo "错误: 离线FAST-LIO2未生成完整3D地图: $MAP_FILE" >&2
  exit 1
fi

echo "开始自动生成Nav2候选2D地图..."
FASTLIO_TRAJECTORY_CSV="${FASTLIO_TRAJECTORY_CSV:-$TRAJECTORY_CSV}" /home/wheeltec/rtk_tools/build_nav2_map_from_fastlio_pcd.sh "$MAP_FILE"

MAP_YAML="$SESSION_DIR/nav2/fast_lio_2d.yaml"
if [ "$AUTO_RTK_ANCHORS" = "true" ]; then
  if [ "$RECORD_TRAJECTORY" != "true" ] || [ ! -f "$TRAJECTORY_CSV" ]; then
    echo "警告: 未找到FAST-LIO轨迹CSV，跳过自动RTK锚点生成: $TRAJECTORY_CSV"
  elif [ ! -f "$MAP_YAML" ]; then
    echo "警告: 未找到候选Nav2地图，跳过自动RTK锚点生成: $MAP_YAML"
  else
    echo
    echo "开始自动采样RTK锚点并计算地图RTK变换..."
    set +e
    /home/wheeltec/rtk_tools/auto_sample_rtk_anchors_from_bag.py \
      --site "${SITE_NAME:-${SITE_DIR##*/}}" \
      --bag "$BAG_PATH" \
      --trajectory-csv "$TRAJECTORY_CSV" \
      --map-yaml "$MAP_YAML" \
      --output-dir "$SESSION_DIR/nav2" \
      --min-spacing "$RTK_ANCHOR_SPACING" \
      --max-time-diff "$RTK_ANCHOR_MAX_TIME_DIFF" \
      --min-count "$RTK_ANCHOR_MIN_COUNT" \
      --max-count "$RTK_ANCHOR_MAX_COUNT" \
      --max-horizontal-stddev "$RTK_ANCHOR_MAX_HORIZONTAL_STDDEV" \
      --max-rms "$RTK_ANCHOR_MAX_RMS"
    anchor_status=$?
    set -e
    if [ "$anchor_status" -ne 0 ]; then
      echo "警告: 自动RTK锚点生成失败，已保留候选地图，可稍后手动处理锚点。"
    fi
  fi
fi
