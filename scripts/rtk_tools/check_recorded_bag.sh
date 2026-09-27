#!/usr/bin/env bash
set -o pipefail

RED=$'\033[31m'
GREEN=$'\033[32m'
YELLOW=$'\033[33m'
BLUE=$'\033[34m'
RESET=$'\033[0m'

ok() { echo "${GREEN}[OK]${RESET} $*"; }
warn() { echo "${YELLOW}[WARN]${RESET} $*"; }
fail() { echo "${RED}[FAIL]${RESET} $*"; }
info() { echo "${BLUE}[INFO]${RESET} $*"; }

MODE="${1:-}"
BAG_PATH="${2:-}"

if [ "$MODE" != "fastlio" ] && [ "$MODE" != "glim" ]; then
  echo "用法:"
  echo "  $0 fastlio /path/to/fastlio_custommsg_bag"
  echo "  $0 glim    /path/to/glim_pointcloud2_bag"
  echo
  echo "如果 BAG_PATH 填 latest，会自动找当天最新对应 bag："
  echo "  $0 fastlio latest"
  echo "  $0 glim latest"
  exit 2
fi

if [ -z "$BAG_PATH" ]; then
  echo "缺少 BAG_PATH。用法: $0 $MODE /path/to/bag 或 $0 $MODE latest"
  exit 2
fi

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
    ok "sourced $setup_file"
  else
    warn "missing setup file: $setup_file"
  fi
}

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash

if [ "$BAG_PATH" = "latest" ]; then
  case "$MODE" in
    fastlio)
      BAG_PATH="$(ls -td /home/wheeltec/bags/rtk_field_$(date +%Y%m%d)/fastlio_custommsg_* 2>/dev/null | head -1)"
      ;;
    glim)
      BAG_PATH="$(ls -td /home/wheeltec/bags/rtk_field_$(date +%Y%m%d)/glim_pointcloud2_* 2>/dev/null | head -1)"
      ;;
  esac
fi

echo "========== 录完 bag 检查: $MODE =========="
date
echo "bag: ${BAG_PATH:-not found}"
echo

if [ -z "${BAG_PATH:-}" ] || [ ! -d "$BAG_PATH" ]; then
  fail "bag directory not found: ${BAG_PATH:-empty}"
  exit 1
fi

tmp_info="$(mktemp)"
if ros2 bag info "$BAG_PATH" >"$tmp_info" 2>/tmp/check_recorded_bag.err; then
  ok "ros2 bag info succeeded"
else
  fail "ros2 bag info failed"
  cat /tmp/check_recorded_bag.err 2>/dev/null || true
  rm -f "$tmp_info" /tmp/check_recorded_bag.err
  exit 1
fi

echo
info "bag 基本信息"
sed -n '1,80p' "$tmp_info"

required_topics=(
  "/livox/lidar"
  "/livox/imu"
  "/odom"
  "/odom_combined"
  "/imu/data_raw"
  "/gps/fix"
  "/nmea_sentence"
  "/gnss/gpgga"
  "/gps/vel"
  "/gps/nmea/vel"
  "/gps/pose"
  "/heading"
  "/heading_deg"
  "/time_reference"
  "/rtk/status"
  "/tf"
  "/tf_static"
)

echo
info "关键话题检查"
for topic in "${required_topics[@]}"; do
  if grep -q "Topic: $topic " "$tmp_info"; then
    ok "$topic exists"
  else
    fail "$topic missing"
  fi
done

echo
info "/livox/lidar 类型检查"
case "$MODE" in
  fastlio)
    if grep -q "Topic: /livox/lidar | Type: livox_ros_driver2/msg/CustomMsg" "$tmp_info"; then
      ok "FAST-LIO2 bag lidar type is CustomMsg"
    else
      fail "FAST-LIO2 bag expected /livox/lidar type livox_ros_driver2/msg/CustomMsg"
    fi
    ;;
  glim)
    if grep -q "Topic: /livox/lidar | Type: sensor_msgs/msg/PointCloud2" "$tmp_info"; then
      ok "GLIM bag lidar type is PointCloud2"
    else
      fail "GLIM bag expected /livox/lidar type sensor_msgs/msg/PointCloud2"
    fi
    ;;
esac

echo
info "消息数量粗检"
for topic in /livox/lidar /livox/imu /gps/fix /nmea_sentence /rtk/status /tf; do
  line="$(grep "Topic: $topic " "$tmp_info" || true)"
  if [ -n "$line" ]; then
    count="$(echo "$line" | sed -n 's/.*Count: \([0-9][0-9]*\).*/\1/p')"
    if [ -n "$count" ] && [ "$count" -gt 0 ]; then
      ok "$topic count=$count"
    else
      fail "$topic count=${count:-unknown}"
    fi
  fi
done

rm -f "$tmp_info" /tmp/check_recorded_bag.err
echo
echo "========== 检查完成 =========="
