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
MIN_FREE_GB="${MIN_FREE_GB:-20}"
RTK_READY_WAIT_SEC="${RTK_READY_WAIT_SEC:-180}"
RTK_READY_INTERVAL_SEC="${RTK_READY_INTERVAL_SEC:-5}"
RTK_READY_STABLE_COUNT="${RTK_READY_STABLE_COUNT:-3}"
CHECK_FAILED=0

MODE="${1:-}"
if [ "$#" -gt 0 ]; then
  shift
fi
REQUIRE_RTK_FIXED=false
while [ "$#" -gt 0 ]; do
  case "$1" in
    --require-rtk-fixed)
      REQUIRE_RTK_FIXED=true
      shift
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
done

if [ "$MODE" != "fastlio" ] && [ "$MODE" != "glim" ]; then
  echo "用法:"
  echo "  $0 fastlio [--require-rtk-fixed]   # 检查 /livox/lidar 是否为 livox_ros_driver2/msg/CustomMsg"
  echo "  $0 glim    [--require-rtk-fixed]   # 检查 /livox/lidar 是否为 sensor_msgs/msg/PointCloud2"
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

topic_exists() {
  local topic="$1"
  for _ in 1 2 3; do
    if ros2 topic info "$topic" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

topic_type_line() {
  ros2 topic list -t 2>/dev/null | grep -E "^$1 \\[" || true
}

check_topic() {
  local topic="$1"
  if topic_exists "$topic"; then
    ok "$(topic_type_line "$topic")"
  else
    fail "missing topic: $topic"
    CHECK_FAILED=1
  fi
}

check_hz() {
  local topic="$1"
  local duration="${2:-5}"
  if ! topic_exists "$topic"; then
    warn "skip hz for missing topic: $topic"
    return
  fi
  info "measuring hz: $topic (${duration}s)"
  timeout "$duration" ros2 topic hz "$topic" 2>/dev/null | tail -n 5 || warn "failed to measure hz: $topic"
}

extract_status_value() {
  local key="$1"
  local file="$2"
  awk -v target="$key" '
    $0 ~ "key: " target "$" { found=1; next }
    found && $1 == "value:" {
      sub(/^ *value: /, "", $0)
      gsub(/^'\''|'\''$/, "", $0)
      print $0
      exit
    }
  ' "$file"
}

read_rtk_status_summary() {
  local output_file="$1"
  timeout 5 ros2 topic echo /rtk/status --once >"$output_file" 2>/tmp/check_bag_ready_rtk.err
}

wait_for_strict_rtk_ready() {
  local deadline stable_count tmp_status
  deadline=$((SECONDS + RTK_READY_WAIT_SEC))
  stable_count=0
  tmp_status="$(mktemp)"

  info "strict RTK wait: waiting up to ${RTK_READY_WAIT_SEC}s for stable rtk_fixed (${RTK_READY_STABLE_COUNT} consecutive checks, interval ${RTK_READY_INTERVAL_SEC}s)"

  while [ "$SECONDS" -le "$deadline" ]; do
    if read_rtk_status_summary "$tmp_status"; then
      ntrip_connected="$(extract_status_value ntrip_connected "$tmp_status")"
      fix_quality="$(extract_status_value fix_quality "$tmp_status")"
      fix_quality_text="$(extract_status_value fix_quality_text "$tmp_status")"
      num_sats="$(extract_status_value num_sats "$tmp_status")"
      hdop="$(extract_status_value hdop "$tmp_status")"
      total_rtcm_bytes="$(extract_status_value total_rtcm_bytes "$tmp_status")"
      echo "ntrip_connected: ${ntrip_connected:-unknown}"
      echo "fix_quality: ${fix_quality:-unknown} (${fix_quality_text:-unknown})"
      echo "num_sats: ${num_sats:-unknown}"
      echo "hdop: ${hdop:-unknown}"
      echo "total_rtcm_bytes: ${total_rtcm_bytes:-unknown}"

      if [ "$ntrip_connected" = "True" ] && [ "$fix_quality" = "4" ]; then
        if timeout 5 ros2 topic echo /gps/fix_filtered --once >/tmp/check_bag_ready_fix_filtered.out 2>/dev/null; then
          stable_count=$((stable_count + 1))
          ok "strict RTK check: rtk_fixed and /gps/fix_filtered publishing (${stable_count}/${RTK_READY_STABLE_COUNT})"
          if [ "$stable_count" -ge "$RTK_READY_STABLE_COUNT" ]; then
            rm -f "$tmp_status" /tmp/check_bag_ready_rtk.err /tmp/check_bag_ready_fix_filtered.out
            return 0
          fi
        else
          stable_count=0
          warn "strict RTK wait: rtk_fixed seen, but /gps/fix_filtered has no message yet"
        fi
      else
        stable_count=0
        warn "strict RTK wait: not ready yet, expected ntrip_connected=True and fix_quality=4"
      fi
    else
      stable_count=0
      warn "strict RTK wait: failed to read /rtk/status"
      cat /tmp/check_bag_ready_rtk.err 2>/dev/null || true
    fi

    if [ "$SECONDS" -lt "$deadline" ]; then
      sleep "$RTK_READY_INTERVAL_SEC"
    fi
  done

  rm -f "$tmp_status" /tmp/check_bag_ready_rtk.err /tmp/check_bag_ready_fix_filtered.out
  return 1
}

echo "========== 录包前检查: $MODE =========="
date
echo

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash

echo
info "磁盘空间"
df -h /home/wheeltec /media/wheeltec/carrot_disk 2>/dev/null || df -h /home/wheeltec
free_gb="$(df -BG /home/wheeltec 2>/dev/null | awk 'NR==2 {gsub(/G/,"",$4); print $4}')"
if [ -n "${free_gb:-}" ]; then
  if [ "$free_gb" -ge "$MIN_FREE_GB" ]; then
    ok "/home/wheeltec free space: ${free_gb}G >= ${MIN_FREE_GB}G"
  else
    fail "/home/wheeltec free space: ${free_gb}G < ${MIN_FREE_GB}G"
    CHECK_FAILED=1
  fi
fi

echo
info "串口占用检查"
if [ -e /dev/wheeltec_gnss ]; then
  if command -v lsof >/dev/null 2>&1; then
    lsof_out="$(lsof /dev/wheeltec_gnss 2>/dev/null || true)"
    if echo "$lsof_out" | grep -Eq "zed_f9p_ntrip|zed_f9p_n"; then
      ok "/dev/wheeltec_gnss is used by zed_f9p_ntrip_node"
    elif [ -n "$lsof_out" ]; then
      warn "/dev/wheeltec_gnss is used by other process:"
      echo "$lsof_out"
    else
      warn "/dev/wheeltec_gnss is not opened by any process"
    fi
  else
    warn "lsof not installed; skip serial owner check"
  fi
else
  fail "/dev/wheeltec_gnss does not exist"
  CHECK_FAILED=1
fi

echo
info "关键话题存在性"
for topic in \
  /livox/lidar \
  /livox/imu \
  /odom \
  /odom_combined \
  /imu/data_raw \
  /gps/fix \
  /nmea_sentence \
  /gnss/gpgga \
  /gps/vel \
  /gps/nmea/vel \
  /gps/pose \
  /heading \
  /heading_deg \
  /time_reference \
  /rtk/status \
  /tf \
  /tf_static
do
  check_topic "$topic"
done

echo
info "Livox 雷达类型检查"
livox_line="$(topic_type_line /livox/lidar)"
echo "${livox_line:-/livox/lidar not found}"
case "$MODE" in
  fastlio)
    if echo "$livox_line" | grep -q "livox_ros_driver2/msg/CustomMsg"; then
      ok "FAST-LIO2 CustomMsg 模式正确"
    else
      fail "FAST-LIO2 需要 /livox/lidar [livox_ros_driver2/msg/CustomMsg]"
      CHECK_FAILED=1
    fi
    ;;
  glim)
    if echo "$livox_line" | grep -q "sensor_msgs/msg/PointCloud2"; then
      ok "GLIM PointCloud2 模式正确"
    else
      fail "GLIM 需要 /livox/lidar [sensor_msgs/msg/PointCloud2]"
      CHECK_FAILED=1
    fi
    ;;
esac

echo
info "发布者检查"
ros2 topic info /livox/lidar -v 2>/dev/null | sed -n '1,80p' || warn "failed to inspect /livox/lidar"
pub_count="$(ros2 topic info /livox/lidar -v 2>/dev/null | awk '/Publisher count:/ {print $3; exit}')"
if [ "${pub_count:-0}" = "1" ]; then
  ok "/livox/lidar publisher count is 1"
else
  fail "/livox/lidar publisher count is ${pub_count:-unknown}; expected 1"
  CHECK_FAILED=1
fi

echo
info "GNSS 发布者检查"
if ros2 node list 2>/dev/null | grep -qx "/zed_f9p_ntrip_node"; then
  ok "/zed_f9p_ntrip_node is running"
else
  fail "/zed_f9p_ntrip_node is not running"
  CHECK_FAILED=1
fi
ros2 topic info /rtk/status -v 2>/dev/null | sed -n '1,80p' || warn "failed to inspect /rtk/status"

echo
info "频率粗检"
check_hz /livox/lidar 6
check_hz /livox/imu 6

echo
info "RTK 状态摘要"
tmp_status="$(mktemp)"
if [ "$REQUIRE_RTK_FIXED" = "true" ]; then
  if wait_for_strict_rtk_ready; then
    ok "strict RTK check: stable rtk_fixed is ready"
  else
    fail "strict RTK check: RTK did not become stable rtk_fixed within ${RTK_READY_WAIT_SEC}s"
    CHECK_FAILED=1
  fi
elif read_rtk_status_summary "$tmp_status"; then
  ntrip_connected="$(extract_status_value ntrip_connected "$tmp_status")"
  fix_quality="$(extract_status_value fix_quality "$tmp_status")"
  fix_quality_text="$(extract_status_value fix_quality_text "$tmp_status")"
  num_sats="$(extract_status_value num_sats "$tmp_status")"
  hdop="$(extract_status_value hdop "$tmp_status")"
  total_rtcm_bytes="$(extract_status_value total_rtcm_bytes "$tmp_status")"
  echo "ntrip_connected: ${ntrip_connected:-unknown}"
  echo "fix_quality: ${fix_quality:-unknown} (${fix_quality_text:-unknown})"
  echo "num_sats: ${num_sats:-unknown}"
  echo "hdop: ${hdop:-unknown}"
  echo "total_rtcm_bytes: ${total_rtcm_bytes:-unknown}"
else
  fail "failed to read /rtk/status"
  cat /tmp/check_bag_ready_rtk.err 2>/dev/null || true
  CHECK_FAILED=1
fi
rm -f "$tmp_status" /tmp/check_bag_ready_rtk.err

echo
info "/gps/fix 一帧"
timeout 5 ros2 topic echo /gps/fix --once 2>/dev/null || {
  warn "failed to read /gps/fix"
  CHECK_FAILED=1
}

if [ "$REQUIRE_RTK_FIXED" = "true" ]; then
  echo
  info "/gps/fix_filtered 一帧"
  if timeout 5 ros2 topic echo /gps/fix_filtered --once 2>/dev/null; then
    ok "/gps/fix_filtered is publishing after strict RTK wait"
  else
    fail "strict RTK check: /gps/fix_filtered has no message"
    CHECK_FAILED=1
  fi
fi

echo
info "建议录制命令包含的话题"
cat <<'TOPICS'
/livox/lidar
/livox/imu
/odom
/odom_combined
/imu/data_raw
/gps/fix
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
TOPICS

echo
echo "========== 检查完成 =========="
if [ "$CHECK_FAILED" -ne 0 ]; then
  exit 1
fi
