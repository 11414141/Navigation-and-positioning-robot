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

echo "========== GNSS / RTK 状态检查 =========="
date
echo

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash

echo
info "设备检查"
if lsusb 2>/dev/null | grep -qi "1546:01a9"; then
  ok "u-blox ZED-F9P USB device found: 1546:01a9"
else
  fail "u-blox ZED-F9P USB device not found in lsusb"
fi

if [ -e /dev/wheeltec_gnss ]; then
  ok "/dev/wheeltec_gnss exists: $(readlink -f /dev/wheeltec_gnss)"
else
  fail "/dev/wheeltec_gnss does not exist"
fi

if [ -e /dev/ttyACM0 ]; then
  ok "/dev/ttyACM0 exists"
else
  warn "/dev/ttyACM0 does not exist"
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
fi

echo
info "磁盘空间检查"
free_gb="$(df -BG /home/wheeltec 2>/dev/null | awk 'NR==2 {gsub(/G/,"",$4); print $4}')"
if [ -n "${free_gb:-}" ]; then
  if [ "$free_gb" -ge "$MIN_FREE_GB" ]; then
    ok "/home/wheeltec free space: ${free_gb}G >= ${MIN_FREE_GB}G"
  else
    fail "/home/wheeltec free space: ${free_gb}G < ${MIN_FREE_GB}G"
  fi
else
  warn "failed to read /home/wheeltec free space"
fi

echo
info "关键话题检查"
for topic in /rtk/status /gps/fix /nmea_sentence /gnss/gpgga /gps/vel /gps/nmea/vel /gps/pose /heading /heading_deg /time_reference; do
  if topic_exists "$topic"; then
    ok "$(topic_type_line "$topic")"
  else
    fail "missing topic: $topic"
  fi
done

echo
info "GNSS 节点和话题发布者检查"
if ros2 node list 2>/dev/null | grep -qx "/zed_f9p_ntrip_node"; then
  ok "/zed_f9p_ntrip_node is running"
else
  fail "/zed_f9p_ntrip_node is not running"
fi
ros2 topic info /rtk/status -v 2>/dev/null | sed -n '1,80p' || warn "failed to inspect /rtk/status"

echo
info "/rtk/status 摘要"
tmp_status="$(mktemp)"
if timeout 5 ros2 topic echo /rtk/status --once >"$tmp_status" 2>/tmp/check_gnss_status.err; then
  ntrip_connected="$(extract_status_value ntrip_connected "$tmp_status")"
  fix_quality="$(extract_status_value fix_quality "$tmp_status")"
  fix_quality_text="$(extract_status_value fix_quality_text "$tmp_status")"
  num_sats="$(extract_status_value num_sats "$tmp_status")"
  hdop="$(extract_status_value hdop "$tmp_status")"
  total_rtcm_bytes="$(extract_status_value total_rtcm_bytes "$tmp_status")"
  seconds_since_gga="$(extract_status_value seconds_since_gga "$tmp_status")"
  seconds_since_rtcm="$(extract_status_value seconds_since_rtcm "$tmp_status")"
  last_ntrip_error="$(extract_status_value last_ntrip_error "$tmp_status")"

  echo "ntrip_connected: ${ntrip_connected:-unknown}"
  echo "fix_quality: ${fix_quality:-unknown} (${fix_quality_text:-unknown})"
  echo "num_sats: ${num_sats:-unknown}"
  echo "hdop: ${hdop:-unknown}"
  echo "total_rtcm_bytes: ${total_rtcm_bytes:-unknown}"
  echo "seconds_since_gga: ${seconds_since_gga:-unknown}"
  echo "seconds_since_rtcm: ${seconds_since_rtcm:-unknown}"
  echo "last_ntrip_error: ${last_ntrip_error:-}"

  if [ "$ntrip_connected" = "True" ]; then
    ok "NTRIP connected"
  else
    warn "NTRIP not connected"
  fi

  case "$fix_quality" in
    4) ok "RTK Fixed: 厘米级固定解目标达成" ;;
    5) warn "RTK Float: 可测试，但不是最终固定解" ;;
    1) warn "Single fix: 只有单点定位，不是厘米级 RTK" ;;
    2) warn "DGPS fix: 差分定位，但不是 RTK Fixed" ;;
    0) warn "No GNSS fix: 当前无定位，室内常见" ;;
    *) warn "Unknown fix_quality: ${fix_quality:-empty}" ;;
  esac
else
  fail "failed to read /rtk/status"
  cat /tmp/check_gnss_status.err 2>/dev/null || true
fi
rm -f "$tmp_status" /tmp/check_gnss_status.err

echo
info "/gps/fix 一帧"
timeout 5 ros2 topic echo /gps/fix --once 2>/dev/null || warn "failed to read /gps/fix"

echo
info "GGA 最近一帧"
timeout 8 bash -lc "ros2 topic echo /nmea_sentence 2>/dev/null | grep --line-buffered -m 1 GGA" || warn "no GGA sentence received"

echo
echo "========== 检查完成 =========="
