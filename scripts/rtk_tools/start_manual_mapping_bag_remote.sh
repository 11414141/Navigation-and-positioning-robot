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

source_if_exists /home/wheeltec/rtk_tools/site_common.sh
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

if [ -n "$SITE_NAME" ]; then
  SITE_DIR="$(site_dir_from_name "$SITE_NAME")"
  ensure_site_dirs "$SITE_DIR"
  export SITE_DIR
fi

date_str="$(date +%Y%m%d)"
time_str="$(date +%Y%m%d_%H%M%S)"
if [ -n "${SITE_DIR:-}" ]; then
  bag_root="$(site_bags_dir "$SITE_DIR")/manual_mapping_${date_str}"
else
  bag_root="/home/wheeltec/bags/manual_mapping_${date_str}"
fi
out_path="${bag_root}/fastlio_manual_${time_str}"
mkdir -p "$bag_root"

echo "启动远程手动录bag建图:"
echo "  site: ${SITE_NAME:-未指定}"
echo "  bag : $out_path"
echo

set +e
OUT_PATH="$out_path" SITE_DIR="${SITE_DIR:-}" /home/wheeltec/rtk_tools/record_rtk_nav_bag.sh fastlio &
record_pid=$!
if [ -n "$SITE_NAME" ]; then
  (
    sleep "${AUTO_HOME_DELAY:-5}"
    echo "自动标定起始点为充电站:"
    /home/wheeltec/rtk_tools/myrobot_places.py record-origin --site "$SITE_NAME" --name "充电站" || true
  ) &
fi
wait "$record_pid"
record_status=$?
set -e

if [ "$record_status" -ne 0 ] && [ "$record_status" -ne 130 ] && [ "$record_status" -ne 143 ]; then
  echo "错误: bag录制异常退出，状态码 $record_status。跳过离线建图。" >&2
  exit "$record_status"
fi

if [ -d "$out_path" ]; then
  echo "开始离线FAST-LIO2建图并生成2D候选地图:"
  SITE_DIR="${SITE_DIR:-}" /home/wheeltec/rtk_tools/run_fastlio_offline_from_bag.sh "$out_path"
  if [ -n "${SITE_DIR:-}" ] && [ ! -f "$(site_current_yaml "$SITE_DIR")" ]; then
    echo
    echo "当前地点尚无Nav2全局地图，自动将首次离线建图候选地图更新为全局地图..."
    /home/wheeltec/rtk_tools/myrobot_site_manager.py update-candidate --site "$SITE_NAME"
  fi
else
  echo "错误: 未找到录制后的bag目录: $out_path" >&2
  exit 1
fi
