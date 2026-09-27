#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法:
  mark_rtk_place.sh --site 工作地点 --name 语义地点名

说明:
  户外RTK fixed语义地点标记命令。
  该命令强制使用 /gps/fix_filtered + 当前地图 rtk_map_transform.yaml 计算map坐标，
  不依赖RViz中的机器人模型位置，也不需要启动Nav2。
EOF
}

SITE_NAME=""
PLACE_NAME=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --site)
      SITE_NAME="${2:-}"
      shift 2
      ;;
    --name)
      PLACE_NAME="${2:-}"
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

if [ -z "$SITE_NAME" ] || [ -z "$PLACE_NAME" ]; then
  echo "错误: 必须指定 --site 和 --name" >&2
  usage >&2
  exit 2
fi

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash

/home/wheeltec/rtk_tools/myrobot_places.py record \
  --site "$SITE_NAME" \
  --name "$PLACE_NAME" \
  --pose-source rtk \
  --require-rtk
