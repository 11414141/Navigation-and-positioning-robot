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
  build_nav2_map_from_fastlio_pcd.sh latest
  build_nav2_map_from_fastlio_pcd.sh /path/to/fast_lio_full.pcd

环境变量可选:
  SLICE_MODE=global|adaptive
  Z_MIN=0.08
  Z_MAX=1.50
  REL_Z_MIN=0.06
  REL_Z_MAX=1.00
  GROUND_GRID_SIZE=0.5
  GROUND_PERCENTILE=10
  GROUND_MIN_POINTS_PER_CELL=20
  FILTER_NEAR_PATH=true
  PATH_CLEAR_RADIUS=0.45
  PATH_CLEAR_REL_Z_MAX=1.00
  COLUMN_FILTER=true
  COLUMN_GRID_SIZE=0.03
  COLUMN_MIN_POINTS=8
  COLUMN_MIN_HEIGHT_THICKNESS=0.12
  COLUMN_Z_BIN_SIZE=0.04
  COLUMN_MIN_Z_BINS=3
  COLUMN_NEIGHBOR_SUPPORT=true
  COLUMN_NEIGHBOR_MIN_COLUMNS=2
  FASTLIO_TRAJECTORY_CSV=/path/to/fastlio_odometry.csv
  VOXEL_SIZE=0.0
  RADIUS=0.1
  MIN_NEIGHBORS=4
  MIN_POINTS_PER_CELL=4
  MAP_SPECKLE_FILTER=true
  MIN_COMPONENT_CELLS=4
  MIN_COMPONENT_LENGTH=0.12
  SPECKLE_CONNECTIVITY=8
  MAP_RESOLUTION=0.03
  PCD2PGM_TIMEOUT=60
EOF
}

find_latest_raw_pcd() {
  local sessions_dir="/home/wheeltec/maps/fast_lio_sessions"
  if [ -n "${SITE_DIR:-}" ]; then
    sessions_dir="$(site_fastlio_sessions_dir "$SITE_DIR")"
  fi
  if [ ! -d "$sessions_dir" ]; then
    return 0
  fi
  find "$sessions_dir" -path '*/raw/fast_lio_full.pcd' -type f \
    -printf '%T@ %p\n' 2>/dev/null | sort -nr | awk 'NR==1 {print $2}' || true
}

INPUT="${1:-latest}"
if [ "$INPUT" = "-h" ] || [ "$INPUT" = "--help" ]; then
  usage
  exit 0
fi

if [ "$INPUT" = "latest" ]; then
  INPUT="$(find_latest_raw_pcd)"
fi

if [ -z "${INPUT:-}" ] || [ ! -f "$INPUT" ]; then
  echo "错误: 找不到输入PCD: ${INPUT:-latest}" >&2
  exit 1
fi

if [ -n "${SITE_DIR:-}" ]; then
  ensure_site_dirs "$SITE_DIR"
fi

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

SLICE_MODE="${SLICE_MODE:-global}"
Z_MIN="${Z_MIN:-0.08}"
Z_MAX="${Z_MAX:-1.50}"
REL_Z_MIN="${REL_Z_MIN:-0.06}"
REL_Z_MAX="${REL_Z_MAX:-1.00}"
GROUND_GRID_SIZE="${GROUND_GRID_SIZE:-0.5}"
GROUND_PERCENTILE="${GROUND_PERCENTILE:-10}"
GROUND_MIN_POINTS_PER_CELL="${GROUND_MIN_POINTS_PER_CELL:-20}"
FILTER_NEAR_PATH="${FILTER_NEAR_PATH:-true}"
PATH_CLEAR_RADIUS="${PATH_CLEAR_RADIUS:-0.45}"
PATH_CLEAR_REL_Z_MAX="${PATH_CLEAR_REL_Z_MAX:-1.00}"
PATH_SAMPLE_STRIDE="${PATH_SAMPLE_STRIDE:-5}"
COLUMN_FILTER="${COLUMN_FILTER:-true}"
COLUMN_GRID_SIZE="${COLUMN_GRID_SIZE:-0.03}"
COLUMN_MIN_POINTS="${COLUMN_MIN_POINTS:-8}"
COLUMN_MIN_HEIGHT_THICKNESS="${COLUMN_MIN_HEIGHT_THICKNESS:-0.12}"
COLUMN_Z_BIN_SIZE="${COLUMN_Z_BIN_SIZE:-0.04}"
COLUMN_MIN_Z_BINS="${COLUMN_MIN_Z_BINS:-3}"
COLUMN_NEIGHBOR_SUPPORT="${COLUMN_NEIGHBOR_SUPPORT:-true}"
COLUMN_NEIGHBOR_MIN_COLUMNS="${COLUMN_NEIGHBOR_MIN_COLUMNS:-2}"
FASTLIO_TRAJECTORY_CSV="${FASTLIO_TRAJECTORY_CSV:-}"
VOXEL_SIZE="${VOXEL_SIZE:-0.0}"
RADIUS="${RADIUS:-0.1}"
MIN_NEIGHBORS="${MIN_NEIGHBORS:-4}"
MIN_POINTS_PER_CELL="${MIN_POINTS_PER_CELL:-4}"
MAP_SPECKLE_FILTER="${MAP_SPECKLE_FILTER:-true}"
MIN_COMPONENT_CELLS="${MIN_COMPONENT_CELLS:-4}"
MIN_COMPONENT_LENGTH="${MIN_COMPONENT_LENGTH:-0.12}"
SPECKLE_CONNECTIVITY="${SPECKLE_CONNECTIVITY:-8}"
MAP_RESOLUTION="${MAP_RESOLUTION:-0.03}"
PCD2PGM_TIMEOUT="${PCD2PGM_TIMEOUT:-60}"

RAW_DIR="$(dirname "$INPUT")"
SESSION_DIR="$(dirname "$RAW_DIR")"
if [ "$(basename "$RAW_DIR")" != "raw" ]; then
  if [ -n "${SITE_DIR:-}" ]; then
    SESSION_DIR="$(site_fastlio_sessions_dir "$SITE_DIR")/postprocess_$(date +%Y%m%d_%H%M%S)"
  else
    SESSION_DIR="/home/wheeltec/maps/fast_lio_sessions/postprocess_$(date +%Y%m%d_%H%M%S)"
  fi
fi

SLICED_DIR="$SESSION_DIR/sliced"
NAV2_DIR="$SESSION_DIR/nav2"
mkdir -p "$SLICED_DIR" "$NAV2_DIR" /home/wheeltec/maps

SLICED_PCD="$SLICED_DIR/fast_lio_sliced.pcd"
MAP_YAML="$NAV2_DIR/fast_lio_2d.yaml"
MAP_PGM="$NAV2_DIR/fast_lio_2d.pgm"
if [ -n "${SITE_DIR:-}" ]; then
  LATEST_FILE="$(site_latest_candidate_file "$SITE_DIR")"
else
  LATEST_FILE="/home/wheeltec/maps/nav2_latest_candidate.txt"
fi
mkdir -p "$(dirname "$LATEST_FILE")"

echo "========== FAST-LIO PCD -> Nav2 2D地图 =========="
echo "input_pcd: $INPUT"
echo "session_dir: $SESSION_DIR"
echo "slice_mode: $SLICE_MODE"
echo "z_range: [$Z_MIN, $Z_MAX]"
echo "relative_z_range: [$REL_Z_MIN, $REL_Z_MAX]"
echo "ground_filter: grid=$GROUND_GRID_SIZE percentile=$GROUND_PERCENTILE min_points=$GROUND_MIN_POINTS_PER_CELL"
echo "near_path_filter: enabled=$FILTER_NEAR_PATH radius=$PATH_CLEAR_RADIUS rel_z_max=$PATH_CLEAR_REL_Z_MAX trajectory=${FASTLIO_TRAJECTORY_CSV:-none}"
echo "column_filter: enabled=$COLUMN_FILTER grid=$COLUMN_GRID_SIZE min_points=$COLUMN_MIN_POINTS min_height_thickness=$COLUMN_MIN_HEIGHT_THICKNESS z_bin=$COLUMN_Z_BIN_SIZE min_z_bins=$COLUMN_MIN_Z_BINS neighbor_support=$COLUMN_NEIGHBOR_SUPPORT neighbor_min_columns=$COLUMN_NEIGHBOR_MIN_COLUMNS"
echo "voxel_size: $VOXEL_SIZE"
echo "radius_filter: radius=$RADIUS min_neighbors=$MIN_NEIGHBORS"
echo "min_points_per_cell: $MIN_POINTS_PER_CELL"
echo "map_speckle_filter: enabled=$MAP_SPECKLE_FILTER min_cells=$MIN_COMPONENT_CELLS min_length=$MIN_COMPONENT_LENGTH connectivity=$SPECKLE_CONNECTIVITY"
echo "map_resolution: $MAP_RESOLUTION"
echo "pcd2pgm_timeout: ${PCD2PGM_TIMEOUT}s"
echo

case "$SLICE_MODE" in
  global)
    ros2 run livox_nav_tools pcd_slice_filter \
      --input "$INPUT" \
      --output "$SLICED_PCD" \
      --z-min "$Z_MIN" \
      --z-max "$Z_MAX" \
      --voxel-size "$VOXEL_SIZE" \
      --radius "$RADIUS" \
      --min-neighbors "$MIN_NEIGHBORS"
    ;;
  adaptive)
    adaptive_args=(
      --input "$INPUT"
      --output "$SLICED_PCD"
      --grid-size "$GROUND_GRID_SIZE"
      --ground-percentile "$GROUND_PERCENTILE"
      --min-points-per-cell "$GROUND_MIN_POINTS_PER_CELL"
      --rel-z-min "$REL_Z_MIN"
      --rel-z-max "$REL_Z_MAX"
      --path-clear-radius "$PATH_CLEAR_RADIUS"
      --path-clear-rel-z-max "$PATH_CLEAR_REL_Z_MAX"
      --path-sample-stride "$PATH_SAMPLE_STRIDE"
      --column-grid-size "$COLUMN_GRID_SIZE"
      --column-min-points "$COLUMN_MIN_POINTS"
      --column-min-height-thickness "$COLUMN_MIN_HEIGHT_THICKNESS"
      --column-z-bin-size "$COLUMN_Z_BIN_SIZE"
      --column-min-z-bins "$COLUMN_MIN_Z_BINS"
      --column-neighbor-min-columns "$COLUMN_NEIGHBOR_MIN_COLUMNS"
    )
    if [ "$COLUMN_FILTER" = "true" ]; then
      adaptive_args+=(--column-filter)
    fi
    if [ "$COLUMN_NEIGHBOR_SUPPORT" = "true" ]; then
      adaptive_args+=(--column-neighbor-support)
    fi
    if [ "$FILTER_NEAR_PATH" = "true" ]; then
      if [ -z "$FASTLIO_TRAJECTORY_CSV" ] || [ ! -f "$FASTLIO_TRAJECTORY_CSV" ]; then
        echo "错误: FILTER_NEAR_PATH=true 但找不到FAST-LIO轨迹CSV: ${FASTLIO_TRAJECTORY_CSV:-未指定}" >&2
        exit 1
      fi
      adaptive_args+=(--filter-near-path --trajectory-csv "$FASTLIO_TRAJECTORY_CSV")
    fi
    /home/wheeltec/rtk_tools/adaptive_ground_slice_pcd.py "${adaptive_args[@]}"
    ;;
  *)
    echo "错误: 无效SLICE_MODE: $SLICE_MODE，应为 global 或 adaptive" >&2
    exit 2
    ;;
esac

set +e
PCD2PGM_Z_MIN="$Z_MIN"
PCD2PGM_Z_MAX="$Z_MAX"
if [ "$SLICE_MODE" = "adaptive" ]; then
  PCD2PGM_Z_MIN="-1000.0"
  PCD2PGM_Z_MAX="1000.0"
fi
timeout "${PCD2PGM_TIMEOUT}s" ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p pcd_file:="$SLICED_PCD" \
  -p flag_pass_through:=false \
  -p thre_z_min:="$PCD2PGM_Z_MIN" \
  -p thre_z_max:="$PCD2PGM_Z_MAX" \
  -p thre_radius:="$RADIUS" \
  -p thres_point_count:="$MIN_NEIGHBORS" \
  -p min_points_per_cell:="$MIN_POINTS_PER_CELL" \
  -p map_resolution:="$MAP_RESOLUTION" \
  -p save_to_file:=true \
  -p output_map_yaml:="$MAP_YAML" \
  -p output_map_image:="$MAP_PGM"
PCD2PGM_STATUS=$?
set -e

if [ "$PCD2PGM_STATUS" -eq 124 ]; then
  if [ -f "$MAP_YAML" ] && [ -f "$MAP_PGM" ]; then
    echo "警告: pcd2pgm达到 ${PCD2PGM_TIMEOUT}s 超时，但地图文件已生成，继续记录候选地图。"
  else
    echo "错误: pcd2pgm超时 ${PCD2PGM_TIMEOUT}s，未生成地图文件" >&2
    exit 124
  fi
elif [ "$PCD2PGM_STATUS" -ne 0 ]; then
  echo "错误: pcd2pgm执行失败，状态码 $PCD2PGM_STATUS" >&2
  exit "$PCD2PGM_STATUS"
fi

if [ ! -f "$MAP_YAML" ] || [ ! -f "$MAP_PGM" ]; then
  echo "错误: 未生成Nav2地图文件: $MAP_YAML / $MAP_PGM" >&2
  exit 1
fi

if [ "$MAP_SPECKLE_FILTER" = "true" ]; then
  /home/wheeltec/rtk_tools/filter_nav2_map_speckles.py \
    --yaml "$MAP_YAML" \
    --min-component-cells "$MIN_COMPONENT_CELLS" \
    --min-component-length "$MIN_COMPONENT_LENGTH" \
    --connectivity "$SPECKLE_CONNECTIVITY"
fi

printf '%s\n' "$MAP_YAML" > "$LATEST_FILE"

echo
echo "已生成Nav2候选地图:"
echo "  yaml: $MAP_YAML"
echo "  pgm : $MAP_PGM"
echo "已记录最新候选地图:"
echo "  $LATEST_FILE"
echo
echo "注意: 当前Nav2全局地图尚未更新。启动导航前运行:"
echo "  /home/wheeltec/rtk_tools/start_rtk_nav2_with_map_check.sh"
