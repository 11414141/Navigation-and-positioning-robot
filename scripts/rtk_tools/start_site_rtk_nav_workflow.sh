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

source_if_exists /opt/ros/humble/setup.bash
source_if_exists /home/wheeltec/wheeltec_ros2/install/setup.bash
source_if_exists /home/wheeltec/ws_livox/install/setup.bash
source_if_exists /home/wheeltec/ws_2dmap/install/setup.bash

SUPPORT_PIDS=()
FASTLIO_START_DELAY="${FASTLIO_START_DELAY:-3}"

choose_site() {
  local registry
  registry="$(site_registry_file)"
  mkdir -p "$(dirname "$registry")"
  touch "$registry"

  while true; do
    mapfile -t sites < "$registry"

    echo "========== 选择工作地点 =========="
    if [ "${#sites[@]}" -gt 0 ]; then
      local i=1
      for site in "${sites[@]}"; do
        echo "  $i) $site"
        i=$((i + 1))
      done
    else
      echo "  当前还没有已登记工作地点。"
    fi
    echo "  n) 新增工作地点"
    echo "  d) 删除工作地点"
    echo

    read -r -p "请选择工作地点编号、n 或 d: " choice
    if [ "$choice" = "n" ] || [ "$choice" = "N" ]; then
      read -r -p "请输入新工作地点名称: " site_name
      if [ -z "$site_name" ]; then
        echo "错误: 工作地点名称不能为空" >&2
        exit 1
      fi
      site_name="$(sanitize_site_name "$site_name")"
      register_site_name "$site_name"
      break
    elif [ "$choice" = "d" ] || [ "$choice" = "D" ]; then
      delete_site_interactive
      continue
    elif [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#sites[@]}" ]; then
      site_name="${sites[$((choice - 1))]}"
      break
    else
      echo "错误: 无效选择: $choice" >&2
      exit 1
    fi
  done

  SITE_DIR="$(site_dir_from_name "$site_name")"
  export SITE_DIR
  ensure_site_dirs "$SITE_DIR"
  echo
  echo "当前工作地点: $site_name"
  echo "工作目录: $SITE_DIR"
  echo
}

delete_site_interactive() {
  local registry site_count target name archived
  registry="$(site_registry_file)"
  mapfile -t sites < "$registry"
  site_count="${#sites[@]}"

  if [ "$site_count" -eq 0 ]; then
    echo "当前没有可删除的工作地点。"
    echo
    return 0
  fi

  echo
  echo "========== 删除工作地点 =========="
  local i=1
  for name in "${sites[@]}"; do
    echo "  $i) $name"
    i=$((i + 1))
  done
  echo "  c) 取消"
  echo
  read -r -p "请选择要删除的工作地点: " target
  if [ "$target" = "c" ] || [ "$target" = "C" ]; then
    echo "已取消删除。"
    echo
    return 0
  fi
  if ! [[ "$target" =~ ^[0-9]+$ ]] || [ "$target" -lt 1 ] || [ "$target" -gt "$site_count" ]; then
    echo "错误: 无效选择: $target" >&2
    exit 1
  fi

  name="${sites[$((target - 1))]}"
  echo
  echo "将删除工作地点登记并归档文件夹:"
  echo "  工作地点: $name"
  echo "  原目录: $(site_dir_from_name "$name")"
  echo "  归档根目录: $(site_deleted_root)"
  echo
  read -r -p "确认归档删除该工作地点及其文件夹？[y/N] " answer
  case "$answer" in
    y|Y|yes|YES)
      archived="$(archive_site_dir "$name")"
      echo "已归档删除。"
      if [ -n "$archived" ]; then
        echo "归档目录: $archived"
      fi
      ;;
    *)
      echo "已取消删除。"
      ;;
  esac
  echo
}

ask_scan_mapping_mode() {
  echo "========== 扫描建图更新 ==========" >&2
  echo "是否在本次巡检中更新扫描建图？" >&2
  echo "  1) 启用 FAST-LIO2 实时建图" >&2
  echo "  2) 录制 bag，巡检结束后离线 FAST-LIO2 建图" >&2
  echo "  3) 不更新扫描建图" >&2
  echo >&2
  read -r -p "请选择 [1/2/3，默认3]: " mode
  mode="${mode:-3}"
  case "$mode" in
    1|2|3)
      printf '%s\n' "$mode"
      ;;
    *)
      echo "错误: 无效选择: $mode" >&2
      exit 1
      ;;
  esac
}

resolve_image_path() {
  local yaml_file="$1"
  local image_line
  image_line="$(grep -E '^image:' "$yaml_file" | head -1 | sed 's/^image:[[:space:]]*//')"
  if [[ "$image_line" = /* ]]; then
    printf '%s\n' "$image_line"
  else
    printf '%s/%s\n' "$(dirname "$yaml_file")" "$image_line"
  fi
}

install_current_map() {
  local candidate_yaml="$1"
  local current_dir="$SITE_DIR/maps/nav2_current"
  local current_yaml="$current_dir/current.yaml"
  local current_pgm="$current_dir/current.pgm"
  local candidate_image

  candidate_image="$(resolve_image_path "$candidate_yaml")"
  if [ ! -f "$candidate_image" ]; then
    echo "错误: 候选地图图像不存在: $candidate_image" >&2
    return 1
  fi

  mkdir -p "$current_dir"
  cp "$candidate_image" "$current_pgm"
  cp "$candidate_yaml" "$current_yaml"
  sed -i 's|^image:.*|image: current.pgm|' "$current_yaml"
}

choose_site_with_current_map() {
  local selected_site="$1"
  local registry candidates candidate_sites i choice selected_yaml
  registry="$(site_registry_file)"
  mapfile -t candidates < "$registry"
  candidate_sites=()

  for site in "${candidates[@]}"; do
    if [ "$site" = "$selected_site" ]; then
      continue
    fi
    selected_yaml="$(site_current_yaml "$(site_dir_from_name "$site")")"
    if [ -f "$selected_yaml" ]; then
      candidate_sites+=("$site")
    fi
  done

  if [ "${#candidate_sites[@]}" -eq 0 ]; then
    echo "没有找到其他已创建全局地图的工作地点。" >&2
    return 1
  fi

  echo "可用的其他工作地点地图:"
  i=1
  for site in "${candidate_sites[@]}"; do
    echo "  $i) $site"
    i=$((i + 1))
  done
  echo
  read -r -p "请选择要借用的工作地点地图: " choice
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#candidate_sites[@]}" ]; then
    echo "错误: 无效选择: $choice" >&2
    return 1
  fi

  selected_yaml="$(site_current_yaml "$(site_dir_from_name "${candidate_sites[$((choice - 1))]}")")"
  install_current_map "$selected_yaml"
  echo "已将其他工作地点地图复制为当前地点全局地图:"
  echo "  $(site_current_yaml "$SITE_DIR")"
}

import_existing_map() {
  local yaml_file
  read -r -p "请输入已有Nav2地图yaml完整路径: " yaml_file
  if [ -z "$yaml_file" ] || [ ! -f "$yaml_file" ]; then
    echo "错误: 地图yaml不存在: ${yaml_file:-未输入}" >&2
    return 1
  fi

  install_current_map "$yaml_file"
  echo "已导入当前地点全局地图:"
  echo "  $(site_current_yaml "$SITE_DIR")"
}

ask_update_global_map() {
  local current_yaml latest_file candidate_yaml
  current_yaml="$(site_current_yaml "$SITE_DIR")"
  latest_file="$(site_latest_candidate_file "$SITE_DIR")"
  candidate_yaml=""

  if [ -f "$latest_file" ]; then
    candidate_yaml="$(cat "$latest_file")"
  fi

  echo "========== Nav2全局地图 =========="
  echo "当前工作地点全局地图:"
  if [ -f "$current_yaml" ]; then
    echo "  $current_yaml"
  else
    echo "  尚未创建。"
  fi
  echo
  echo "该工作地点最新候选地图:"
  if [ -n "$candidate_yaml" ] && [ -f "$candidate_yaml" ]; then
    echo "  $candidate_yaml"
    read -r -p "是否更新Nav2全局地图为该候选地图？[y/N] " answer
    case "$answer" in
      y|Y|yes|YES)
        install_current_map "$candidate_yaml"
        echo "已更新当前全局地图: $current_yaml"
        ;;
      *)
        echo "保持当前全局地图不变。"
        ;;
    esac
  else
    echo "  未找到"
    echo "没有候选地图，跳过更新。"
  fi
  echo
}

map_to_use() {
  local current_yaml
  current_yaml="$(site_current_yaml "$SITE_DIR")"
  if [ -f "$current_yaml" ]; then
    printf '%s\n' "$current_yaml"
  else
    return 1
  fi
}

check_existing_nodes() {
  local node_list conflicts
  set +e
  node_list="$(ros2 node list 2>/dev/null)"
  local status=$?
  set -e
  if [ "$status" -ne 0 ]; then
    echo "警告: 无法查询 ros2 node list，跳过残留节点检查。"
    return 0
  fi

  conflicts="$(printf '%s\n' "$node_list" | grep -E \
    '^/(map_server|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager_rtk_navigation|zed_f9p_ntrip_node|rtk_quality_filter|ekf_filter_node_odom|ekf_filter_node_map|navsat_transform|livox_custom_to_pointcloud2|livox_lidar_publisher|laser_mapping)$' || true)"

  if [ -n "$conflicts" ]; then
    echo "错误: 检测到旧的RTK/Nav2/建图相关节点仍在运行，先不要重复启动。"
    echo
    echo "$conflicts" | sed 's/^/  /'
    echo
    echo "请先到之前启动这些节点的终端按 Ctrl+C，或重启ROS相关终端/小车系统。"
    exit 1
  fi
}

start_manual_mapping_support_nodes() {
  SUPPORT_PIDS=()

  echo "启动手动建图所需基础节点:"
  echo "  base_serial, robot_description, Livox MID360s, livox_frame静态TF"

  ros2 launch turn_on_wheeltec_robot base_serial.launch.py &
  SUPPORT_PIDS+=("$!")

  ros2 launch turn_on_wheeltec_robot robot_mode_description.launch.py &
  SUPPORT_PIDS+=("$!")

  ros2 launch livox_ros_driver2 msg_MID360s_launch.py &
  SUPPORT_PIDS+=("$!")

  ros2 run tf2_ros static_transform_publisher \
    --x 0.14 --y 0.0 --z 0.16 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_footprint --child-frame-id livox_frame &
  SUPPORT_PIDS+=("$!")

  echo "等待雷达/IMU稳定 ${FASTLIO_START_DELAY}s 后再启动FAST-LIO2..."
  sleep "$FASTLIO_START_DELAY"
}

stop_manual_mapping_support_nodes() {
  if [ "${#SUPPORT_PIDS[@]}" -gt 0 ]; then
    echo
    echo "停止手动建图基础节点..."
    local pid
    for pid in "${SUPPORT_PIDS[@]}"; do
      kill -INT "$pid" 2>/dev/null || true
    done
    for pid in "${SUPPORT_PIDS[@]}"; do
      wait "$pid" 2>/dev/null || true
    done
    SUPPORT_PIDS=()
  fi
}

manual_realtime_fastlio_mapping() {
  start_manual_mapping_support_nodes
  trap 'stop_manual_mapping_support_nodes' EXIT
  echo
  echo "进入手动遥控实时FAST-LIO2建图。结束建图时在本终端按 Ctrl+C。"
  SITE_DIR="$SITE_DIR" RVIZ="${FASTLIO_RVIZ:-true}" FASTLIO_START_DELAY=0 /home/wheeltec/rtk_tools/start_fastlio_mapping_auto_2d.sh
  trap - EXIT
  stop_manual_mapping_support_nodes
}

manual_bag_then_offline_mapping() {
  local date_str time_str bag_root out_path
  date_str="$(date +%Y%m%d)"
  time_str="$(date +%Y%m%d_%H%M%S)"
  bag_root="$(site_bags_dir "$SITE_DIR")/manual_mapping_${date_str}"
  out_path="${bag_root}/fastlio_manual_${time_str}"
  mkdir -p "$bag_root"

  start_manual_mapping_support_nodes
  trap 'stop_manual_mapping_support_nodes' EXIT
  echo
  echo "开始手动遥控录制FAST-LIO2建图bag。结束录制时在本终端按 Ctrl+C。"
  set +e
  OUT_PATH="$out_path" SITE_DIR="$SITE_DIR" /home/wheeltec/rtk_tools/record_rtk_nav_bag.sh fastlio
  local record_status=$?
  set -e
  trap - EXIT
  stop_manual_mapping_support_nodes

  if [ "$record_status" -ne 0 ] && [ "$record_status" -ne 130 ] && [ "$record_status" -ne 143 ]; then
    echo "错误: bag录制异常退出，状态码 $record_status。跳过离线建图。" >&2
    exit "$record_status"
  fi

  if [ -d "$out_path" ]; then
    echo
    echo "开始离线FAST-LIO2建图并生成2D候选地图:"
    echo "  bag: $out_path"
    SITE_DIR="$SITE_DIR" /home/wheeltec/rtk_tools/run_fastlio_offline_from_bag.sh "$out_path"
  else
    echo "错误: 未找到录制后的bag目录: $out_path" >&2
    exit 1
  fi
}

manual_mapping_without_nav2() {
  echo "========== 手动遥控建图 =========="
  echo "当前地点没有全局地图，将不启动Nav2自动导航。"
  echo "请选择建图方式:"
  echo "  1) 实时FAST-LIO2建图，结束后自动切片并转2D"
  echo "  2) 录bag，结束后离线FAST-LIO2建图并转2D"
  echo
  read -r -p "请选择 [1/2]: " mode
  case "$mode" in
    1)
      manual_realtime_fastlio_mapping
      ;;
    2)
      manual_bag_then_offline_mapping
      ;;
    *)
      echo "错误: 无效选择: $mode" >&2
      exit 1
      ;;
  esac

  echo
  echo "手动建图流程已结束。本次任务退出；下次启动时可选择是否把候选地图更新为全局地图。"
  exit 0
}

handle_missing_current_map() {
  local current_yaml mode
  current_yaml="$(site_current_yaml "$SITE_DIR")"
  if [ -f "$current_yaml" ]; then
    return 0
  fi

  echo "========== 当前工作地点尚无全局地图 =========="
  echo "当前工作地点没有可用于Nav2的全局地图:"
  echo "  $current_yaml"
  echo
  echo "请选择处理方式:"
  echo "  1) 使用其他工作地点的全局地图进行Nav2导航巡检"
  echo "  2) 导入已有Nav2地图进行Nav2导航巡检"
  echo "  3) 不启动Nav2，手动遥控机器人建图或录bag"
  echo
  read -r -p "请选择 [1/2/3]: " mode

  case "$mode" in
    1)
      choose_site_with_current_map "$site_name"
      ;;
    2)
      import_existing_map
      ;;
    3)
      manual_mapping_without_nav2
      ;;
    *)
      echo "错误: 无效选择: $mode" >&2
      exit 1
      ;;
  esac

  if [ ! -f "$current_yaml" ]; then
    echo "错误: 当前工作地点仍没有全局地图，无法启动Nav2。" >&2
    exit 1
  fi
  echo
}

prepare_realtime_fastlio_session() {
  local session_id session_dir raw_dir map_file config_file rviz
  session_id="$(date +%Y%m%d_%H%M%S)_nav_realtime"
  session_dir="$(site_fastlio_sessions_dir "$SITE_DIR")/$session_id"
  raw_dir="$session_dir/raw"
  map_file="$raw_dir/fast_lio_full.pcd"
  config_file="${CONFIG_FILE:-mid360.yaml}"
  rviz="${FASTLIO_RVIZ:-false}"
  mkdir -p "$raw_dir"
  export FASTLIO_MAP_FILE="$map_file"
  export FASTLIO_CONFIG_FILE="$config_file"
  export FASTLIO_RVIZ_VALUE="$rviz"
}

run_realtime_fastlio_foreground() {
  echo "启动实时FAST-LIO2建图:"
  echo "  raw_pcd: $FASTLIO_MAP_FILE"
  exec ros2 launch fast_lio mapping.launch.py \
    config_file:="$FASTLIO_CONFIG_FILE" \
    map_file_path:="$FASTLIO_MAP_FILE" \
    convert_livox_points:=false \
    rviz:="$FASTLIO_RVIZ_VALUE"
}

delayed_launch_realtime_fastlio() {
  echo
  echo "等待雷达/IMU稳定 ${FASTLIO_START_DELAY}s 后启动实时FAST-LIO2..."
  sleep "$FASTLIO_START_DELAY"
  run_realtime_fastlio_foreground
}

stop_realtime_fastlio_and_postprocess() {
  if [ -n "${FASTLIO_DELAY_PID:-}" ]; then
    kill -INT "$FASTLIO_DELAY_PID" 2>/dev/null || true
    wait "$FASTLIO_DELAY_PID" 2>/dev/null || true
  fi
  if [ -n "${FASTLIO_MAP_FILE:-}" ]; then
    echo
    if [ -f "${FASTLIO_MAP_FILE:-}" ]; then
      echo "实时FAST-LIO2已停止，生成候选2D地图..."
      /home/wheeltec/rtk_tools/build_nav2_map_from_fastlio_pcd.sh "$FASTLIO_MAP_FILE"
    else
      echo "警告: 未找到实时FAST-LIO2输出PCD，跳过2D地图生成: ${FASTLIO_MAP_FILE:-未设置}" >&2
    fi
  fi
}

start_nav2_for_inspection() {
  local map_file
  if ! map_file="$(map_to_use)"; then
    echo "错误: 当前工作地点没有可用地图，无法启动Nav2。" >&2
    exit 1
  fi
  if [ ! -f "$map_file" ]; then
    echo "错误: 没有可用地图，无法启动Nav2: $map_file" >&2
    exit 1
  fi
  echo "Nav2使用地图:"
  echo "  $map_file"
  echo
  ros2 launch wheeltec_nav2 wheeltec_rtk_nav2.launch.py map:="$map_file"
}

record_bag_during_nav2() {
  local date_str time_str bag_root out_path
  date_str="$(date +%Y%m%d)"
  time_str="$(date +%Y%m%d_%H%M%S)"
  bag_root="$(site_bags_dir "$SITE_DIR")/rtk_nav_${date_str}"
  out_path="${bag_root}/rtk_nav_full_${time_str}"
  mkdir -p "$bag_root"

  echo "启动bag录制:"
  echo "  $out_path"
  OUT_PATH="$out_path" /home/wheeltec/rtk_tools/record_rtk_nav_bag.sh full &
  BAG_PID=$!
  export BAG_PID
  export RECORDED_BAG_PATH="$out_path"
}

stop_bag_and_run_offline_mapping() {
  if [ -n "${BAG_PID:-}" ]; then
    echo
    echo "停止bag录制..."
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
    if [ -d "${RECORDED_BAG_PATH:-}" ]; then
      echo "开始用bag离线FAST-LIO2建图..."
      /home/wheeltec/rtk_tools/run_fastlio_offline_from_bag.sh "$RECORDED_BAG_PATH"
    else
      echo "警告: 未找到录制bag目录，跳过离线建图: ${RECORDED_BAG_PATH:-未设置}" >&2
    fi
  fi
}

check_existing_nodes
choose_site
handle_missing_current_map
ask_update_global_map
SCAN_MODE="$(ask_scan_mapping_mode)"

case "$SCAN_MODE" in
  1)
    prepare_realtime_fastlio_session
    delayed_launch_realtime_fastlio &
    FASTLIO_DELAY_PID=$!
    export FASTLIO_DELAY_PID
    trap 'stop_realtime_fastlio_and_postprocess' EXIT
    ;;
  2)
    record_bag_during_nav2
    trap 'stop_bag_and_run_offline_mapping' EXIT
    ;;
  3)
    ;;
esac

echo "========== 启动Nav2巡检 =========="
echo "已完成工作地点、全局地图和扫描建图模式选择。"
echo

start_nav2_for_inspection
