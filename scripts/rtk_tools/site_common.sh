#!/usr/bin/env bash

SITE_ROOT_DEFAULT="/home/wheeltec/field_sites"
SITE_DELETED_ROOT_DEFAULT="/home/wheeltec/field_sites_deleted"

site_root() {
  printf '%s\n' "${SITE_ROOT:-$SITE_ROOT_DEFAULT}"
}

site_registry_file() {
  printf '%s/sites.txt\n' "$(site_root)"
}

site_deleted_root() {
  printf '%s\n' "${SITE_DELETED_ROOT:-$SITE_DELETED_ROOT_DEFAULT}"
}

sanitize_site_name() {
  local name="$1"
  name="${name//\//_}"
  name="${name//\\/_}"
  name="${name//:/_}"
  name="${name//$'\n'/_}"
  printf '%s\n' "$name"
}

site_dir_from_name() {
  local name
  name="$(sanitize_site_name "$1")"
  printf '%s/%s\n' "$(site_root)" "$name"
}

ensure_site_dirs() {
  local dir="$1"
  mkdir -p \
    "$dir/maps/fast_lio_sessions" \
    "$dir/maps/nav2_current" \
    "$dir/bags" \
    "$dir/logs"
}

register_site_name() {
  local name="$1"
  local registry
  registry="$(site_registry_file)"
  mkdir -p "$(dirname "$registry")"
  touch "$registry"
  if ! grep -Fxq "$name" "$registry"; then
    printf '%s\n' "$name" >> "$registry"
  fi
}

unregister_site_name() {
  local name="$1"
  local registry tmp_file
  registry="$(site_registry_file)"
  tmp_file="${registry}.tmp"
  mkdir -p "$(dirname "$registry")"
  touch "$registry"
  grep -Fxv "$name" "$registry" > "$tmp_file" || true
  mv "$tmp_file" "$registry"
}

archive_site_dir() {
  local name="$1"
  local src_dir archive_root timestamp dst_dir
  src_dir="$(site_dir_from_name "$name")"
  archive_root="$(site_deleted_root)"
  timestamp="$(date +%Y%m%d_%H%M%S)"
  dst_dir="$archive_root/${name}_${timestamp}"

  if [ ! -d "$src_dir" ]; then
    unregister_site_name "$name"
    return 0
  fi

  mkdir -p "$archive_root"
  mv "$src_dir" "$dst_dir"
  unregister_site_name "$name"
  printf '%s\n' "$dst_dir"
}

site_current_yaml() {
  printf '%s/maps/nav2_current/current.yaml\n' "$1"
}

site_latest_candidate_file() {
  printf '%s/maps/nav2_latest_candidate.txt\n' "$1"
}

site_fastlio_sessions_dir() {
  printf '%s/maps/fast_lio_sessions\n' "$1"
}

site_bags_dir() {
  printf '%s/bags\n' "$1"
}

site_logs_dir() {
  printf '%s/logs\n' "$1"
}
