#!/usr/bin/env bash
# Foxglove 窗口集合的归一化与校验（纯函数，便于离线单测）。
#
# 被 `start_offline_foxglove.sh` source 使用；不要在 source 时产生副作用。
#
# 语义（照 test13 性能记录 §5）：
#   FOXGLOVE_WINDOWS 未设置或空 -> dashboard（默认只开比赛仪表盘）
#   all / ALL                   -> map detection dashboard（旧的三个窗口行为）
#   逗号或空格分隔              -> 逐项采用，未知项判为非法
#
# 为什么默认只开 dashboard：map 窗口的 3D 面板在无硬件 GL 的 VM 上会吞掉可观 CPU，
# 实测把相机从 ~13 Hz 压到 ~5 Hz；地图/相机/激光/TF 的调试职责交给 RViz。

foxglove_normalize_windows() {
  local select="${1:-}"
  local -a parts
  case "$select" in
    ""|" ")
      printf '%s' "dashboard"
      return 0
      ;;
    all|ALL)
      printf '%s' "map detection dashboard"
      return 0
      ;;
  esac
  # 逗号或空格分隔都接受；先去掉重复项，再用 "${uniq[*]}" 收敛空白，
  # 避免 runtime.env 里出现 "map  detection" 或重复窗口。
  # 这里的去重顺序与 start_competition.py::foxglove_selected_windows 必须一致，
  # 由 test_foxglove_window_contract.py 守住。
  local -a parts uniq
  local item existing seen
  read -r -a parts <<<"${select//,/ }"
  for item in "${parts[@]}"; do
    seen=0
    for existing in ${uniq[@]+"${uniq[@]}"}; do
      if [[ "$existing" == "$item" ]]; then seen=1; break; fi
    done
    [[ "$seen" -eq 0 ]] && uniq+=("$item")
  done
  printf '%s' "${uniq[*]}"
}

foxglove_validate_windows() {
  local window
  for window in $1; do
    case "$window" in
      map|detection|dashboard) ;;
      *) return 1 ;;
    esac
  done
  return 0
}

foxglove_window_port() {
  case "$1" in
    map) printf '%s' "${FOXGLOVE_MAP_PORT:-8088}" ;;
    detection) printf '%s' "${FOXGLOVE_DETECTION_PORT:-8089}" ;;
    dashboard) printf '%s' "${FOXGLOVE_DASHBOARD_PORT:-8090}" ;;
    *) return 1 ;;
  esac
}

foxglove_window_geometry() {
  # 输出 "<宽,高>|<x,y>"
  case "$1" in
    map) printf '%s' "900,650|0,0" ;;
    detection) printf '%s' "640,480|920,0" ;;
    dashboard) printf '%s' "1000,650|100,0" ;;
    *) return 1 ;;
  esac
}
