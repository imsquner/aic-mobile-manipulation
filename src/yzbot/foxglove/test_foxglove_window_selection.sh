#!/usr/bin/env bash
# Foxglove 窗口选择的离线单测。
# 先失败证明：改造前 start_offline_foxglove.sh 里既没有 foxglove_window_selection.sh，
# 也没有任何 FOXGLOVE_WINDOWS 开关，三个窗口是硬编码的 —— 本测试在改造前
# 会以 "No such file" 失败。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="$HERE/foxglove_window_selection.sh"
FAIL=0
PASS=0

check() { # check <说明> <期望> <实际>
  if [[ "$2" == "$3" ]]; then
    PASS=$((PASS + 1))
  else
    FAIL=$((FAIL + 1))
    echo "FAIL: $1  期望='$2'  实际='$3'"
  fi
}

if [[ ! -f "$TARGET" ]]; then
  echo "FAIL: 找不到 $TARGET"
  exit 1
fi
# shellcheck disable=SC1090
source "$TARGET"

# --- 归一化 ---
check "未设置 -> dashboard"   "dashboard"                   "$(foxglove_normalize_windows "")"
check "空格   -> dashboard"   "dashboard"                   "$(foxglove_normalize_windows " ")"
check "all    -> 三个"        "map detection dashboard"     "$(foxglove_normalize_windows all)"
check "ALL    -> 三个"        "map detection dashboard"     "$(foxglove_normalize_windows ALL)"
check "逗号分隔"              "map detection"               "$(foxglove_normalize_windows map,detection)"
check "逗号+空格"             "map detection"               "$(foxglove_normalize_windows "map, detection")"
check "单个"                  "detection"                   "$(foxglove_normalize_windows detection)"
check "重复项去重"            "map dashboard"               "$(foxglove_normalize_windows map,map,dashboard)"
check "顺序保持"              "dashboard map detection"     "$(foxglove_normalize_windows dashboard,map,detection)"

# --- 校验 ---
rc_of() { # rc_of <值> -> 打印校验结果码（0 合法 / 1 非法）
  if foxglove_validate_windows "$1"; then printf '0'; else printf '1'; fi
}
check "校验 dashboard"      "0" "$(rc_of dashboard)"
check "校验 三者"           "0" "$(rc_of "map detection dashboard")"
check "拒绝 bogus"          "1" "$(rc_of bogus)"
check "拒绝混合非法"        "1" "$(rc_of "map bogus")"
check "空串放行(由归一化兜)" "0" "$(rc_of "")"

# --- 端口 ---
for w in map:8088 detection:8089 dashboard:8090; do
  name="${w%%:*}"; want="${w##*:}"
  check "端口 $name" "$want" "$(foxglove_window_port "$name")"
done

# --- 几何 ---
check "几何 dashboard" "1000,650|100,0" "$(foxglove_window_geometry dashboard)"
check "几何 detection" "640,480|920,0"  "$(foxglove_window_geometry detection)"
check "几何 map"       "900,650|0,0"    "$(foxglove_window_geometry map)"

# --- 启动脚本必须真的用上这套开关 ---
LAUNCHER="$HERE/start_offline_foxglove.sh"
for needle in "FOXGLOVE_WINDOWS" "foxglove_window_selection.sh" "FOXGLOVE_NICE"; do
  if grep -q -- "$needle" "$LAUNCHER"; then
    PASS=$((PASS + 1))
  else
    FAIL=$((FAIL + 1)); echo "FAIL: 启动脚本缺少 $needle"
  fi
done
# 默认必须是 dashboard，不能退回三个窗口
if grep -q 'WINDOWS="\$(foxglove_normalize_windows "\${FOXGLOVE_WINDOWS:-}")"' "$LAUNCHER"; then
  PASS=$((PASS + 1))
else
  FAIL=$((FAIL + 1)); echo "FAIL: 启动脚本没有用 FOXGLOVE_WINDOWS 归一化默认值"
fi
# 不再用 mktemp 每次新建 profile（要跨轮复用固定 profile）
if grep -q 'mktemp -d "\$PROFILE_BASE/profile-' "$LAUNCHER"; then
  FAIL=$((FAIL + 1)); echo "FAIL: 仍在用 mktemp 新建 profile"
else
  PASS=$((PASS + 1))
fi
# 三个窗口不再硬编码
if grep -q '"map|\$MAP_URL|900,650|0,0"' "$LAUNCHER"; then
  FAIL=$((FAIL + 1)); echo "FAIL: 三个窗口仍硬编码"
else
  PASS=$((PASS + 1))
fi

echo
echo "通过 $PASS 项，失败 $FAIL 项"
[[ "$FAIL" -eq 0 ]]
