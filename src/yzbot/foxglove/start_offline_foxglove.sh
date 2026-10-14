#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
WORKSPACE="$(cd "$REPO_DIR/../.." && pwd)"
ARCHIVE="$SCRIPT_DIR/offline_studio_1.87.0_web.tar.gz"
CACHE_BASE="${XDG_CACHE_HOME:-$HOME/.cache}/yzbot-foxglove"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/yzbot-foxglove"
MAP_HTTP_PORT="${FOXGLOVE_MAP_PORT:-8088}"
DETECTION_HTTP_PORT="${FOXGLOVE_DETECTION_PORT:-8089}"
DASHBOARD_HTTP_PORT="${FOXGLOVE_DASHBOARD_PORT:-8090}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9091}"
DISPLAY_VALUE="${DISPLAY:-:0}"
NO_BROWSER=0

if [[ "${1:-}" == "--no-browser" ]]; then
  NO_BROWSER=1
fi

mkdir -p "$CACHE_BASE" "$STATE_DIR"
test -s "$ARCHIVE" || {
  echo "Missing offline Foxglove archive: $ARCHIVE" >&2
  exit 2
}

ARCHIVE_HASH="$(sha256sum "$ARCHIVE" | awk '{print $1}')"
WEB_ROOT="$CACHE_BASE/web-$ARCHIVE_HASH"
PROFILE_DIRS=()
BROWSER_PIDS=()
declare -A BROWSER_PID=()
declare -A BROWSER_RESTARTS=()
BROWSER_FAILED_WINDOWS=""
LAUNCH_ORDER=""
HTTP_PIDS=()

if [[ ! -s "$WEB_ROOT/index.html" ]]; then
  TEMP_ROOT="$(mktemp -d "$CACHE_BASE/extract.XXXXXX")"
  tar -xzf "$ARCHIVE" -C "$TEMP_ROOT"
  test -s "$TEMP_ROOT/index.html" || {
    echo "Offline Foxglove archive has no index.html" >&2
    exit 3
  }
  mv "$TEMP_ROOT" "$WEB_ROOT"
fi

ROS_DISTRO_VALUE="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_VALUE/setup.bash"
test -f "$ROS_SETUP" || {
  echo "ROS setup not found: $ROS_SETUP" >&2
  exit 4
}

set +u
source "$ROS_SETUP"
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  source "$WORKSPACE/install/setup.bash"
fi
set -u

command -v python3 >/dev/null
ROSBRIDGE_EXEC="$(ros2 pkg prefix rosbridge_server)/lib/rosbridge_server/rosbridge_websocket"
ROSAPI_EXEC="$(ros2 pkg prefix rosapi)/lib/rosapi/rosapi_node"
test -x "$ROSBRIDGE_EXEC"
test -x "$ROSAPI_EXEC"
export DISPLAY="$DISPLAY_VALUE"

if command -v xdpyinfo >/dev/null; then
  xdpyinfo -display "$DISPLAY" >/dev/null 2>&1 || {
    echo "Ubuntu display is unavailable: DISPLAY=$DISPLAY" >&2
    exit 5
  }
fi

# --- 窗口选择（照 test13 PERF-§5：默认只开 dashboard）-------------------------
# test13 的性能记录把「Foxglove 默认只开 dashboard」列为降低可视化负载的主要手段
# ——地图/相机/激光/TF 的调试职责交给 RViz。本交付包原先固定开
# map+detection+dashboard 三个窗口，其中 map 的 3D 面板在无硬件 GL 的 VM 上会占掉
# 可观 CPU（实测相机从 ~13 Hz 掉到 ~5 Hz）。
# 归一化/校验/端口/几何都在 foxglove_window_selection.sh 里，便于离线单测。
source "$SCRIPT_DIR/foxglove_window_selection.sh"
WINDOWS="$(foxglove_normalize_windows "${FOXGLOVE_WINDOWS:-}")"
if ! foxglove_validate_windows "$WINDOWS"; then
  echo "Unknown FOXGLOVE_WINDOWS='${FOXGLOVE_WINDOWS:-}' (use dashboard|map|detection|all, comma separated)" >&2
  exit 8
fi

window_port() { foxglove_window_port "$1"; }
window_geometry() { foxglove_window_geometry "$1"; }
window_url() {
  case "$1" in
    map) printf '%s' "$MAP_URL" ;;
    detection) printf '%s' "$DETECTION_URL" ;;
    dashboard) printf '%s' "$DASHBOARD_URL" ;;
  esac
}

SELECTED_PORTS=("$ROSBRIDGE_PORT")
for window in $WINDOWS; do SELECTED_PORTS+=("$(window_port "$window")"); done
for port in "${SELECTED_PORTS[@]}"; do
  if ss -ltn "sport = :$port" | grep -q LISTEN; then
    echo "Required port is already in use: $port" >&2
    exit 6
  fi
done

ROSBRIDGE_LOG="$STATE_DIR/rosbridge.log"
LAYOUT_ARGS=()
for window in $WINDOWS; do
  LAYOUT_ARGS+=(--layout "$SCRIPT_DIR/window_$window.json")
done
WINDOW_ROOTS_OUTPUT="$(python3 "$SCRIPT_DIR/prepare_window_roots.py" \
  --base-root "$WEB_ROOT" --cache-root "$CACHE_BASE" "${LAYOUT_ARGS[@]}")"
declare -A WINDOW_ROOTS=()
for window in $WINDOWS; do
  WINDOW_ROOTS["$window"]="$(awk -F= -v k="window_$window" '$1 == k {print $2}' <<<"$WINDOW_ROOTS_OUTPUT")"
  if [[ -z "${WINDOW_ROOTS[$window]}" ]]; then
    echo "Failed to prepare Foxglove layout root for window: $window" >&2
    exit 9
  fi
done
for window in $WINDOWS; do
  port="$(window_port "$window")"
  python3 -u -m http.server "$port" --bind 127.0.0.1 \
    --directory "${WINDOW_ROOTS[$window]}" \
    >"$STATE_DIR/http-$window.log" 2>&1 &
  HTTP_PIDS+=("$!")
done

"$ROSBRIDGE_EXEC" --ros-args \
  -p "port:=$ROSBRIDGE_PORT" -p address:=127.0.0.1 \
  -p default_call_service_timeout:=5.0 -p call_services_in_new_thread:=true \
  -p send_action_goals_in_new_thread:=true >"$ROSBRIDGE_LOG" 2>&1 &
ROSBRIDGE_PID=$!

# rosapi 要看护启动：Humble 的 rosapi_node::get_node_details 在枚举途中遇到
# 节点消失会抛 `TypeError: cannot unpack non-iterable NoneType object` 而退出
# （2026-09-23 实测，dashboard 会遍历 /rosapi/node_details）。rosapi 一死，
# 仪表盘就永久失联；如果直接把它当必需进程，还会连锁把整场比赛判失败。
# 这里给它一个重启预算，让可视化能自愈。
ROSAPI_MAX_RESTARTS="${FOXGLOVE_ROSAPI_MAX_RESTARTS:-30}"
rosapi_supervisor() {
  local attempts=0 child=0
  # 收到 TERM 时要把正在跑的 rosapi 一起带走，否则 cleanup 只杀到看护进程。
  trap 'kill -TERM "$child" 2>/dev/null; exit 0' TERM INT
  while [ "$attempts" -lt "$ROSAPI_MAX_RESTARTS" ]; do
    "$ROSAPI_EXEC" >>"$STATE_DIR/rosapi.log" 2>&1 &
    child=$!
    # 脚本开了 set -e，子进程崩溃会让 wait 返回非零，必须显式吞掉。
    wait "$child" || true
    attempts=$((attempts + 1))
    echo "[supervisor] rosapi exited; restart $attempts/$ROSAPI_MAX_RESTARTS" \
      >>"$STATE_DIR/rosapi.log"
    sleep 1
  done
  echo "[supervisor] rosapi restart budget exhausted" >>"$STATE_DIR/rosapi.log"
  exit 1
}
rosapi_supervisor &
ROSAPI_PID=$!

stop_pid() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.2
  done
  kill -KILL "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${BROWSER_PIDS[@]}" "$ROSAPI_PID" "$ROSBRIDGE_PID" "${HTTP_PIDS[@]}"; do
    stop_pid "$pid"
  done
  wait "$ROSBRIDGE_PID" 2>/dev/null || true
  wait "$ROSAPI_PID" 2>/dev/null || true
  for pid in "${HTTP_PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
  for profile in "${PROFILE_DIRS[@]}"; do
    if [[ -n "$profile" && -d "$profile" ]]; then rm -rf -- "$profile"; fi
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 30); do
  ready=1
  for window in $WINDOWS; do
    curl -fsS "http://127.0.0.1:$(window_port "$window")/" >/dev/null || ready=0
  done
  ss -ltn "sport = :$ROSBRIDGE_PORT" | grep -q LISTEN || ready=0
  [[ "$ready" -eq 1 ]] && break
  for pid in "${HTTP_PIDS[@]}"; do kill -0 "$pid"; done
  kill -0 "$ROSBRIDGE_PID"
  kill -0 "$ROSAPI_PID"
  sleep 1
done
for window in $WINDOWS; do
  curl -fsS "http://127.0.0.1:$(window_port "$window")/" >/dev/null
done
ss -ltn "sport = :$ROSBRIDGE_PORT" | grep -q LISTEN

MAP_URL="http://127.0.0.1:$MAP_HTTP_PORT/?ds=rosbridge-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A$ROSBRIDGE_PORT"
DETECTION_URL="http://127.0.0.1:$DETECTION_HTTP_PORT/?ds=rosbridge-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A$ROSBRIDGE_PORT"
DASHBOARD_URL="http://127.0.0.1:$DASHBOARD_HTTP_PORT/?ds=rosbridge-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A$ROSBRIDGE_PORT"
if [[ "$NO_BROWSER" -eq 0 ]]; then
  # A snap-confined browser can only write inside its own snap tree. Pointing
  # --user-data-dir / --profile at ~/.cache makes it abort with
  # "Failed to create .../SingletonLock: Permission denied (13)" and no window
  # ever appears, while the bridge itself still reports ready. Ubuntu 22.04
  # ships Chromium *only* as a snap (/snap/bin/chromium), so this must be
  # handled for Chromium too, not just for Firefox.
  browser_profile_base() {
    local command_name="$1" snap_name="$2"
    local resolved path
    path="$(command -v "$command_name" 2>/dev/null || true)"
    resolved="$(readlink -f "$path" 2>/dev/null || true)"
    if [[ "$path" == /snap/bin/* || "$resolved" == /snap/* || "$resolved" == */snap-confine ]] \
       || { command -v snap >/dev/null && snap list "$snap_name" >/dev/null 2>&1; }; then
      local base="$HOME/snap/$snap_name/common/yzbot-foxglove"
      if mkdir -p "$base" 2>/dev/null; then
        printf '%s' "$base"
        return 0
      fi
    fi
    printf '%s' "$CACHE_BASE"
  }

  if command -v chromium >/dev/null; then
    BROWSER_KIND="chromium"
    BROWSER_COMMAND="chromium"
    PROFILE_BASE="$(browser_profile_base chromium chromium)"
  elif command -v chromium-browser >/dev/null; then
    BROWSER_KIND="chromium"
    BROWSER_COMMAND="chromium-browser"
    PROFILE_BASE="$(browser_profile_base chromium-browser chromium)"
  elif command -v firefox >/dev/null; then
    BROWSER_KIND="firefox"
    BROWSER_COMMAND="firefox"
    PROFILE_BASE="$(browser_profile_base firefox firefox)"
  else
    echo "No Ubuntu browser found; install Firefox or Chromium" >&2
    exit 7
  fi
  mkdir -p "$PROFILE_BASE"
  BROWSER_NICE="${FOXGLOVE_NICE:-19}"
  # test13 PERF-§5：每个窗口用固定 profile（profile-<window>-<archive>-<transport>），
  # 启动时只移除浏览器锁文件，不新建也不删除布局与缓存——避免每轮运行堆积 profile。
  TRANSPORT_TAG="rosbridge"
  # snap Chromium 的固有毛病：两个实例几乎同时启动时，后一个常在 Zygote 阶段崩掉
  #   FATAL:content/browser/zygote_host/zygote_host_impl_linux.cc:213
  #         Zygote process exited prematurely with exit code -1
  #   + Trace/breakpoint trap (core dumped)
  # 现场表现：只弹出一个窗口（曾经是"检测窗活着、看板窗没出来"），
  # 而 foxglove_bridge 仍报 ready —— 于是栈报就绪、选手却没有操作入口（2026-09-24 实跑复现）。
  # 因为无法保证 snap Chromium 不崩，这里改成「崩了能自愈 + 崩了不谎报就绪」：
  #   ① 看板优先启动（操作入口最关键）
  #   ② 错开间隔加大到 20 s
  #   ③ 每个窗口启动后校验进程存活，崩了换全新 profile 重试
  #   ④ 运行期持续监督，窗口掉了自动拉起
  #   ⑤ 有任何窗口最终起不来时打印 FOXGLOVE_BROWSER_FAILED，并在 ready 行里带 browser_degraded
  # 覆盖方式：FOXGLOVE_BROWSER_STAGGER_SEC / FOXGLOVE_BROWSER_MAX_RESTARTS / FOXGLOVE_BROWSER_CHECK_SEC
  BROWSER_STAGGER="${FOXGLOVE_BROWSER_STAGGER_SEC:-20}"
  case "$BROWSER_STAGGER" in ''|*[!0-9]*) BROWSER_STAGGER=20 ;; esac
  BROWSER_MAX_RESTARTS="${FOXGLOVE_BROWSER_MAX_RESTARTS:-3}"
  case "$BROWSER_MAX_RESTARTS" in ''|*[!0-9]*) BROWSER_MAX_RESTARTS=3 ;; esac
  BROWSER_CHECK_SEC="${FOXGLOVE_BROWSER_CHECK_SEC:-5}"
  case "$BROWSER_CHECK_SEC" in ''|*[!0-9]*) BROWSER_CHECK_SEC=5 ;; esac

  # 看板排在最前
  LAUNCH_ORDER=""
  for window in dashboard $WINDOWS; do
    case " $LAUNCH_ORDER " in
      *" $window "*) ;;
      *) LAUNCH_ORDER="$LAUNCH_ORDER $window" ;;
    esac
  done
  echo "Foxglove browser: kind=$BROWSER_KIND profile_base=$PROFILE_BASE nice=$BROWSER_NICE windows=$WINDOWS order=${LAUNCH_ORDER# } transport=$TRANSPORT_TAG stagger=${BROWSER_STAGGER}s max_restarts=$BROWSER_MAX_RESTARTS check=${BROWSER_CHECK_SEC}s"

  browser_profile_for() { # $1=窗口名 $2=轮次（0=首次，>0=重试用全新 profile）
    local name="$1" attempt="$2"
    if [[ "$attempt" -eq 0 ]]; then
      printf '%s' "$PROFILE_BASE/profile-$name-$ARCHIVE_HASH-$TRANSPORT_TAG"
    else
      printf '%s' "$PROFILE_BASE/profile-$name-$ARCHIVE_HASH-$TRANSPORT_TAG-r$attempt"
    fi
  }

  # 2026-10-14：曾因浏览器输出/缓存无限增长把根分区写满，机器 ENOSPC 宕机。
  # 浏览器窗口只靠存活检查判定，默认把 stdout/stderr 丢到 /dev/null；
  # 排障时 FOXGLOVE_BROWSER_LOG=1 才落文件，并自行盯住大小。
  browser_output() { # $1=窗口名
    if [[ "${FOXGLOVE_BROWSER_LOG:-0}" = "1" ]]; then
      printf '%s' "$STATE_DIR/browser-$1.log"
    else
      printf '%s' "/dev/null"
    fi
  }
  browser_cache_prefs() { # $1=profile，限制 Firefox 磁盘缓存与会话存储
    cat >"$1/user.js" <<'PREFS'
user_pref("browser.cache.disk.capacity", 51200);
user_pref("browser.cache.disk.smart_size.enabled", false);
user_pref("browser.cache.disk.enable", true);
user_pref("browser.sessionstore.interval", 120000);
user_pref("datareporting.healthreport.uploadEnabled", false);
PREFS
  }

  browser_start() { # $1=窗口名 $2=profile $3=1 表示先把 profile 清空（首次用干净目录）
    local name="$1" profile="$2" fresh="${3:-0}" url size position width height screen_x screen_y
    url="$(window_url "$name")"
    IFS='|' read -r size position <<<"$(window_geometry "$name")"
    if [[ "$fresh" = "1" ]]; then
      # 复用上一次运行留下的 Chromium profile，下一次启动会在 Zygote 阶段崩
      # （2026-09-24 实测 6/6：复用目录必崩，全新目录必活）。首次一律用干净目录。
      rm -rf "$profile"
    fi
    mkdir -p "$profile"
    browser_cache_prefs "$profile"
    rm -f "$profile/SingletonLock" "$profile/SingletonSocket" "$profile/SingletonCookie" 2>/dev/null || true
    if [[ "$BROWSER_KIND" == "chromium" ]]; then
      # 本 VM 无可用硬件 GL，Chromium 会把 WebGL 拉黑、Foxglove 的 3D 面板报
      # "Error creating WebGL context."；放行 SwiftShader 软件回退让面板能渲染。
      nice -n "$BROWSER_NICE" "$BROWSER_COMMAND" --user-data-dir="$profile" --no-first-run \
        --app="$url" --window-size="$size" --window-position="$position" \
        --ignore-gpu-blocklist --enable-unsafe-swiftshader \
        --disable-background-networking --disk-cache-size=52428800 \
        >>"$(browser_output "$name")" 2>&1 &
    else
      IFS=',' read -r width height <<<"$size"
      IFS=',' read -r screen_x screen_y <<<"$position"
      cat >"$profile/xulstore.json" <<EOF
{"chrome://browser/content/browser.xhtml":{"main-window":{"screenX":"$screen_x","screenY":"$screen_y","width":"$width","height":"$height","sizemode":"normal"}}}
EOF
      nice -n "$BROWSER_NICE" "$BROWSER_COMMAND" --no-remote --new-instance \
        --profile "$profile" "$url" \
        >>"$(browser_output "$name")" 2>&1 &
    fi
    printf '%s' "$!"
  }

  browser_survives() { # $1=pid $2=观察秒数
    local pid="$1" secs="$2" i
    for ((i = 0; i < secs * 2; i++)); do
      kill -0 "$pid" 2>/dev/null || return 1
      sleep 0.5
    done
    return 0
  }

  declare -A BROWSER_PID=()
  declare -A BROWSER_RESTARTS=()
  BROWSER_FAILED_WINDOWS=""
  # 清掉上一次运行留下的重试 profile（-r1/-r2…），它们只占磁盘、不再被用到
  for stale in "$PROFILE_BASE"/profile-*-r[0-9]*; do
    if [[ -d "$stale" ]]; then
      rm -rf "$stale"
    fi
  done
  stagger_first=1
  for name in $LAUNCH_ORDER; do
    if [[ "$stagger_first" -eq 0 ]] && [[ "$BROWSER_STAGGER" -gt 0 ]]; then
      echo "Waiting ${BROWSER_STAGGER}s before starting the next browser window ($name)"
      sleep "$BROWSER_STAGGER"
    fi
    stagger_first=0
    attempt=0
    while :; do
      profile="$(browser_profile_for "$name" "$attempt")"
      fresh=0
      if [[ "$attempt" -eq 0 ]]; then
        fresh=1
      fi
      pid="$(browser_start "$name" "$profile" "$fresh")"
      if browser_survives "$pid" "$BROWSER_CHECK_SEC"; then
        BROWSER_PID[$name]="$pid"
        BROWSER_RESTARTS[$name]="$attempt"
        BROWSER_PIDS+=("$pid")
        [[ "$attempt" -gt 0 ]] && echo "Browser window '$name' recovered on attempt $attempt (pid=$pid)"
        break
      fi
      echo "WARNING: browser window '$name' died within ${BROWSER_CHECK_SEC}s (attempt $attempt, pid=$pid); retrying with a fresh profile" >&2
      wait "$pid" 2>/dev/null || true
      attempt=$((attempt + 1))
      if [[ "$attempt" -gt "$BROWSER_MAX_RESTARTS" ]]; then
        BROWSER_FAILED_WINDOWS="$BROWSER_FAILED_WINDOWS $name"
        echo "ERROR: browser window '$name' could not be started after $BROWSER_MAX_RESTARTS retries" >&2
        break
      fi
      sleep 10
    done
  done
fi

{
  echo "archive_sha256=$ARCHIVE_HASH"
  echo "foxglove_windows=$WINDOWS"
  echo "browser_nice=${BROWSER_NICE:-unset}"
  for window in $WINDOWS; do
    echo "${window}_root=${WINDOW_ROOTS[$window]}"
    echo "${window}_url=$(window_url "$window")"
  done
  echo "http_pids=${HTTP_PIDS[*]}"
  echo "rosbridge_pid=$ROSBRIDGE_PID"
  echo "rosapi_pid=$ROSAPI_PID"
  echo "browser_pids=${BROWSER_PIDS[*]}"
  for window in $LAUNCH_ORDER; do
    echo "${window}_browser_pid=${BROWSER_PID[$window]:-none}"
    echo "${window}_browser_restarts=${BROWSER_RESTARTS[$window]:-0}"
  done
  echo "browser_failed=${BROWSER_FAILED_WINDOWS# }"
  echo "map_url=$MAP_URL"
  echo "detection_url=$DETECTION_URL"
  echo "dashboard_url=$DASHBOARD_URL"
} >"$STATE_DIR/runtime.env"

# 就绪口径：不再"只看端口"。任何浏览器窗口最终没起来都要显式说出来，
# 但仍然打印 OFFLINE_FOXGLOVE_READY（带 browser_degraded=1），
# 因为机器人本体与任务链不依赖浏览器 —— 让它继续跑、同时把问题暴露出来，
# 比整体拒绝启动更有价值（选手可改用 SSH 端口转发在外部浏览器打开面板）。
if [[ -n "${BROWSER_FAILED_WINDOWS// /}" ]]; then
  BROWSER_DEGRADED=1
  echo "FOXGLOVE_BROWSER_FAILED windows=${BROWSER_FAILED_WINDOWS# } started=${BROWSER_PIDS[*]:-none} hint=open_the_panel_from_another_machine_via_ssh_port_forward" >&2
else
  BROWSER_DEGRADED=0
fi

echo "OFFLINE_FOXGLOVE_READY windows=$WINDOWS browser_degraded=$BROWSER_DEGRADED \
nice=${BROWSER_NICE:-unset} \
map=$MAP_URL detection=$DETECTION_URL dashboard=$DASHBOARD_URL \
archive_sha256=$ARCHIVE_HASH"

# 运行期监督：
#   HTTP / rosbridge / rosapi 掉了 -> 整体退出（交给启动器按 FATAL 处理）
#   浏览器窗口掉了 -> 自动拉起（每个窗口最多 BROWSER_MAX_RESTARTS 次）
while true; do
  for pid in "${HTTP_PIDS[@]}"; do kill -0 "$pid" || exit 8; done
  kill -0 "$ROSBRIDGE_PID" || exit 8
  kill -0 "$ROSAPI_PID" || exit 8
  if [[ "${NO_BROWSER:-0}" -eq 0 ]]; then
    for name in $LAUNCH_ORDER; do
      current_pid="${BROWSER_PID[$name]:-}"
      [[ -z "$current_pid" ]] && continue
      if ! kill -0 "$current_pid" 2>/dev/null; then
        restarts="${BROWSER_RESTARTS[$name]:-0}"
        if [[ "$restarts" -lt "$BROWSER_MAX_RESTARTS" ]]; then
          restarts=$((restarts + 1))
          BROWSER_RESTARTS[$name]="$restarts"
          new_profile="$(browser_profile_for "$name" "$restarts")"
          echo "Browser window '$name' exited; restarting (attempt $restarts/$BROWSER_MAX_RESTARTS)" >&2
          new_pid="$(browser_start "$name" "$new_profile" 1)"
          if browser_survives "$new_pid" "$BROWSER_CHECK_SEC"; then
            BROWSER_PID[$name]="$new_pid"
            BROWSER_PIDS+=("$new_pid")
            echo "Browser window '$name' restarted (pid=$new_pid)" >&2
          else
            BROWSER_PID[$name]=""
            echo "Browser window '$name' could not be restarted; giving up" >&2
          fi
        fi
      fi
    done
  fi
  sleep 2
done
