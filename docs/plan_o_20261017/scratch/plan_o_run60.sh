#!/usr/bin/env bash
# Plan O: 一条命令启动「避障修复后的 60 轮验证」。
#   bash scratch/plan_o_run60.sh          # 默认 60 轮，tag=after60
#   bash scratch/plan_o_run60.sh 20 after20
#
# 做三件事：① 停掉旧的探针与栈；② 提交 Source 的 P0/P2 修复（prepare 脚本要求 Source 干净）；
#           ③ 调用 plan_o_measure.sh 建记录→起栈→起探针→跑 N 轮→停栈→打摘要。
set -uo pipefail

W=/home/ros/桌面/AIC_Competition_Work
S=/home/ros/AIC_Competition_Source
ROUNDS=${1:-60}
TAG=${2:-after60}
OLD=$W/run_records/20261017_001904_312e163_plan_o_avoidance_before

log() { printf '%s %s\n' "$(date '+%H:%M:%S')" "$*"; }

# ① 停旧（探针 + 旧驱动 + 旧栈）
log "停止旧探针、旧驱动与旧栈 ..."
for p in $(pgrep -f 'plan_o_avoidance_probe.py'); do [ "$p" != "$$" ] && kill -INT "$p"; done
# 关键：旧驱动若还活着，新驱动会瞬时 REFUSED 退出（measure 现已把驱动输出写进
# <rec>/logs/driver_wrapper.log 并在 10 秒后校验进程，但为稳妥仍在启动前清干净）。
for p in $(pgrep -f 'plan_d_round_driver.py'); do [ "$p" != "$$" ] && kill -INT "$p"; done
sleep 5
if pgrep -af 'plan_d_round_driver.py' >/dev/null 2>&1; then
  log "⚠️ 旧驱动仍在：pgrep -af plan_d_round_driver.py（建议 kill -9 后再跑）"
else
  log "驱动已清空 ✓"
fi
LP=$(cat "$OLD/checkpoints/launcher.pid" 2>/dev/null || true)
[ -n "$LP" ] && kill -INT "$LP"
sleep 20
if pgrep -a gzserver >/dev/null 2>&1; then
  log "⚠️ 仍有 gzserver 在跑；如启动失败请手动清理后再来：pgrep -a gzserver"
else
  log "环境干净 ✓"
fi
# 探针/驱动的收尾校验提示：跑完后若探针没被收掉，用下面这行手动收
log "提示：跑完后若仍有探针残留 → pkill -INT -f plan_o_avoidance_probe.py（JSON 是边跑边写的，不影响已采数据）"

# ② 提交 Source 修复
log "提交 Source 修复 ..."
git -C "$S" add src/yzbot/mybot_description/urdf/large_base_gazebo.xacro \
                src/yzbot/mybot/launch/gazebo_world2.launch.py \
                src/yzbot/bot_navigation/param/originbot_nav2_2.yaml 2>/dev/null
git -C "$S" commit -q -m "P0: odom TF 30->100Hz, robot_state_publisher 15->50Hz; collision_monitor min_points 6->3, time_before_collision 1.2->1.6" || true
if [ -n "$(git -C "$S" status --porcelain)" ]; then
  log "⚠️ Source 仍不干净，prepare 会拒绝；请先处理："
  git -C "$S" status --porcelain | head -20
  exit 2
fi
log "Source 干净 ✓ HEAD=$(git -C "$S" rev-parse --short HEAD)"

# ③ 跑 N 轮
log "开始 $ROUNDS 轮验证（tag=$TAG，预计约 $((ROUNDS * 10 / 3 / 60)) 小时）"
cd "$W" && exec bash scratch/plan_o_measure.sh "$TAG" "$ROUNDS"
