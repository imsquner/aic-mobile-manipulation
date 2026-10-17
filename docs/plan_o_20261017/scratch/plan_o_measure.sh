#!/usr/bin/env bash
# Plan O: 一条命令完成「N 轮避障量化测量」——起栈 → 起探针 → 驱动 N 轮 → 收尾汇总。
#
# 用法：  bash scratch/plan_o_measure.sh before 10      # 基线（改参数前）
#         bash scratch/plan_o_measure.sh after  10      # 改参数后
#
# 前置：Source/Work 的待提交改动会被本脚本先提交（prepare 脚本要求 Source 干净）。
# 产物： <新记录>/results/avoidance_<tag>.json （探针 10 Hz 全量 + 逐项统计）
#        <新记录>/results/rounds.jsonl        （驱动逐轮账本）
#        <新记录>/logs/avoidance_<tag>.log    （探针终端输出）
set -uo pipefail

W=/home/ros/桌面/AIC_Competition_Work
S=/home/ros/AIC_Competition_Source
TAG=${1:-before}
ROUNDS=${2:-10}
COMMIT=${3:-312e163}
cd "$W" || exit 1

log() { printf '%s %s\n' "$(date '+%H:%M:%S')" "$*"; }

# ---------- 0) 预检 ----------
log "== Plan O 测量开始 tag=$TAG rounds=$ROUNDS =="
df -h /home | tail -1
LP=$(cat "$W/scratch/"plan_*_record.txt 2>/dev/null | head -1)
if pgrep -a gzserver >/dev/null 2>&1; then
  log "检测到已有 gzserver 在跑；如需干净测量请先停栈（见 plan_d_verify_idle.py）"
fi

# ---------- 1) 先提交待提交改动（prepare 要求 Source 干净） ----------
log "提交待提交改动..."
git -C "$S" add src/yzbot/aic_task/aic_task/planning/allocation.py \
               src/yzbot/aic_task/aic_task/task_input/mapping_contract.py \
               src/yzbot/aic_task/aic_task/placement/zone_slots/geometry.py \
               src/yzbot/aic_task/config/scenes/world2/points.yaml \
               src/yzbot/aic_task/config/scenes/world2/entities.yaml \
               src/yzbot/aic_task/config/scenes/world2/obstacles.yaml 2>/dev/null
git -C "$S" commit -q -m "Plan N/O: moving-obstacle parking corridors, box2 speed, zone B/C move, slot-boundary override, mapping-clause hygiene" || true

git -C "$W" add plan_n_changes.md \
               runtime/312e163/dev_ws/src/yzbot/aic_task/config/scenes/world2/points.yaml \
               runtime/312e163/dev_ws/src/yzbot/aic_task/config/scenes/world2/entities.yaml \
               runtime/312e163/dev_ws/src/yzbot/aic_task/config/scenes/world2/obstacles.yaml \
               run_records/20261015_111808_312e163_plan_d_30rounds_random/checkpoints/start.sh \
               run_records/20261014_233609_312e163_dynamic_acceptance/checkpoints/start.sh \
               scratch/plan_l_mapping_noise_check.py scratch/plan_m_corridor_check.py \
               scratch/plan_n_translate_zone_slots.py scratch/plan_o_avoidance_probe.py \
               scratch/plan_o_measure.sh 2>/dev/null
git -C "$W" commit -q -m "Plan N/O: avoidance probe + measurement runner; zone move and config sync" || true
log "Source HEAD=$(git -C "$S" rev-parse --short HEAD)  Work HEAD=$(git -C "$W" rev-parse --short HEAD)"

# ---------- 2) 建记录（会把 scratch/plan_d_record.txt 指向新记录） ----------
REC=$(python3 scratch/prepare_plan_d_30rounds.py --commit "$COMMIT" \
        --purpose "plan_o_avoidance_${TAG}" 2>&1 | tail -1)
if [ ! -d "$REC" ]; then log "建记录失败：$REC"; exit 2; fi
log "记录=$REC"

# ---------- 3) 起栈并等 READY ----------
nohup bash scratch/plan_d_launch_with_retry.sh 3 300 > "scratch/launch_wrapper_o_${TAG}.log" 2>&1 &
for i in $(seq 1 40); do
  sleep 10
  L=$(ls -t "$REC"/logs/launch_attempt_*.log 2>/dev/null | head -1)
  [ -z "$L" ] && continue
  if grep -q "SYSTEM READY" "$L" 2>/dev/null; then log "栈已就绪（第 ${i}0 秒）"; break; fi
  grep -q "FATAL" "$L" 2>/dev/null && log "启动失败（wrapper 会自动重试）：$(grep FATAL "$L" | tail -1)"
done
grep -q "SYSTEM READY" "$(ls -t "$REC"/logs/launch_attempt_*.log | head -1)" || { log "未就绪，退出"; exit 3; }

# ---------- 4) 起探针 ----------
# 注意：`set -u` 下 source ROS 的 setup.bash 会直接中断整个脚本（已知坑），必须先 set +u。
set +u
source /opt/ros/humble/setup.bash
source "$W/runtime/$COMMIT/dev_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
nohup python3 -u scratch/plan_o_avoidance_probe.py \
      --out "$REC/results/avoidance_${TAG}.json" > "$REC/logs/avoidance_${TAG}.log" 2>&1 &
PROBE=$!
sleep 5
log "探针 pid=$PROBE（输出 $REC/logs/avoidance_${TAG}.log）"

# ---------- 5) 跑 N 轮 ----------
# 驱动输出留档（原来是 /dev/null）：若已有驱动在跑，wrapper 会立刻 REFUSED 退出，
# 不记日志就会"看起来跑完了但其实 0 轮"，这里留证据并在 10 秒后校验进程是否真的在。
nohup bash scratch/plan_d_start_driver.sh "$COMMIT" "$REC/logs/driver.log" \
      --start-round 0 --rounds "$ROUNDS" --continue-on-problems \
      > "$REC/logs/driver_wrapper.log" 2>&1 &
DRV=$!
log "驱动 pid=$DRV，开始跑 $ROUNDS 轮（wrapper 日志 $REC/logs/driver_wrapper.log）"
sleep 10
if ! pgrep -f 'plan_d_round_driver.py' >/dev/null 2>&1; then
  log "⚠️ 驱动未起来！看 $REC/logs/driver_wrapper.log（常见：REFUSED 已有驱动 / 记录指针不对）"
fi
while kill -0 "$DRV" 2>/dev/null; do
  sleep 30
  python3 scratch/plan_g_progress.py --record "$REC" --total "$ROUNDS" 2>/dev/null | sed -n '2,4p'
done
log "驱动结束"

# ---------- 6) 收尾 ----------
kill -INT "$PROBE" 2>/dev/null; sleep 6
LP=$(cat "$REC/checkpoints/launcher.pid" 2>/dev/null || true)
[ -n "$LP" ] && kill -INT "$LP"; sleep 20
python3 scratch/plan_d_verify_idle.py --record "$REC"
python3 scratch/plan_g_progress.py --record "$REC" --total "$ROUNDS" | head -4

log "== 结果 =="
log "探针：$REC/results/avoidance_${TAG}.json"
log "逐项摘要（最后 8 行）："
tail -8 "$REC/logs/avoidance_${TAG}.log"
python3 - "$REC/results/avoidance_${TAG}.json" <<'PY'
import json, sys
from pathlib import Path
p = Path(sys.argv[1])
if not p.exists():
    print('（探针结果文件不存在）'); raise SystemExit
data = json.loads(p.read_text())
print('overall:')
for name, e in data['overall'].items():
    print(f"  {name}: min={e['min_clearance']} near(<{data['args']['near']})={e['near']} "
          f"danger(<{data['args']['danger']})={e['danger']} contact={e['contact']} "
          f"speed_at_min={e['speed_at_min']}")
print(f"监视器触发总数={data['monitor']['activations_total']}  时长={data['monitor']['durations_sec']}")
print(f"作业项数={len(data['items'])}")
for it in data['items']:
    line = ' '.join(f"{n}:min={v['min_clearance']}" for n, v in it['boxes'].items())
    print(f"  item{it['item']:>2} ({it['end_reason']}) monitor={it['monitor_activations']} {line}")
PY
log "== 测量结束（tag=$TAG）=="
