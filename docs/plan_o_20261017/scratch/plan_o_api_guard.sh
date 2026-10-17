#!/usr/bin/env bash
# API 失联早停守卫：连续 3 轮都是 CONNECTION_ERROR 类 driver_error，就把 driver 停掉。
# 目的：2026-10-17 15:06–18:54 曾因外部 LLM API 断线，24 轮全部 RESET/解析失败白烧 3.8 小时。
#
# 用法：bash scratch/plan_o_api_guard.sh <record_dir>
set -uo pipefail
REC=$1
log() { printf '%s [guard] %s\n' "$(date '+%H:%M:%S')" "$*"; }
log "start on $(basename "$REC")"
while pgrep -f "[p]lan_d_round_driver.py" >/dev/null 2>&1; do
  if [ -f "$REC/results/rounds.jsonl" ]; then
    N=$(python3 - "$REC/results/rounds.jsonl" <<'PY'
import json, sys
rows = []
for line in open(sys.argv[1]):
    line = line.strip()
    if not line:
        continue
    try:
        rows.append(json.loads(line))
    except json.JSONDecodeError:
        pass
tail = rows[-3:]
bad = sum(1 for r in tail if "CONNECTION_ERROR" in str(r.get("driver_error") or ""))
print(bad if len(tail) == 3 else 0)
PY
)
    if [ "${N:-0}" -ge 3 ]; then
      log "连续 3 轮 CONNECTION_ERROR ⇒ 停 driver"
      pkill -INT -f '[p]lan_d_round_[d]river.py'
      sleep 10
      log "已请求停止"
      exit 0
    fi
  fi
  sleep 60
done
log "driver 已退出，守卫结束"
