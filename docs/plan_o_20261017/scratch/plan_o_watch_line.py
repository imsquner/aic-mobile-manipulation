#!/usr/bin/env python3
"""Plan O: 打印一行紧凑状态（供 watchdog 每 5 分钟追加到时间线日志）。

只读文件，不调用 ROS，故对正在跑的仿真是零负载。
"""
import json
import sys
import time
from pathlib import Path


def main():
    rec = Path(sys.argv[1])
    parts = [time.strftime("%H:%M:%S")]

    rounds = []
    p = rec / "results" / "rounds.jsonl"
    if p.exists():
        for line in p.read_text().splitlines():
            line = line.strip()
            if line:
                try:
                    rounds.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    ok = sum(1 for r in rounds if r.get("round_passed"))
    last = rounds[-1].get("round") if rounds else 0
    parts.append(f"rounds={len(rounds):>3}(last=r{last}) pass={ok} fail={len(rounds) - ok}")

    js = sorted((rec / "results").glob("avoidance_*.json"))
    if js:
        d = json.loads(js[-1].read_text())
        items = d.get("items", [])
        t = items[-1]["ended_s"] if items else 0
        parts.append(f"items={len(items):>4} t={t:.0f}s")
        for n, e in d["overall"].items():
            short = n.replace("moving_box_", "b")
            parts.append(f"{short}: min={e['min_clearance']:.3f} c={e['contact']}")

    mp = rec / "results" / "monitor_probe.json"
    if mp.exists():
        try:
            m = json.loads(mp.read_text())
            parts.append(f"mon_itv={m.get('interventions')}")
        except json.JSONDecodeError:
            pass

    print(" | ".join(parts), flush=True)


if __name__ == "__main__":
    main()
