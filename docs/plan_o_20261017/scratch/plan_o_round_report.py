#!/usr/bin/env python3
"""Plan O: 逐轮紧凑报告（只读文件，绝不调用 ROS，故对正在跑的栈零负载）。

用法：
    python3 scratch/plan_o_round_report.py <record_dir> [--items N]

输出：
  1) 逐轮通过/失败/落点核验/重试（来自 results/rounds.jsonl）
  2) 探针累计指标 + 出现过接触的作业项清单（来自 results/avoidance_*.json）
"""
import argparse
import json
from pathlib import Path


def load_jsonl(path):
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("--items", type=int, default=25,
                    help="最多列出多少个『有接触』的作业项")
    args = ap.parse_args()
    rec = Path(args.record)

    rounds = load_jsonl(rec / "results" / "rounds.jsonl")
    print(f"== 轮次台账 ({rec.name}) ==")
    if not rounds:
        print("  （尚无轮次：results/rounds.jsonl 为空/不存在）")
    else:
        passed = sum(1 for r in rounds if r.get("round_passed"))
        n = len(rounds)
        print(f"  已跑 {n} 轮：通过 {passed} / 失败 {n - passed} "
              f"→ 成功率 {100.0 * passed / n:.1f}%")
        for r in rounds:
            c = r.get("competition") or {}
            wall = r.get("wall_execution_sec")
            print(f"  r{r.get('round'):>2} passed={str(r.get('round_passed')):<5} "
                  f"ok={c.get('completed')}/{c.get('attempted')} fail={c.get('failed')} "
                  f"strict={c.get('strict_inside_count')}/{c.get('strict_inside_total')} "
                  f"retry={r.get('retries')} {c.get('state')} "
                  f"{'' if wall is None else f'{wall:.0f}s'}")

    jsons = sorted((rec / "results").glob("avoidance_*.json"))
    print()
    print("== 探针（避障） ==")
    if not jsons:
        print("  （尚无探针 JSON）")
        return
    data = json.loads(jsons[-1].read_text())
    near = data["args"]["near"]
    items = data.get("items", [])
    mon = data.get("monitor", {})
    print(f"  文件={jsons[-1].name} 作业项={len(items)} "
          f"监视器触发={mon.get('activations_total')} 时长={mon.get('durations_sec')}")
    for name, e in data["overall"].items():
        print(f"  {name}: min={e['min_clearance']} <{near}={e['near']} "
              f"danger={e['danger']} contact={e['contact']} "
              f"speed_at_min={e['speed_at_min']}")

    touched = []
    for it in items:
        for name, v in it["boxes"].items():
            if v.get("contact"):
                touched.append((it["item"], name, v["min_clearance"], v["contact"]))
    print(f"  有接触的作业项：{len(touched)} 个（下面最多列 {args.items} 条）")
    for item, name, mc, ct in touched[-args.items:]:
        print(f"    item{item:>4} {name}: min={mc} contact={ct}")

    mins = {}
    for it in items:
        for name, v in it["boxes"].items():
            mc = v.get("min_clearance")
            if mc is None:
                continue
            if name not in mins or mc < mins[name][0]:
                mins[name] = (mc, it["item"])
    print("  各箱最小间隙（全程最小）:")
    for name, (mc, item) in mins.items():
        print(f"    {name}: {mc} @ item{item}")


if __name__ == "__main__":
    main()
