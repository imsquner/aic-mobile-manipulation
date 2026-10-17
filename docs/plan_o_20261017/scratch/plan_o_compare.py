#!/usr/bin/env python3
"""Plan O: 避障 before/after 对照（读两份探针 JSON，输出小表）。

用法：
  python3 scratch/plan_o_compare.py \
    run_records/20261017_001904_312e163_plan_o_avoidance_before/results/avoidance_before.json \
    <新记录>/results/avoidance_after60.json \
    [<before 记录目录> <after 记录目录>]      # 可选：附带比较 60 轮成功率

判定规则（老板关注："不要撞移动障碍"）：
  - 最小表面间隙：**越大越好**（负数=足迹重叠=真撞）
  - `<0.30 m` / `<0.15 m` 次数：越小越好
  - `contact`（重叠）次数：**必须为 0**
  - 碰撞监视器触发次数：允许上升（说明它真的在干活），但**不应暴涨**
  - 成功率：不低于 Plan J 的 14/18 = 77.8%
"""
from pathlib import Path
import json
import statistics
import sys

KEYS = (
    ("min_clearance", "最小间隙(m)", "up"),
    ("near", "<0.30 次数", "down"),
    ("danger", "<0.15 次数", "down"),
    ("contact", "contact 次数", "down"),
)


def load(path):
    return json.loads(Path(path).read_text())


def verdict(before, after, better):
    if not isinstance(before, (int, float)) or not isinstance(after, (int, float)):
        return ""
    if before == after:
        return "持平"
    improved = after > before if better == "up" else after < before
    return "改善 ✓" if improved else "变差 ✗"


def rounds(path):
    ledger = Path(path) / "results" / "rounds.jsonl"
    if not ledger.exists():
        return None
    rows = [json.loads(line) for line in ledger.read_text().splitlines() if line.strip()]
    passed = sum(1 for row in rows if row.get("round_passed"))
    errs = sum(1 for row in rows if row.get("driver_error"))
    return len(rows), passed, errs


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    before, after = load(sys.argv[1]), load(sys.argv[2])
    print("== before vs after ==")
    for box in sorted(set(before["overall"]) | set(after["overall"])):
        for key, label, better in KEYS:
            b = before["overall"].get(box, {}).get(key)
            a = after["overall"].get(box, {}).get(key)
            print(f"{box + ' ' + label:<30}{str(b):>10}{str(a):>12}   {verdict(b, a, better)}")
        bs = before["overall"].get(box, {}).get("speed_at_min")
        as_ = after["overall"].get(box, {}).get("speed_at_min")
        print(f"{box + ' 最小间隙时车速(m/s)':<30}{str(bs):>10}{str(as_):>12}")

    print(f"\n监视器触发次数：before={before['monitor']['activations_total']} "
          f"after={after['monitor']['activations_total']}")
    print(f"作业项数：before={len(before['items'])} after={len(after['items'])}")
    for tag, data in (("before", before), ("after", after)):
        for box in sorted(data["overall"]):
            values = [it["boxes"].get(box, {}).get("min_clearance") for it in data["items"]]
            values = [v for v in values if isinstance(v, (int, float))]
            if values:
                print(f"{tag} {box}: 每项最小间隙 中位={statistics.median(values):.3f} "
                      f"最差={min(values):.3f} 最好={max(values):.3f} (n={len(values)})")

    if len(sys.argv) >= 5:
        for label, path in (("before", sys.argv[3]), ("after", sys.argv[4])):
            stats = rounds(path)
            if stats:
                total, passed, errs = stats
                rate = 100.0 * passed / total if total else 0.0
                print(f"{label} 记录 {Path(path).name}: {passed}/{total} = {rate:.1f}% "
                      f"(driver_error {errs})")
        print("参考：Plan J 基线 14/18 = 77.8%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
