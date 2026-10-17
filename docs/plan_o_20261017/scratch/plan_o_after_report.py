#!/usr/bin/env python3
"""Plan O: 最终 before/after 汇总（可跨多个 after 记录，且剔除无效轮）。

为什么需要它（相对 scratch/plan_o_compare.py 的增强）：
  1. `plan_o_compare.py` 直接比 overall 计数，但 after 运行中**失败的作业项**会让机器人
     停在走廊里被方块反复扫过，把 `contact` 灌成总量的 ~2/3。所以必须**按作业项结局拆分**。
  2. 本机 nav2 构建**不发布** `/collision_monitor_state`（头文件无 state_pub_），
     所以探针里的 `监视器触发` 恒为 0，不能当判据；改用 cmd_vel vs cmd_vel_safe 探针。
  3. 一次跑不完时会出现 `driver_error` 的轮次（RESET_FAILED 等），这些**不是任务失败**，
     必须剔除，否则成功率被严重低估。

用法：
  python3 scratch/plan_o_after_report.py <before_rec> <after_rec> [<after_rec> ...]
"""
import json
import re
import sys
from pathlib import Path


def load_rounds(rec):
    p = Path(rec) / "results" / "rounds.jsonl"
    if not p.exists():
        return []
    rows = []
    for line in p.read_text().splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return rows


def valid_rounds(rows):
    """剔除 driver_error 轮（栈级故障，不是任务结果）。"""
    return [r for r in rows if not r.get("driver_error")]


def probe(rec):
    js = sorted((Path(rec) / "results").glob("avoidance_*.json"))
    if not js:
        return None
    return json.loads(js[-1].read_text())


def split_items(d):
    out = {}
    for lbl, keep in (("succ", lambda r: r == "state->ITEM_SUCCEEDED"),
                      ("fail", lambda r: r == "state->ITEM_FAILED"),
                      ("all", lambda r: True)):
        agg = {}
        for n in d["overall"]:
            xs = [it for it in d["items"] if keep(it["end_reason"])]
            c = sum(it["boxes"][n]["contact"] for it in xs)
            ms = [it["boxes"][n]["min_clearance"] for it in xs
                  if it["boxes"][n]["min_clearance"] is not None]
            agg[n] = {"items": len(xs), "contact": c, "min": min(ms) if ms else None}
        out[lbl] = agg
    return out


def nav2_errors(rec):
    """从 nav2.log 抽 TF 相关证据（取最大的那个 run_*）。"""
    logs = sorted(Path(rec).glob("logs/run_*/nav2.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        return None
    txt = logs[-1].read_text(errors="replace")
    gaps = [abs(float(a) - float(b)) * 1000 for a, b in
            re.findall(r"Requested time ([0-9.]+) but the latest data is at time ([0-9.]+)", txt)]
    span = 0
    ts = re.findall(r"\[(1792\d+)\.\d+\]", txt)
    if len(ts) > 1:
        span = int(ts[-1]) - int(ts[0])
    return {
        "file": logs[-1].name,
        "span_s": span,
        "cm_extrap": len(re.findall(r"collision_monitor.*extrapolation", txt)),
        "laser_link_fail": len(re.findall(r'Failed to get "laser_link"', txt)),
        "msg_filter_drop": len(re.findall(r"Message Filter dropping", txt)),
        "gap_median_ms": sorted(gaps)[len(gaps) // 2] if gaps else None,
        "gap_max_ms": max(gaps) if gaps else None,
    }


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        raise SystemExit(2)
    before, afters = sys.argv[1], sys.argv[2:]

    # ---------- 轮次 ----------
    all_rows = [(r, "before") for r in load_rounds(before)]
    for a in afters:
        all_rows += [(r, "after") for r in load_rounds(a)]
    print("=" * 72)
    print("轮次成功率（已剔除 driver_error 的栈级故障轮）")
    print("=" * 72)
    for tag, sel in (("before", "before"), ("after", "after")):
        rows = [r for r, t in all_rows if t == sel]
        v = valid_rounds(rows)
        inv = len(rows) - len(v)
        ok = sum(1 for r in v if r.get("round_passed"))
        rate = 100.0 * ok / len(v) if v else 0.0
        extra = f"（另有 {inv} 轮栈级故障已剔除）" if inv else ""
        print(f"  {tag:<6} 有效轮={len(v):>3} 通过={ok:>3} 失败={len(v) - ok:>3} "
              f"成功率={rate:5.1f}%  {extra}")

    # ---------- 探针 ----------
    print()
    print("=" * 72)
    print("避障探针（按作业项结局拆分；contact = 车足迹与方块重叠采样数）")
    print("=" * 72)
    data = {}
    for tag, recs in (("before", [before]), ("after", afters)):
        merged = None
        tot_t = 0.0
        for rec in recs:
            d = probe(rec)
            if d is None:
                continue
            tot_t += d["items"][-1]["ended_s"] if d["items"] else 0.0
            if merged is None:
                merged = d
            else:  # 合并：逐项拼接，overall 相加
                for it in d["items"]:
                    it = dict(it)
                    it["item"] = len(merged["items"]) + 1
                    merged["items"].append(it)
                for n in merged["overall"]:
                    for k in ("near", "danger", "contact"):
                        merged["overall"][n][k] += d["overall"][n][k]
                    a, b = merged["overall"][n]["min_clearance"], d["overall"][n]["min_clearance"]
                    merged["overall"][n]["min_clearance"] = min(x for x in (a, b) if x is not None)
        if merged is None:
            print(f"  {tag}: 无探针数据")
            continue
        data[tag] = (merged, tot_t)
        sp = split_items(merged)
        print(f"\n  --- {tag}  运行={tot_t:.0f}s ({tot_t / 60:.1f}min) 作业项={len(merged['items'])}")
        for lbl, name in (("succ", "仅 ITEM_SUCCEEDED"), ("fail", "仅 ITEM_FAILED")):
            for n, a in sp[lbl].items():
                rate = 1000.0 * a["contact"] / tot_t if tot_t else 0
                print(f"      {name:<18} {n}: items={a['items']:>3} contact={a['contact']:>7} "
                      f"({rate:7.1f}/1000s) min={a['min'] if a['min'] is None else round(a['min'], 4)}")

    # ---------- 结论行 ----------
    if "before" in data and "after" in data:
        print()
        print("=" * 72)
        print("判据对照（成功项内、按时间归一）")
        print("=" * 72)
        sb = split_items(data["before"][0])["succ"]
        sa = split_items(data["after"][0])["succ"]
        tb, ta = data["before"][1], data["after"][1]
        for n in sb:
            rb = 1000.0 * sb[n]["contact"] / tb if tb else 0
            ra = 1000.0 * sa[n]["contact"] / ta if ta else 0
            if rb == 0 and ra == 0:
                verdict = "双方均为 0"
            elif ra == 0:
                verdict = "改善 ✓"
            elif rb == 0:
                verdict = "变差 ✗"
            else:
                verdict = f"{rb / ra:.2f}× 更好 ✓" if ra < rb else f"{ra / rb:.2f}× 更差 ✗"
            print(f"  {n:<14} contact/1000s: {rb:8.1f} → {ra:8.1f}   {verdict}")
            print(f"  {'':<14} 最小间隙:        {sb[n]['min']:.4f} → {sa[n]['min']:.4f}"
                  f"   {'改善 ✓' if (sa[n]['min'] or -9) > (sb[n]['min'] or -9) else '变差 ✗'}")

    # ---------- nav2 层证据 ----------
    print()
    print("=" * 72)
    print("nav2 层证据（TF 时序）")
    print("=" * 72)
    for tag, recs in (("before", [before]), ("after", afters)):
        for rec in recs:
            e = nav2_errors(rec)
            if e:
                print(f"  {tag:<6} {Path(rec).name}")
                print(f"          span={e['span_s']}s  collision_monitor 未来外推={e['cm_extrap']}  "
                      f"laser_link 失败={e['laser_link_fail']}  costmap 丢扫描={e['msg_filter_drop']}")
                print(f"          外推差值 中位={e['gap_median_ms']}ms 最大={e['gap_max_ms']}ms")


if __name__ == "__main__":
    main()
