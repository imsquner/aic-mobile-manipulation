#!/usr/bin/env python3
"""Plan O: 把 run_records/ 下所有实验的细节导出成「明细文档 + CSV」。

产出（默认写到当前目录）：
  实验全记录.md    —— 人读：每次实验的时间/配置/逐轮耗时与结果/失败原因/接触统计
  rounds_all.csv   —— 机器读：一行一轮（跨全部记录）
  items_all.csv    —— 机器读：一行一个探针作业项（跨全部记录，含逐箱 min/near/danger/contact）
  runs_all.csv     —— 机器读：一行一次实验（配置、轮数、成功率、接触汇总）

用法：
  python3 scratch/plan_o_experiment_log.py [--out DIR]
"""
import argparse
import csv
import json
import math
from datetime import datetime, timezone, timedelta
from pathlib import Path

CST = timezone(timedelta(hours=8))

# 每次实验的配置（按记录名关键词匹配，顺序敏感：先匹配到的先算）
CONFIG_RULES = [
    ("avoidance_before", "改前基线：TF 30/15Hz，tbc 1.2(无效键 min_points=6)，box1 0.570 m/s，box2 0.094 m/s"),
    ("after60v40a", "TF 100/50Hz，tbc 1.2，box1 0.400 m/s，box2 0.094（本批 API 断线，0 有效轮）"),
    ("after60v40b1", "TF 100/50Hz，tbc 1.2，box1 0.400 m/s，box2 0.094"),
    ("after60v40b2", "TF 100/50Hz，tbc 1.2，box1 0.400 m/s，box2 0.094"),
    ("after20v40", "TF 100/50Hz，tbc 1.2，box1 0.400 m/s，box2 0.094"),
    ("after20", "TF 100/50Hz，tbc 1.2，box1 0.570 m/s，box2 0.094"),
    ("after60b", "TF 100/50Hz，tbc 1.6，box1 0.570 m/s，box2 0.094"),
    ("after60", "TF 100/50Hz，tbc 1.6，box1 0.570 m/s，box2 0.094"),
    ("plan_j", "Plan J：tbc 1.6 之前，TF 未提频"),
    ("plan_h", "Plan H：headless 18 轮"),
    ("plan_g", "Plan G：54 轮大盘"),
    ("plan_e", "Plan E：18 轮"),
    ("plan_d", "Plan D：30 轮随机"),
]


def cfg_of(name):
    for key, desc in CONFIG_RULES:
        if key in name:
            return desc
    return "（未知配置）"


def hm(ts):
    if not ts:
        return ""
    return datetime.fromtimestamp(ts, CST).strftime("%m-%d %H:%M:%S")


def load_jsonl(p):
    if not p.exists():
        return []
    out = []
    for line in p.read_text().splitlines():
        line = line.strip()
        if line:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def fnum(v, nd=3):
    return "—" if v is None else f"{v:.{nd}f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--records", default="run_records")
    ap.add_argument("--out", default=".")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    recs = []
    for d in sorted(Path(args.records).iterdir()):
        if not d.is_dir():
            continue
        rp = d / "results"
        if not rp.exists():
            continue
        rounds = load_jsonl(rp / "rounds.jsonl")
        ajs = sorted(rp.glob("avoidance_*.json"))
        mp = rp / "monitor_probe.json"
        if not rounds and not ajs:
            continue
        recs.append({"dir": d, "name": d.name, "rounds": rounds, "ajs": ajs,
                     "mon": json.loads(mp.read_text()) if mp.exists() else None})

    # ---------- CSV: runs ----------
    with (out / "runs_all.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["记录", "配置", "开始", "结束", "记录行数", "有效轮", "通过", "失败", "成功率%",
                    "耗时中位s", "耗时max_s", "栈故障轮", "探针作业项", "box1_min", "box1_contact",
                    "box2_min", "box2_contact", "监视器干预"])
        for r in recs:
            V = [x for x in r["rounds"] if not x.get("driver_error")]
            W = [x["wall_execution_sec"] for x in V if x.get("wall_execution_sec")]
            ok = sum(1 for x in V if x.get("round_passed"))
            items = []
            overall = {}
            for j in r["ajs"]:
                d = json.loads(j.read_text())
                items += d.get("items", [])
                overall = d.get("overall", {}) or overall
            t0 = next((x.get("started_at") or x.get("finished_at") for x in r["rounds"]), None)
            t1 = next((x.get("finished_at") for x in reversed(r["rounds"])), None)
            b1, b2 = overall.get("moving_box_1", {}), overall.get("moving_box_2", {})
            w.writerow([r["name"], cfg_of(r["name"]), hm(t0), hm(t1), len(r["rounds"]), len(V), ok,
                        len(V) - ok,
                        f"{100 * ok / len(V):.1f}" if V else "",
                        f"{sorted(W)[len(W) // 2]:.0f}" if W else "",
                        f"{max(W):.0f}" if W else "",
                        len(r["rounds"]) - len(V), len(items),
                        fnum(b1.get("min_clearance"), 4), b1.get("contact", ""),
                        fnum(b2.get("min_clearance"), 4), b2.get("contact", ""),
                        (r["mon"] or {}).get("interventions", "")])

    # ---------- CSV: rounds ----------
    with (out / "rounds_all.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["记录", "配置", "轮", "规则", "开始", "结束", "耗时s", "通过", "state",
                    "完成", "尝试", "失败数", "失败物块", "失败明细", "重试", "补充轮", "连续问题", "driver_error"])
        for r in recs:
            for x in r["rounds"]:
                c = x.get("competition") or {}
                bad = [f"{i.get('cube')}: {i.get('detail')}" for i in (c.get("items") or []) if not i.get("success")]
                w.writerow([r["name"], cfg_of(r["name"]), x.get("round"), x.get("rule_id"),
                            hm(x.get("started_at")), hm(x.get("finished_at")),
                            f"{x['wall_execution_sec']:.0f}" if x.get("wall_execution_sec") else "",
                            x.get("round_passed"), c.get("state"), c.get("completed"), c.get("attempted"),
                            c.get("failed"), ";".join(c.get("failed_cubes") or []), " | ".join(bad),
                            x.get("retries"), x.get("supplementary"), x.get("consecutive_problems"),
                            x.get("driver_error") or ""])

    # ---------- CSV: items ----------
    with (out / "items_all.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["记录", "作业项", "起s", "止s", "耗时s", "结局", "监视器触发",
                    "box1_min", "box1_near", "box1_danger", "box1_contact",
                    "box2_min", "box2_near", "box2_danger", "box2_contact"])
        for r in recs:
            for j in r["ajs"]:
                d = json.loads(j.read_text())
                for it in d.get("items", []):
                    b1 = it["boxes"].get("moving_box_1", {})
                    b2 = it["boxes"].get("moving_box_2", {})
                    w.writerow([r["name"], it.get("item"), it.get("started_s"), it.get("ended_s"),
                                f"{(it.get('ended_s') or 0) - (it.get('started_s') or 0):.1f}",
                                it.get("end_reason"), it.get("monitor_activations"),
                                fnum(b1.get("min_clearance"), 4), b1.get("near"), b1.get("danger"), b1.get("contact"),
                                fnum(b2.get("min_clearance"), 4), b2.get("near"), b2.get("danger"), b2.get("contact")])

    # ---------- Markdown ----------
    L = ["# Plan O 实验全记录（明细）", "",
         f"生成时间：{datetime.now(CST).strftime('%Y-%m-%d %H:%M:%S')}　记录数：{len(recs)}", "",
         "> 口径：`contact`/`near`/`danger`/`min_clearance` 来自只读探针（约 25 Hz 采样）；",
         "> `contact` = 车足迹与方块**重叠**的采样数（>0 即为真撞）；`min_clearance` 为全程最小表面间隙（负=重叠）。",
         "> 栈故障轮（`driver_error`，复位死锁 / API 断线）不计入有效轮。", ""]

    # 总表
    L += ["## 0. 总览", "",
          "| 记录 | 配置 | 有效轮 | 通过 | 成功率 | 耗时中位 | max | 栈故障 | box1 min | box1 c/1000s(成功项) | box1 contact(全部) | box2 min | box2 c/1000s(成功项) | box2 contact(全部) |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in recs:
        V = [x for x in r["rounds"] if not x.get("driver_error")]
        W = [x["wall_execution_sec"] for x in V if x.get("wall_execution_sec")]
        ok = sum(1 for x in V if x.get("round_passed"))
        overall = {}
        for j in r["ajs"]:
            overall = json.loads(j.read_text()).get("overall", {}) or overall
        b1, b2 = overall.get("moving_box_1", {}), overall.get("moving_box_2", {})
        its, t_run = [], 0.0
        for j in r["ajs"]:
            dd = json.loads(j.read_text())
            its += dd.get("items", [])
            t_run += (dd.get("items") or [{}])[-1].get("ended_s", 0.0)
        norm = {}
        for n in ("moving_box_1", "moving_box_2"):
            c = sum(it["boxes"][n]["contact"] for it in its if it["end_reason"] == "state->ITEM_SUCCEEDED")
            norm[n] = f"{1000 * c / t_run:.1f}" if t_run else "—"
        L.append(f"| `{r['name']}` | {cfg_of(r['name'])} | {len(V)} | {ok} | "
                 f"{100 * ok / len(V):.1f}% | {sorted(W)[len(W) // 2]:.0f}s | {max(W):.0f}s | "
                 f"{len(r['rounds']) - len(V)} | {fnum(b1.get('min_clearance'), 4)} | {norm['moving_box_1']} | "
                 f"{b1.get('contact', '—')} | {fnum(b2.get('min_clearance'), 4)} | {norm['moving_box_2']} | "
                 f"{b2.get('contact', '—')} |" if V else
                 f"| `{r['name']}` | {cfg_of(r['name'])} | 0 | 0 | — | — | — | {len(r['rounds'])} | — | — | — | — | — | — |")

    # 逐记录明细
    for r in recs:
        V = [x for x in r["rounds"] if not x.get("driver_error")]
        W = [x["wall_execution_sec"] for x in V if x.get("wall_execution_sec")]
        ok = sum(1 for x in V if x.get("round_passed"))
        t0 = next((x.get("started_at") or x.get("finished_at") for x in r["rounds"]), None)
        t1 = next((x.get("finished_at") for x in reversed(r["rounds"])), None)
        L += ["", f"## `{r['name']}`", "",
              f"- 配置：**{cfg_of(r['name'])}**",
              f"- 墙钟：{hm(t0)} → {hm(t1)}",
              f"- 轮次：记录 {len(r['rounds'])}，有效 {len(V)}，通过 {ok}"
              + (f"，成功率 **{100 * ok / len(V):.1f}%**" if V else "")
              + (f"，栈故障 {len(r['rounds']) - len(V)}" if len(r['rounds']) > len(V) else ""),
              (f"- 单轮耗时：min {min(W):.0f}s / 中位 {sorted(W)[len(W) // 2]:.0f}s / mean {sum(W) / len(W):.0f}s / max {max(W):.0f}s"
               if W else "- 单轮耗时：（无）")]

        # 探针
        items, overall, mon = [], {}, {}
        for j in r["ajs"]:
            d = json.loads(j.read_text())
            items += d.get("items", [])
            overall = d.get("overall", {}) or overall
            mon = d.get("monitor", {}) or mon
        if items:
            t_run = max((it.get("ended_s") or 0) for it in items)
            L += ["", f"- 探针：作业项 {len(items)}，运行 {t_run:.0f}s；"
                      f"监视器状态话题触发={mon.get('activations_total', '—')}（⚠️本构建无该话题，恒 0，不可作判据）"
                      + (f"；cmd_vel 实测干预={r['mon'].get('interventions')} 次" if r["mon"] else ""), "",
                  "| 方块 | 全程最小间隙 | <0.30 | <0.15(danger) | contact | 接触时车速 | 成功项 contact | 失败项 contact |",
                  "|---|---|---|---|---|---|---|---|"]
            for n in ("moving_box_1", "moving_box_2"):
                e = overall.get(n, {})
                sc = sum(it["boxes"][n]["contact"] for it in items if it["end_reason"] == "state->ITEM_SUCCEEDED")
                fc = sum(it["boxes"][n]["contact"] for it in items if it["end_reason"] == "state->ITEM_FAILED")
                L.append(f"| {n} | {fnum(e.get('min_clearance'), 4)} m | {e.get('near')} | {e.get('danger')} | "
                         f"{e.get('contact')} | {fnum(e.get('speed_at_min'), 3)} m/s | {sc} | {fc} |")
            top = sorted(((it["boxes"][n]["contact"], it.get("item"), n, it["end_reason"],
                           it["boxes"][n]["min_clearance"])
                          for it in items for n in it["boxes"] if it["boxes"][n]["contact"]), reverse=True)[:8]
            if top:
                L += ["", "**接触最集中的作业项（前 8）**", "",
                      "| 作业项 | 方块 | contact | 最小间隙 | 结局 |", "|---|---|---|---|---|"]
                for c, i, n, er, mc in top:
                    L.append(f"| item{i} | {n} | {c} | {fnum(mc, 4)} | {er} |")

        # 逐轮
        if r["rounds"]:
            L += ["", "**逐轮明细**", "",
                  "| 轮 | 规则 | 耗时 | 结果 | 完成/尝试 | 失败物块 | 失败原因 | 连续问题 |",
                  "|---|---|---|---|---|---|---|---|"]
            for x in r["rounds"]:
                c = x.get("competition") or {}
                if x.get("driver_error"):
                    L.append(f"| {x.get('round')} | — | — | **栈故障** | — | — | "
                             f"`{str(x.get('driver_error'))[:90]}` | {x.get('consecutive_problems')} |")
                    continue
                bad = [i.get("detail") for i in (c.get("items") or []) if not i.get("success")]
                dur = f"{x['wall_execution_sec']:.0f}s" if x.get("wall_execution_sec") else "—"
                L.append(f"| {x.get('round')} | {x.get('rule_id')} | {dur} | "
                         f"{'✅通过' if x.get('round_passed') else '❌失败'} | "
                         f"{c.get('completed')}/{c.get('attempted')} | "
                         f"{', '.join(c.get('failed_cubes') or []) or '—'} | "
                         f"{(bad[0][:70] if bad else '—')} | {x.get('consecutive_problems')} |")
            # 失败原因汇总
            from collections import Counter
            cnt = Counter()
            for x in V:
                if x.get("round_passed"):
                    continue
                for i in ((x.get("competition") or {}).get("items") or []):
                    if not i.get("success"):
                        cnt[(i.get("detail") or "")[:80]] += 1
            if cnt:
                L += ["", "失败原因汇总：" + "；".join(f"×{n} {k}" for k, n in cnt.most_common())]

    (out / "实验全记录.md").write_text("\n".join(L) + "\n")
    print(f"已生成：{out/'实验全记录.md'}  {out/'runs_all.csv'}  {out/'rounds_all.csv'}  {out/'items_all.csv'}")
    print(f"记录 {len(recs)} 条")


if __name__ == "__main__":
    main()
