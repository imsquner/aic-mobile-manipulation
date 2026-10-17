#!/usr/bin/env python3
"""Plan O: 一条命令判定 P0 是否奏效（对比改前基线）。

用法：
  python3 scratch/plan_o_check_p0.py [记录目录]
  # 不传参数则自动用 scratch/plan_d_record.txt 指向的当前记录

输出：
  - after60（当前记录）run 目录里 nav2.log 的 `extrapolation into the future` 次数（改前 = 20）
  - 该记录探针日志最后一行（最小间隙 / contact / 监视器触发）
  - rounds.jsonl 的行数（已跑轮数）
"""
from pathlib import Path
import re
import sys

W = Path("/home/ros/桌面/AIC_Competition_Work")
BASELINE_EXTRAPOLATION = 20          # 改前 20261017_001904_..._avoidance_before 实测
BASELINE_CONTACT = {"moving_box_1": 4326, "moving_box_2": 66}


def newest_run(record):
    runs = sorted([p for p in (record / "logs").glob("run_*") if p.is_dir()],
                  key=lambda p: p.stat().st_mtime)
    return runs[-1] if runs else None


def find_probe_log(record):
    """优先 avoidance_after*.log，其次 avoidance_before*.log，取最新修改的那个。"""
    for pattern in ("avoidance_after*.log", "avoidance_before*.log"):
        matches = list((record / "logs").glob(pattern))
        if matches:
            return max(matches, key=lambda p: p.stat().st_mtime)
    return None


def main():
    record = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        (W / "scratch/plan_d_record.txt").read_text().strip())
    print(f"记录: {record}")

    run = newest_run(record)
    if run is None:
        print("（还没有 run_* 目录：栈可能还没起来）")
    else:
        nav2 = run / "nav2.log"
        if not nav2.exists():
            print(f"{run.name}/nav2.log 不存在")
        else:
            text = nav2.read_text(errors="replace")
            hits = len(re.findall(r"extrapolation into the future", text))
            # 按时间归一：改前基线 20 次 / 约 34.7 分钟 ≈ 0.58 次/分钟。
            # 60 轮约 3.3 小时，直接比总次数会误判，所以用"预期次数"做参照。
            elapsed_min, expected = None, None
            probe_file = find_probe_log(record)
            if probe_file is not None:
                last = [l for l in probe_file.read_text(errors="replace").splitlines()
                        if "moving_box_1:" in l]
                if last:
                    found = re.search(r"\[\s*([\d.]+)s\]", last[-1])
                    if found:
                        elapsed_min = float(found.group(1)) / 60.0
                        expected = (20.0 / 34.7) * elapsed_min
            if hits <= 3:
                verdict = "P0 生效 ✓"
            elif expected is not None and hits <= max(3.0, 0.2 * expected):
                verdict = "P0 生效（按时间归一：≤改前速率的 20%）✓"
            elif hits < BASELINE_EXTRAPOLATION:
                verdict = "部分生效 ⚠️"
            else:
                verdict = "未见改善 ✗"
            window = (f"；本次已跑 {elapsed_min:.1f} 分钟，按改前速率预期约 {expected:.0f} 次"
                      if elapsed_min else "")
            print(f"nav2.log: extrapolation into the future = {hits}"
                  f"（改前 20 次 / 34.7 分钟{window}） ⇒ {verdict}")

    probe = find_probe_log(record)
    if probe is None:
        print("（还没找到探针日志）")
    else:
        lines = probe.read_text(errors="replace").splitlines()
        overall = [l for l in lines if "moving_box_1:" in l]
        print(f"探针 {probe.name}: {len(lines)} 行")
        if overall:
            print("  最新: " + overall[-1].split("]: ", 1)[-1])
            for box, before in BASELINE_CONTACT.items():
                match = re.search(rf"{box}: min=(-?[\d.]+) .*?contact=(\d+)", overall[-1])
                if match:
                    print(f"  {box}: min={match.group(1)} contact={match.group(2)}"
                          f"（改前 contact={before}）")

    ledger = record / "results" / "rounds.jsonl"
    if ledger.exists():
        rows = [l for l in ledger.read_text(errors="replace").splitlines() if l.strip()]
        print(f"rounds.jsonl: {len(rows)} 轮")
    else:
        print("（还没有 rounds.jsonl）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
