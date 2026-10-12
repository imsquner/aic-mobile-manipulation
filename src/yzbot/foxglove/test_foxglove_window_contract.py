#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Foxglove 窗口选择的跨语言契约测试。

为什么需要它：窗口集合有两个实现 —— shell 的
`foxglove/foxglove_window_selection.sh`（决定真的起哪些 HTTP 服务与浏览器窗口）
和 Python 的 `start_competition.py::foxglove_selected_windows`（决定就绪门校验哪些
端口）。两边一旦不一致，就会出现「服务起来了但启动器报
`Offline Foxglove did not listen on HTTP 8088/8089/8090`」这类假失败。

2026-09-23 实测事故：把默认窗口收窄到 dashboard 后，Python 侧仍写死校验
8088/8089/8090，导致完整比赛态启动直接 FATAL。本测试守住这个契约。

用法：python3 foxglove/test_foxglove_window_contract.py
"""
import importlib.util
import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
YZBOT = HERE.parent
SHELL_HELPER = HERE / "foxglove_window_selection.sh"
LAUNCHER = YZBOT / "start_competition.py"

# Windows 上 `bash` 常解析到 System32\bash.exe（WSL）：它把工作目录映射成 /mnt/d/...
# 且不继承 env= 传入的变量，会让本测试假失败。因此显式优先 Git Bash。
BASH_CANDIDATES = (
    os.environ.get("BASH_EXE", ""),
    r"C:\Program Files\Git\bin\bash.exe",
    r"C:\Program Files (x86)\Git\bin\bash.exe",
    "/usr/bin/bash",
    "/bin/bash",
)

CASES = [
    "",                    # 未设置 -> dashboard
    " ",
    "dashboard",
    "map",
    "detection",
    "map,detection",
    "map, detection",
    "map ,detection",
    "all",
    "ALL",
    "map,map,dashboard",   # 去重
    "dashboard,map,detection",
]

failures = []
checks = 0


def find_bash():
    for candidate in BASH_CANDIDATES:
        if candidate and Path(candidate).exists():
            return candidate
    found = shutil.which("bash")
    if found and "system32" not in found.lower():
        return found
    return found


BASH = find_bash()


def run_shell(script, extra_exports=""):
    """在 helper 所在目录用相对路径跑 bash。

    取值通过**脚本内联赋值**传入，不用 env=：Git Bash 与 WSL 对内联赋值行为一致，
    而 env= 在 WSL 下会丢。
    也不要给 bash 传绝对路径：Windows 的 `D:\\桌面\\...` 在 bash 里 source 不了。
    """
    full = ""
    if extra_exports:
        full += extra_exports + "\n"
    full += "source ./foxglove_window_selection.sh\n" + script
    return subprocess.run(
        [BASH, "-c", full],
        cwd=str(HERE),
        capture_output=True, text=True,
        encoding="utf-8", errors="replace",
    )


def shell_normalize(value):
    result = run_shell(
        'foxglove_normalize_windows "$FGW_SELECT"',
        "FGW_SELECT=%s" % shlex.quote(value),
    )
    if result.returncode != 0:
        raise RuntimeError(
            "shell 归一化失败 value=%r stderr=%s" % (value, result.stderr.strip())
        )
    return result.stdout.strip()


def load_launcher_module():
    spec = importlib.util.spec_from_file_location("aic_start_competition", LAUNCHER)
    module = importlib.util.module_from_spec(spec)
    sys.modules["aic_start_competition"] = module
    spec.loader.exec_module(module)
    return module


def main():
    global checks
    for path in (SHELL_HELPER, LAUNCHER):
        if not path.exists():
            print("FAIL: 找不到 %s" % path)
            return 1
    print("bash = %s" % BASH)

    module = load_launcher_module()

    # 1) 归一化结果必须一致
    for case in CASES:
        checks += 1
        py = " ".join(module.foxglove_selected_windows(case))
        sh = shell_normalize(case)
        if py != sh:
            failures.append(
                "归一化不一致 FOXGLOVE_WINDOWS=%r: python=%r shell=%r"
                % (case, py, sh)
            )

    # 2) 端口映射必须一致
    for window, expected in module.FOXGLOVE_WINDOW_DEFAULTS.items():
        checks += 1
        got = run_shell(
            'foxglove_window_port "$FGW_WINDOW"',
            "FGW_WINDOW=%s" % shlex.quote(window),
        ).stdout.strip()
        if got != str(expected):
            failures.append(
                "端口不一致 %s: python=%s shell=%s" % (window, expected, got)
            )

    # 3) 非法值两边都必须拒绝
    checks += 1
    try:
        module.foxglove_selected_windows("bogus")
        failures.append("python 未拒绝非法窗口名 bogus")
    except ValueError:
        pass
    checks += 1
    if run_shell("foxglove_validate_windows bogus").returncode == 0:
        failures.append("shell 未拒绝非法窗口名 bogus")

    # 4) 回归守卫：启动器不得再写死三个端口
    launcher_src = LAUNCHER.read_text(encoding="utf-8")
    checks += 1
    if "for port in (8088, 8089, 8090):" in launcher_src:
        failures.append("start_competition.py 仍在就绪门里写死 8088/8089/8090")
    checks += 1
    if "foxglove_window_ports(" not in launcher_src:
        failures.append("start_competition.py 没有使用 foxglove_window_ports()")

    # 5) 默认必须是 dashboard-only（性能基线）
    checks += 1
    os.environ.pop("FOXGLOVE_WINDOWS", None)
    if module.foxglove_selected_windows() != ("dashboard",):
        failures.append("默认窗口集不是 dashboard-only")
    checks += 1
    if module.foxglove_window_ports() != [8090]:
        failures.append("默认端口集不是 [8090]")

    print("契约检查 %d 项" % checks)
    if failures:
        for line in failures:
            print("FAIL: %s" % line)
        print("失败 %d 项" % len(failures))
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
