#!/usr/bin/env python3
"""
Question source: executes TMSCQtest binary to get random competition questions.
Also supports manual question input mode.
"""

import subprocess
import time
from datetime import datetime
from zoneinfo import ZoneInfo
import os
import stat
import hashlib
import re
import unicodedata
from pathlib import Path
from typing import Dict, List, Optional, Tuple


VARIABLE_PATTERN = r"[A-Za-z][A-Za-z0-9_]*"

# The competition generator is a Nuitka one-file bundle: its bootloader
# extracts into TMPDIR, and this build then prints nothing at all (exit 0, empty
# stdout *and* stderr) whenever TMPDIR contains non-ASCII characters. The
# workspace path is Chinese and the launcher deliberately points TMPDIR at a
# per-record directory inside it, so question generation failed silently as
# "Binary produced no output" (ISSUE-013, 2026-10-13 21:57 PARSE_FAILED).
# Measured 2026-10-14 with the same binary and tokens:
#   TMPDIR=/tmp/ascii_tmp               -> 427 B question, exit 0
#   TMPDIR=/tmp/ascii_tmp/中文子目录      -> 0 B, exit 0
#   TMPDIR=<Chinese Work path>/logs/tmp -> 0 B, exit 0
# It also raises UnicodeEncodeError from print() when it inherits an ASCII
# locale (a launcher started without LANG), so pin a UTF-8 stdio encoding too.
QUESTION_BINARY_TMPDIR = "/tmp/aic_question_binary"

# Question-source selector. The competition stack runs with --mode competition,
# which normally pins the official token below. AIC_QUESTION_MODE=random calls
# the generator with no arguments instead (development/random question set), so a
# long acceptance run can exercise the published generator's other question
# templates without changing --mode or any mapping/parsing semantics.
# Unset/`official` keeps the historical behaviour byte for byte; an unknown value
# is refused instead of silently falling back to the official token.
QUESTION_MODE_ENV = "AIC_QUESTION_MODE"
OFFICIAL_QUESTION_TOKEN = "11470507"
_RANDOM_QUESTION_MODES = {"random", "development", "dev"}
_OFFICIAL_QUESTION_MODES = {"", "official", "competition"}


def resolve_generator_args(competition_mode: bool, environ: Optional[Dict[str, str]] = None) -> List[str]:
    """Generator argv from AIC_QUESTION_MODE; default keeps the official token."""
    env = os.environ if environ is None else environ
    requested = str(env.get(QUESTION_MODE_ENV, "")).strip().lower()
    if requested in _RANDOM_QUESTION_MODES:
        return []
    if requested in _OFFICIAL_QUESTION_MODES:
        return [OFFICIAL_QUESTION_TOKEN] if competition_mode else []
    raise ValueError(
        f"Unsupported {QUESTION_MODE_ENV}={requested!r}; expected 'official' or 'random'"
    )


def binary_subprocess_env() -> Dict[str, str]:
    """Environment for the question generator: ASCII TMPDIR plus UTF-8 stdio."""
    env = os.environ.copy()
    if not env.get("TMPDIR", "").isascii():
        try:
            os.makedirs(QUESTION_BINARY_TMPDIR, exist_ok=True)
        except OSError:
            env["TMPDIR"] = "/tmp"
        else:
            env["TMPDIR"] = QUESTION_BINARY_TMPDIR
    env["PYTHONIOENCODING"] = "utf-8"
    env.setdefault("LANG", "C.UTF-8")
    return env


def extract_result_variables(text: str) -> List[str]:
    """Extract the requested result-variable order from generator evidence."""
    value = str(text or "").strip()
    if not value:
        return []

    explicit = re.search(
        r"(?:抓取顺序|结果顺序|result\s+variable\s+order|order)\s*[:：]?\s*"
        rf"(?P<variables>{VARIABLE_PATTERN}(?:\s*[,，、]\s*{VARIABLE_PATTERN})+)",
        value,
        flags=re.IGNORECASE,
    )
    if explicit:
        return re.findall(VARIABLE_PATTERN, explicit.group("variables"))

    requested = re.search(
        rf"计算\s*(?P<variables>{VARIABLE_PATTERN}"
        rf"(?:\s*[,，、]\s*{VARIABLE_PATTERN})+)\s*分别",
        value,
        flags=re.IGNORECASE,
    )
    if requested:
        return re.findall(VARIABLE_PATTERN, requested.group("variables"))
    short_request = re.search(
        rf"计算\s*(?P<variables>{VARIABLE_PATTERN}(?:\s*[,，、]\s*{VARIABLE_PATTERN})+)\s*[。?？]?(?:\s*[-]+)?\s*$",
        value,
        flags=re.IGNORECASE,
    )
    if short_request:
        return re.findall(VARIABLE_PATTERN, short_request.group("variables"))
    return []


def calculate_generated_evidence(question_text: str) -> Optional[dict]:
    """Parse every demand, including references, before doing integer arithmetic.

    Unknown clauses are never skipped. This supports the published generator's
    grammar, not arbitrary natural-language mathematics.
    """
    text = unicodedata.normalize("NFKC", str(question_text or ""))
    lines = []
    for line in text.splitlines():
        line = line.strip()
        if not line or re.fullmatch(r"-+", line) or re.fullmatch(r"使用种子:\s*\d+", line):
            continue
        lines.append(line)
    text = "".join(lines)
    text = re.sub(r"^题目\s*[:：]\s*", "", text)
    text = re.sub(r"\s+", "", text).replace("；", ";").replace("，", ",")
    text = text.rstrip("。;")
    header = re.match(r"每个(?P<container>[^;。]+?)有(?P<capacity>\d+)[^;。]*[;。]"
                      r"有(?P<first>[^、,;。]+)[、,](?P<second>[^;。]+?)两种"
                      r"(?P<category_container>[^;。]+)[;。]", text)
    if not header:
        return None
    units = {"衣柜": "件衣物", "宠物店": "只宠物", "果篮": "个水果", "书箱": "本书籍",
             "礼品盒": "个糖果", "文具盒": "个文具", "玩具箱": "个玩具",
             "食品袋": "个食品", "球框": "个运动器材"}
    noun = units.get(header.group("container"))
    if noun is None or not re.match(re.escape("每个" + header.group("container") + "有" + header.group("capacity") + noun) + r"[;。]", text):
        return None
    capacity = int(header.group("capacity"))
    container = header.group("container")
    categories = (header.group("first"), header.group("second"))
    if capacity <= 0 or categories[0] == categories[1] or header.group("category_container") != container:
        return None
    clauses = [part for part in re.split(r"[;。]", text[header.end():]) if part]
    if len(clauses) < 2 or not clauses[-1].startswith("设"):
        return None
    ending = re.fullmatch(r"设(.+),计算([A-Za-z][A-Za-z0-9_]*[、,][A-Za-z][A-Za-z0-9_]*)"
                          r"(?:分别为多少|分别是多少)?[?？]?", clauses[-1])
    if not ending:
        return None
    definitions = ending.group(1).split(",")
    requested = re.split(r"[、,]", ending.group(2))
    if len(definitions) != 2 or len(set(requested)) != 2:
        return None
    aliases = {container}
    aliases.update({"礼品盒": {"礼盒", "盒"}, "食品袋": {"袋"}, "书箱": {"箱"}}.get(container, set()))
    suffix = "(?:" + "|".join(re.escape(v) for v in sorted(aliases, key=len, reverse=True)) + ")"
    variable_by_category = {}
    for category, definition in zip(categories, definitions):
        match = re.fullmatch(re.escape(category) + suffix + r"(?:数量|数)为(" + VARIABLE_PATTERN + ")", definition)
        if not match:
            return None
        variable_by_category[category] = match.group(1)
    if set(requested) != set(variable_by_category.values()):
        return None

    people = {}
    totals = dict.fromkeys(categories, 0)
    item_pattern = re.compile(r"(\d+)(?:件|条|只|个|份|箱|包|本|盒)?(" +
                              "|".join(re.escape(v) for v in sorted(categories, key=len, reverse=True)) + ")")
    for clause in clauses[:-1]:
        demand = dict.fromkeys(categories, 0)
        reference = re.fullmatch(r"(.+?)(?:说|也)?和(.+?)(?:需求)?一样(.*)", clause)
        if reference:
            subject, previous, tail = reference.groups()
            if previous not in people:
                return None
            demand = dict(people[previous])
            if tail:
                swap = re.fullmatch(r"但是换成(.+)", tail)
                less = re.fullmatch(r",但少(?:一|1)(?:件|条|只|个|份|本|盒)(.+)", tail)
                if swap and swap.group(1) in categories:
                    nonzero = [v for v in categories if demand[v]]
                    if len(nonzero) != 1:
                        return None
                    value = demand[nonzero[0]]
                    demand = dict.fromkeys(categories, 0)
                    demand[swap.group(1)] = value
                elif less and less.group(1) in categories:
                    demand[less.group(1)] -= 1
                    if demand[less.group(1)] < 0:
                        return None
                else:
                    return None
            subjects = [subject]
        else:
            action = re.fullmatch(r"(.+?)(都)?(?:想要|想玩|需要|领养|领取|购买|获得|借阅|要)(.+)", clause)
            if not action:
                return None
            subject_text, all_people, items = action.groups()
            subjects = re.split(r"[、,]", subject_text)
            if any(v.endswith(("不", "没", "未", "无需")) for v in subjects):
                return None
            if len(subjects) > 1 and not all_people:
                return None
            doubled = False
            if ",他是双胞胎所以需要双倍" in items:
                items, descriptor = items.split(",他是双胞胎所以需要双倍", 1)
                if descriptor not in (categories[0], "水果", "的书"):
                    return None
                doubled = True
            for item in items.split(","):
                match = item_pattern.fullmatch(item)
                if not match:
                    return None
                demand[match.group(2)] += int(match.group(1))
            if doubled:
                if demand[categories[1]] or not demand[categories[0]]:
                    return None
                demand[categories[0]] *= 2
        if not subjects or any(not re.fullmatch(r"小[\u4e00-\u9fff]{1,2}", v) or v in people for v in subjects):
            return None
        for subject in subjects:
            people[subject] = dict(demand)
            for category in categories:
                totals[category] += demand[category]
    if any(v <= 0 for v in totals.values()):
        return None
    calculation = {variable_by_category[c]: (totals[c] + capacity - 1) // capacity for c in categories}
    return {"calculation": calculation, "capacity": capacity, "totals": totals,
            "variable_by_category": variable_by_category,
            "formula": "count = (total + capacity - 1) // capacity"}


def calculate_generated_answer(question_text: str) -> Optional[Dict[str, int]]:
    evidence = calculate_generated_evidence(question_text)
    return evidence["calculation"] if evidence is not None else None


class QuestionSource:
    """Handles question generation from TMSCQtest binary or manual input."""

    DEFAULT_BINARY_PATHS = [
        str(Path(__file__).resolve().parents[5] / "TMSCQtest_x86_x64.bin"),
        "/mnt/hgfs/vmshared/question_generator/TMSCQtest_x86_x64.bin",
        "/home/ros/Desktop/TMSCQtest_x86_x64.bin",
        "/home/ros/dev_ws/TMSCQtest_x86_x64.bin",
    ]

    def __init__(self, binary_path: Optional[str] = None, *, competition_mode: bool = False):
        self.competition_mode = bool(competition_mode)
        self.last_run_evidence = None
        self.binary_path = None
        self.binary_info = {}

        binary_path = binary_path or os.environ.get("AIC_QUESTION_BINARY_PATH")
        if binary_path:
            self._setup_binary(binary_path)
        else:
            for path in self.DEFAULT_BINARY_PATHS:
                if os.path.exists(path):
                    self._setup_binary(path)
                    break

    def _setup_binary(self, path: str):
        """Verify and prepare the binary."""
        p = Path(path)
        if not p.exists():
            raise FileNotFoundError(f"Binary not found: {path}")

        # Make executable if needed
        st = p.stat()
        if not st.st_mode & stat.S_IEXEC:
            os.chmod(path, st.st_mode | stat.S_IEXEC)

        # Record info
        sha = hashlib.sha256(p.read_bytes()).hexdigest()
        requested_mode = str(os.environ.get(QUESTION_MODE_ENV, "")).strip().lower()
        self.binary_info = {
            "path": str(p.absolute()),
            "size": p.stat().st_size,
            "sha256": sha,
            "mode": "competition" if self.competition_mode else "development",
            "question_mode": "random" if requested_mode in _RANDOM_QUESTION_MODES else "official",
            "args": resolve_generator_args(self.competition_mode),
        }
        self.binary_path = str(p.absolute())

    @property
    def is_available(self) -> bool:
        return self.binary_path is not None and os.path.exists(self.binary_path)

    def run(self, timeout: float = 30.0) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        """
        Execute the TMSCQtest binary and capture output.

        Returns:
            (stdout, stderr, error_message)
            stdout: Raw question output on success
            stderr: Stderr output if any
            error_message: None on success, error string on failure
        """
        if not self.is_available:
            return None, None, "Binary not found or not executable"

        try:
            started = datetime.now(ZoneInfo("Asia/Shanghai")).isoformat()
            timer = time.monotonic()
            command = [self.binary_path, *self.binary_info["args"]]
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                cwd=os.path.dirname(self.binary_path),
                env=binary_subprocess_env(),
            )

            self.last_run_evidence = {
                **self.binary_info, "command": command, "started_at": started,
                "finished_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
                "elapsed_sec": time.monotonic() - timer, "returncode": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr,
            }

            if result.returncode != 0:
                return (
                    result.stdout if result.stdout else None,
                    result.stderr if result.stderr else None,
                    f"Binary exited with code {result.returncode}",
                )

            stdout = result.stdout.strip() if result.stdout else None
            stderr = result.stderr.strip() if result.stderr else None

            if not stdout:
                return None, stderr, (
                    "Binary produced no output"
                    " (check the generator TMPDIR is ASCII and its stdio encoding is UTF-8)"
                )

            return stdout, stderr, None

        except subprocess.TimeoutExpired:
            return None, None, f"Binary timed out after {timeout}s"
        except Exception as e:
            return None, None, f"Binary execution error: {str(e)}"

    def run_and_parse(self, timeout: float = 30.0) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        """
        Run the binary and attempt to parse the output.
        Returns (question_text, grab_order, error_message).
        """
        stdout, stderr, error = self.run(timeout)
        if error:
            return None, None, error

        # Try to extract question and order from the binary output
        question_text = stdout
        grab_order = None

        # Prefer an explicit order line, otherwise derive the requested result order
        # from the final "计算x、y分别为多少" phrase.
        for line in stdout.split("\n"):
            line_stripped = line.strip()
            if "顺序" in line_stripped or "order" in line_stripped.lower():
                grab_order = line_stripped
                break

        if grab_order is None:
            variables = extract_result_variables(stdout)
            if variables:
                grab_order = "result variable order: " + ",".join(variables)

        return question_text, grab_order, None


def main():
    """Test the question source."""
    src = QuestionSource()
    print(f"Binary available: {src.is_available}")
    if src.is_available:
        print(f"Binary info: {src.binary_info}")
        question, order, error = src.run_and_parse()
        if error:
            print(f"Error: {error}")
        else:
            print(f"Question: {question}")
            print(f"Order: {order}")


if __name__ == "__main__":
    main()

