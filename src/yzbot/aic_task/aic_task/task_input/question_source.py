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
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


VARIABLE_PATTERN = r"[A-Za-z][A-Za-z0-9_]*"

# The competition generator is a PyInstaller one-file bundle: its bootloader
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
    return []


def calculate_generated_answer(question_text: str) -> Optional[Dict[str, int]]:
    """Independently solve the two-category template emitted by TMSCQtest."""
    text = str(question_text or "").strip()
    if not text:
        return None

    capacity_match = re.search(r"每个[^;；\n]*?有\s*(\d+)", text)
    if not capacity_match:
        return None
    capacity = int(capacity_match.group(1))
    if capacity <= 0:
        return None

    clauses = [part.strip() for part in re.split(r"[;；。]", text) if part.strip()]
    category_index = None
    categories = None
    for index, clause in enumerate(clauses):
        match = re.match(r"有\s*(.+?)\s*[、,，]\s*(.+?)\s*两种", clause)
        if match:
            category_index = index
            categories = (match.group(1).strip(), match.group(2).strip())
            break
    if categories is None:
        return None

    definition_index = next(
        (index for index, clause in enumerate(clauses) if clause.startswith("设")),
        None,
    )
    if definition_index is None or definition_index <= category_index:
        return None

    variable_by_category = {}
    definition = clauses[definition_index]
    for category in categories:
        match = re.search(
            re.escape(category)
            + rf"[^,，;；。]*?数量为\s*({VARIABLE_PATTERN})",
            definition,
            flags=re.IGNORECASE,
        )
        if not match:
            return None
        variable_by_category[category] = match.group(1)

    totals = {category: 0 for category in categories}
    action_pattern = re.compile(r"^(?P<subjects>.+?)(?P<all>都)?(?:需要|领养|领取|购买|获得|要)(?P<items>.+)$")
    for clause in clauses[category_index + 1 : definition_index]:
        action = action_pattern.match(clause)
        if not action:
            return None
        multiplier = 1
        if action.group("all"):
            subjects = [
                item for item in re.split(r"[、,，]", action.group("subjects")) if item
            ]
            multiplier = len(subjects)
        items = action.group("items")
        matched_any = False
        for category in categories:
            matches = re.findall(
                rf"(\d+)\s*(?:件|条|只|个|份|箱|包)?\s*{re.escape(category)}",
                items,
            )
            if matches:
                matched_any = True
                totals[category] += multiplier * sum(int(value) for value in matches)
        if not matched_any:
            return None

    if any(total <= 0 for total in totals.values()):
        return None
    return {
        variable_by_category[category]: math.ceil(totals[category] / capacity)
        for category in categories
    }


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
        self.binary_info = {
            "path": str(p.absolute()),
            "size": p.stat().st_size,
            "sha256": sha,
            "mode": "competition" if self.competition_mode else "development",
            "args": ["11470507"] if self.competition_mode else [],
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

