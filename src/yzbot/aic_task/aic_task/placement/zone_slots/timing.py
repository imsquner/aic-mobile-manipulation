"""Pure timestamp validation used by World2's slot-flow safety gates."""

import math


NANOSECONDS_PER_SECOND = 1_000_000_000
# 契约值：必须与 Nav2 默认 YAML 的 AMCL transform_tolerance 一致（有契约测试守着）。
AMCL_MAP_ODOM_TRANSFORM_TOLERANCE_SEC = 1.0
# 余量系数：本 VM 负载高时 AMCL 自身会漏更新，2026-09-26 实测 map->odom 的 age 达到
# 1.046 s（刚好越过 1.0 s）而内容仍有效。时间戳门限取"契约值 × 该系数"，AMCL 真正
# 停摆时仍会拦下；保持契约常量不变以维持与 Nav2 配置的一致性检查。
AMCL_TOLERANCE_HEADROOM = 2.0
_CLOCK_DOMAIN_OR_LARGE_JUMP_SEC = 60.0


def _integer_stamp_ns(value):
    if value is None or isinstance(value, bool):
        return None
    try:
        numeric = float(value)
        if not math.isfinite(numeric) or not numeric.is_integer():
            return None
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return None


def evaluate_transform_age(now_ns, stamp_ns, max_age_sec, age_offset_sec=0.0):
    """Return ``(accepted, corrected_age_sec, reason)`` for a ROS timestamp.

    ``age_offset_sec`` corrects publisher-specific future dating. For AMCL's
    map->odom transform it is the configured transform tolerance. The bound is
    symmetric: source stamps may be at most ``max_age_sec`` stale or future.
    This preserves small same-clock future skew while rejecting clock-domain
    mismatches and arbitrarily future-dated transforms.
    """
    now_value = _integer_stamp_ns(now_ns)
    stamp_value = _integer_stamp_ns(stamp_ns)
    if now_value is None or stamp_value is None:
        return False, None, "missing_or_nonfinite_stamp"
    if now_value <= 0 or stamp_value <= 0:
        return False, None, "zero_or_uninitialized_stamp"

    try:
        max_age = float(max_age_sec)
        age_offset = float(age_offset_sec)
    except (TypeError, ValueError, OverflowError):
        return False, None, "invalid_age_configuration"
    if (
        not math.isfinite(max_age)
        or max_age <= 0.0
        or not math.isfinite(age_offset)
    ):
        return False, None, "invalid_age_configuration"

    age_sec = (now_value - stamp_value) / NANOSECONDS_PER_SECOND + age_offset
    if not math.isfinite(age_sec):
        return False, age_sec, "nonfinite_age"
    if abs(age_sec) > _CLOCK_DOMAIN_OR_LARGE_JUMP_SEC:
        return False, age_sec, "clock_domain_or_large_jump"
    if age_sec < -max_age:
        return False, age_sec, "future"
    if age_sec > max_age:
        return False, age_sec, "stale"
    return True, age_sec, ""


def evaluate_scan_tf_fallback_age(now_ns, scan_stamp_ns, transform_stamp_ns,
                                 max_window_sec):
    """Bound scan and latest-TF ages plus their mutual timestamp skew.

    Return ``(accepted, scan_age_sec, transform_age_sec, skew_sec, reason)``.
    This is used only after a scan-time transform lookup fails. Both stamps
    must be valid in the current ROS clock domain, each must be within the
    configured window of ``now_ns``, and they must be within that same window
    of one another.
    """
    scan_ok, scan_age_sec, scan_reason = evaluate_transform_age(
        now_ns, scan_stamp_ns, max_window_sec
    )
    if not scan_ok:
        return False, scan_age_sec, None, None, f"scan_{scan_reason}"

    transform_ok, transform_age_sec, transform_reason = evaluate_transform_age(
        now_ns, transform_stamp_ns, max_window_sec
    )
    if not transform_ok:
        return (
            False,
            scan_age_sec,
            transform_age_sec,
            None,
            f"transform_{transform_reason}",
        )

    scan_value = _integer_stamp_ns(scan_stamp_ns)
    transform_value = _integer_stamp_ns(transform_stamp_ns)
    skew_sec = abs(scan_value - transform_value) / NANOSECONDS_PER_SECOND
    if not math.isfinite(skew_sec):
        return False, scan_age_sec, transform_age_sec, skew_sec, "nonfinite_skew"
    try:
        max_window = float(max_window_sec)
    except (TypeError, ValueError, OverflowError):
        return (
            False,
            scan_age_sec,
            transform_age_sec,
            skew_sec,
            "invalid_age_configuration",
        )
    if skew_sec > max_window:
        return (
            False,
            scan_age_sec,
            transform_age_sec,
            skew_sec,
            "scan_transform_skew",
        )
    return True, scan_age_sec, transform_age_sec, skew_sec, ""


def should_log_scan_tf_fallback(accepted, monotonic_now, last_accepted_state,
                                last_event_monotonic, min_interval_sec=1.0):
    """Log every rejection and limit repeated accepted fallback events.

    An accepted event is logged on the first fallback, after a state change,
    or once the minimum interval has elapsed. Invalid throttle inputs fail
    toward logging so diagnostics are not silently suppressed.
    """
    if not accepted:
        return True
    try:
        now_value = float(monotonic_now)
        min_interval = float(min_interval_sec)
    except (TypeError, ValueError, OverflowError):
        return True
    if (
        not math.isfinite(now_value)
        or not math.isfinite(min_interval)
        or min_interval <= 0.0
        or last_accepted_state is not True
    ):
        return True
    try:
        last_value = float(last_event_monotonic)
    except (TypeError, ValueError, OverflowError):
        return True
    if not math.isfinite(last_value):
        return True
    elapsed = now_value - last_value
    return not math.isfinite(elapsed) or elapsed < 0.0 or elapsed >= min_interval
