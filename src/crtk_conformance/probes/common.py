"""Helpers shared by the motion probes: enabling the arm, settling detection, step execution."""
from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np

from .. import geometry as G
from ..adapter import PlatformAdapter, Buffer, pose_msg_to_matrix


def ensure_enabled(a: PlatformAdapter, timeout_s: float = 3.0) -> dict:
    """Bring the arm to ENABLED/homed through the public operating-state interface, if it exists.
    Returns a dict describing what was found and done."""
    info = {"operating_state_present": a.has("operating_state"), "state_before": None, "state_after": None, "enable_latency_s": None}
    if not info["operating_state_present"]:
        return info
    st = a.operating_state(timeout=1.0)
    info["state_before"] = None if st is None else {"state": st.state, "is_homed": st.is_homed, "is_busy": st.is_busy}
    t0 = time.monotonic()
    a.state_command("enable")
    a.state_command("home")
    deadline = t0 + timeout_s
    while time.monotonic() < deadline:
        st = a.operating_state(timeout=0.3)
        if st is not None and st.state == "ENABLED" and st.is_homed:
            info["enable_latency_s"] = time.monotonic() - t0
            break
    st = a.operating_state(timeout=0.5)
    info["state_after"] = None if st is None else {"state": st.state, "is_homed": st.is_homed, "is_busy": st.is_busy}
    return info


def wait_settled(a: PlatformAdapter, buf: Buffer, timeout_s: float, still_tol: float = 1e-5, still_count: int = 3) -> Tuple[Optional[np.ndarray], float]:
    """Wait until measured_cp is stationary; return (pose, time_to_first_motion_or_nan)."""
    t0 = time.monotonic()
    ref = a.latest_pose(buf, max_age_s=1.0)
    first_motion = float("nan")
    last = ref
    still = 0
    deadline = t0 + timeout_s
    while time.monotonic() < deadline:
        msg = a.wait_for(buf, min(0.2, max(0.0, deadline - time.monotonic())))
        if msg is None:
            continue
        T = pose_msg_to_matrix(msg)
        if ref is not None and np.isnan(first_motion) and np.linalg.norm(T[:3, 3] - ref[:3, 3]) > still_tol:
            first_motion = time.monotonic() - t0
        if last is not None and np.linalg.norm(T[:3, 3] - last[:3, 3]) <= still_tol:
            still += 1
            if still >= still_count and (not np.isnan(first_motion) or time.monotonic() - t0 > 0.3):
                return T, first_motion
        else:
            still = 0
        last = T
    return last, first_motion


def step_and_measure(a: PlatformAdapter, buf: Buffer, delta: np.ndarray, settle_s: float = 1.0, still_tol: float = 1e-5) -> dict:
    """Command measured_cp + delta (in interface units, unqualified frame) and measure the response."""
    p0 = a.latest_pose(buf, max_age_s=1.0)
    if p0 is None:
        msg = a.wait_for(buf, 1.0)
        if msg is None:
            return {"ok": False, "reason": "no measured_cp"}
        p0 = pose_msg_to_matrix(msg)
    goal = p0.copy()
    goal[:3, 3] += delta
    t_cmd = time.monotonic()
    a.servo_cp(goal)
    p1, t_first = wait_settled(a, buf, settle_s, still_tol=still_tol)
    if p1 is None:
        return {"ok": False, "reason": "no response"}
    moved = p1[:3, 3] - p0[:3, 3]
    return {"ok": True, "p0": p0, "p1": p1, "delta_cmd": delta, "delta_meas": moved, "first_motion_s": t_first, "t_cmd": t_cmd}
