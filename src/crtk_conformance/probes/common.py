"""Helpers shared by the motion probes: enabling the arm, settling detection, step execution."""
from __future__ import annotations

import time
from typing import Optional, Tuple

import numpy as np

from .. import geometry as G
from ..adapter import PlatformAdapter, Buffer, pose_msg_to_matrix


ENABLE_TIMEOUT_S = 3.0  # implementation constant (0.1.6: settable with --enable-timeout-s; a homing sequence may take longer)


def _state_latest(a: PlatformAdapter, wait_s: float = 0.5):
    """(receipt time, message) of the most recent operating_state message, however old (an implementation may
    publish it on change only); waits up to wait_s for a first one.  (None, None) if there is none."""
    buf = a.subscribe("operating_state")
    d = buf.latest()
    if d is not None:
        return d
    m = a.wait_for(buf, wait_s)
    d = buf.latest()
    return d if d is not None else (None, m)


def ensure_enabled(a: PlatformAdapter, timeout_s: float = None) -> dict:
    """Bring the arm to ENABLED/homed through the public operating-state interface, if it exists.
    Returns a dict describing what was found and done.

    0.1.6 (RC9): one state command at a time.  0.1.5 published `enable` and `home` back to back; the released dVRK
    console (cisst-ros 4.0.0 ROS 1 bridge, every write-command subscriber has queue size 1, mtsROSBridge.h l.295-296)
    keeps only the latest state command, so `enable` was superseded by `home`, which is not accepted while DISABLED:
    an arm disabled by the state-machine sub-probe was never re-enabled (RC9 harness diagnostic, before any probe run
    on that target).  Now `enable` is sent only when the state is not ENABLED and the arm is awaited in ENABLED before
    `home` is sent; `home` is then awaited until is_homed and not is_busy (homing moves the arm; is_homed is reported
    before the homing motion ends).  After an `enable`, `home` is sent even when is_homed is already true: the dVRK
    console reports the flag across a disable/enable cycle while its own state machine requires homing again.  An arm
    already ENABLED, homed and not busy receives no command at all (0.1.5 sent both on every call, also before every
    gap trial)."""
    if timeout_s is None:
        timeout_s = ENABLE_TIMEOUT_S
    info = {"operating_state_present": a.has("operating_state"), "state_before": None, "state_after": None, "enable_latency_s": None,
            "commands_sent": []}
    if not info["operating_state_present"]:
        return info
    st = a.operating_state(timeout=1.0)
    info["state_before"] = None if st is None else {"state": st.state, "is_homed": st.is_homed, "is_busy": st.is_busy}
    t0 = time.monotonic()
    deadline = t0 + timeout_s

    def await_state(pred, t_sent):
        # a state message received after the command, or 0.3 s without one (a no-op command may publish nothing)
        while True:
            t_rx, s = _state_latest(a, 0.1)
            fresh = t_rx is not None and (t_rx >= t_sent or time.monotonic() - t_sent > 0.3)
            if s is not None and fresh and pred(s):
                return s
            if time.monotonic() >= deadline:
                return s
            time.sleep(0.02)

    if st is None or st.state != "ENABLED":
        t_sent = time.monotonic()
        a.state_command("enable")
        info["commands_sent"].append("enable")
        st = await_state(lambda s: s.state == "ENABLED", t_sent)
    if st is not None and st.state == "ENABLED" and (info["commands_sent"] or not st.is_homed):
        t_sent = time.monotonic()
        a.state_command("home")
        info["commands_sent"].append("home")
        st = await_state(lambda s: s.state == "ENABLED" and s.is_homed and not s.is_busy, t_sent)
    elif st is not None and st.state == "ENABLED" and st.is_busy:
        st = await_state(lambda s: s.state == "ENABLED" and s.is_homed and not s.is_busy, -1.0)
    if st is not None and st.state == "ENABLED" and st.is_homed and not st.is_busy:
        info["enable_latency_s"] = time.monotonic() - t0
    st = _state_latest(a, 0.5)[1]
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


def stream_goal(a: PlatformAdapter, goal, until: float, hz: float) -> None:
    """Publish `goal` on servo_cp at `hz` until the monotonic time `until` (servo commands are streamed; a single
    servo_cp followed by silence is not how a client uses them and would trigger any silence-triggered stop policy)."""
    if goal is None or hz <= 0:
        time.sleep(max(0.0, until - time.monotonic()))
        return
    period = 1.0 / hz
    while True:
        a.servo_cp(goal)
        nxt = time.monotonic() + period
        if nxt >= until:
            time.sleep(max(0.0, until - time.monotonic()))
            return
        time.sleep(period)


def step_and_measure(a: PlatformAdapter, buf: Buffer, delta: np.ndarray, settle_s: float = 1.0, still_tol: float = 1e-5, window_s: float = 0.3,
                     stream_hz: float = 0.0, hold=None) -> dict:
    """Command measured_cp + delta (in interface units, unqualified frame) and measure the response.

    0.1.1: noise-robust form.  The start pose p0 is the mean of the feedback samples in a window_s window before
    the command; after the command the probe waits settle_s and takes p1 as the mean of the samples in the last
    window_s.  The first-motion time is the first sample farther than still_tol from p0 (the caller passes a
    noise-scaled still_tol).  A trial with no sample beyond still_tol is a 'no response' (first_motion_s = nan).
    With stream_hz > 0 the goal is streamed at that rate from the command until the end of the measurement window,
    and `hold` (a pose) is streamed during the pre-command window, as a servo client would: a single command
    followed by silence lets a silence-triggered stop policy fire inside the settle window (found in the v0.1.1 mock
    campaign on the AMBF-watchdog emulation, where the release drift was measured as a unit scale of 1.9).
    """
    t0 = time.monotonic()
    stream_goal(a, hold, t0 + window_s, stream_hz)
    pre = buf.since(t0)
    if not pre:
        msg = a.wait_for(buf, 1.0)
        if msg is None:
            return {"ok": False, "reason": "no measured_cp"}
        pre = [(time.monotonic(), msg)]
    P0 = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in pre])
    p0 = pose_msg_to_matrix(pre[-1][1])
    p0[:3, 3] = P0.mean(axis=0)
    goal = p0.copy()
    goal[:3, 3] += delta
    t_cmd = time.monotonic()
    a.servo_cp(goal)
    next_pub = t_cmd + (1.0 / stream_hz if stream_hz > 0 else float("inf"))
    deadline = t_cmd + settle_s + window_s
    first_motion = float("nan")
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_pub:
            a.servo_cp(goal)
            next_pub = now + 1.0 / stream_hz
        msg = a.wait_for(buf, min(0.05, max(0.0, min(deadline, next_pub) - time.monotonic())))
        if msg is None:
            continue
        if np.isnan(first_motion) and np.linalg.norm(pose_msg_to_matrix(msg)[:3, 3] - p0[:3, 3]) > still_tol:
            first_motion = time.monotonic() - t_cmd
    post = buf.since(deadline - window_s)
    if not post:
        return {"ok": False, "reason": "no response"}
    P1 = np.array([pose_msg_to_matrix(m)[:3, 3] for _, m in post])
    p1 = pose_msg_to_matrix(post[-1][1])
    p1[:3, 3] = P1.mean(axis=0)
    moved = p1[:3, 3] - p0[:3, 3]
    return {"ok": True, "p0": p0, "p1": p1, "delta_cmd": delta, "delta_meas": moved, "first_motion_s": first_motion, "t_cmd": t_cmd,
            "n_pre": int(len(pre)), "n_post": int(len(post)), "post_std_m": float(P1.std(axis=0).max())}
