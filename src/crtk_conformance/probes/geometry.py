"""GeometryAnchorProbe -- dimensional binding class, instrument-geometry unit anchor (0.1.6).

Question answered: how many metres is one interface length unit, using a known physical dimension of the instrument
rather than an out-of-band SI pose?

Procedure (public topics only: measured_js, servo_jp, measured_cp): from a reference joint configuration (declared, or
the current one), each trial steps the wrist-pitch joint by +delta_q with servo_jp (all other joints held), settles,
returns, then does the same with the wrist-yaw joint.  The full measured_cp pose before and after each step gives the
step's screw axis (dimensional.screw_axis).  The common-normal distance between the two axes, in interface units, is
the pitch-to-yaw length of the instrument model; lambda_hat = L_phys / d_int.  The decision is eq. (6) with the
interval widened by the declared relative uncertainty of L_phys (dimensional.geometry_anchor_decision).  Goals are
streamed at the client rate, as a servo client would.

What it assumes (recorded in the report): the published pose is rigidly attached to the distal link; the implementation's
wrist geometry equals the instrument's within u_rel; the joint steps are executed as commanded (gated: each step's
rotation angle must equal delta_q, and each step must be a pure rotation).  What it does not need: which point on the
distal link is published, or the unit of the prismatic insertion joint.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np

from sensor_msgs.msg import JointState

from .. import geometry as G
from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..dimensional import geometry_anchor_decision, screw_axis
from ..expectations import Expectations
from ..thresholds import Tolerance
from .base import Outcome, ProbeResult
from .common import ensure_enabled


def _joint_index(names: List[str], spec: str) -> Optional[int]:
    if spec is None:
        return None
    s = str(spec).strip()
    if s.lstrip("-").isdigit():
        i = int(s)
        return i if -len(names) <= i < len(names) else None
    hits = [i for i, n in enumerate(names) if s == n]
    if not hits:
        hits = [i for i, n in enumerate(names) if s in n]
    return hits[0] if len(hits) == 1 else None


class GeometryAnchorProbe:
    name = "GeometryAnchorProbe"

    def __init__(self, adapter: PlatformAdapter, tol: Tolerance, trials: int = 5, settle_s: float = 1.0, window_s: float = 0.3,
                 expectations: Optional[Expectations] = None):
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.settle = settle_s
        self.window = window_s
        self.exp = expectations or Expectations()

    # ------------------------------------------------------------------ helpers
    def _stream_jp(self, names, q, duration_s: float):
        period = 1.0 / max(1.0, self.tol.client_rate_hz)
        t_end = time.monotonic() + duration_s
        while True:
            self.a.servo_jp(names, q)
            if time.monotonic() + period >= t_end:
                time.sleep(max(0.0, t_end - time.monotonic()))
                return
            time.sleep(period)

    def _settled_pose(self, buf, names, q) -> Optional[np.ndarray]:
        """Stream q for settle_s + window_s; return the average measured_cp pose over the last window_s."""
        self._stream_jp(names, q, self.settle)
        t0 = time.monotonic()
        self._stream_jp(names, q, self.window)
        samples = buf.since(t0)
        if not samples:
            return None
        return G.average_pose([pose_msg_to_matrix(m) for _, m in samples])

    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "dimensional", Outcome.UNDETERMINED)
        de = self.exp.dimensional
        res.observations["expectation"] = self.exp.to_dict()["dimensional"]
        if de.mode != "geometry_anchor":
            res.decision_basis = "dimensional.mode is not geometry_anchor: probe not applicable"
            res.duration_s = time.time() - t0
            return res
        disc = self.a.discovery or self.a.discover()
        for need in ("measured_cp", "measured_js", "servo_jp"):
            if not disc["topics"][need]["present"]:
                res.notes.append(f"{need} missing: the geometry anchor needs measured_cp, measured_js and servo_jp")
                res.decision_basis = "missing topics"
                res.duration_s = time.time() - t0
                return res
        buf = self.a.subscribe("measured_cp")
        buf_js = self.a.subscribe("measured_js")
        res.observations["enable"] = ensure_enabled(self.a)
        js = self.a.wait_for(buf_js, 2.0)
        if js is None:
            res.decision_basis = "no measured_js received"
            res.duration_s = time.time() - t0
            return res
        q_now = np.array(js.position, dtype=float)
        # measured_js may list more names than positions (SRC lists the two gripper links): the joints are the first len(position)
        names = list(js.name)[:len(q_now)]
        ip, iy = _joint_index(names, de.pitch_joint), _joint_index(names, de.yaw_joint)
        res.observations["joint_names"] = names
        res.observations["pitch_joint_index"], res.observations["yaw_joint_index"] = ip, iy
        if ip is None or iy is None or ip == iy:
            res.notes.append(f"could not identify the wrist joints {de.pitch_joint!r} / {de.yaw_joint!r} among {names}")
            res.decision_basis = "wrist joints not identified"
            res.duration_s = time.time() - t0
            return res
        q_ref = np.array(de.reference_joints, dtype=float) if de.reference_joints is not None else q_now.copy()
        if len(q_ref) != len(names):
            res.decision_basis = f"reference_joints has {len(q_ref)} values, measured_js {len(names)}"
            res.duration_s = time.time() - t0
            return res
        res.observations["reference_joints"] = q_ref.tolist()
        dq = float(de.delta_q_rad)
        pitch_axes, yaw_axes, trials = [], [], []
        for k in range(self.trials):
            entry = {"trial": k}
            try:
                T0 = self._settled_pose(buf, names, q_ref)
                qp = q_ref.copy(); qp[ip] += dq
                Tp = self._settled_pose(buf, names, qp)
                T0b = self._settled_pose(buf, names, q_ref)
                qy = q_ref.copy(); qy[iy] += dq
                Ty = self._settled_pose(buf, names, qy)
                if any(T is None for T in (T0, Tp, T0b, Ty)):
                    entry["ok"] = False
                    entry["reason"] = "no measured_cp sample in a settle window"
                    trials.append(entry)
                    continue
                ap, ay = screw_axis(T0, Tp), screw_axis(T0b, Ty)
                pitch_axes.append(ap)
                yaw_axes.append(ay)
                entry.update(ok=True, pitch_axis=ap.__dict__, yaw_axis=ay.__dict__)
            except ValueError as e:
                entry.update(ok=False, reason=str(e))
            trials.append(entry)
        self._stream_jp(names, q_ref, self.settle)
        res.observations["trials"] = trials
        expected = de.expected_unit_m if de.declared else None
        dec = geometry_anchor_decision(pitch_axes, yaw_axes, dq, float(de.L_m), float(de.u_rel), expected, self.tol,
                                       axes_angle_deg=de.axes_angle_deg)
        res.estimates["geometry_anchor"] = dec.to_dict()
        res.estimates["L_source"] = de.L_source
        res.observations["assumptions"] = dec.assumptions
        res.outcome = Outcome(dec.outcome)
        if dec.gate_failures:
            res.notes.extend(dec.gate_failures)
        if not dec.gates_passed:
            res.decision_basis = "geometry anchor gates failed or too few trials -> undetermined"
        else:
            lo, hi = dec.lambda_ci_widened_m
            res.predicted_error_m = dec.predicted_error_m
            res.predicted_error_ci = dec.predicted_error_ci_m
            res.decision_basis = (f"instrument-geometry anchor: common normal of the wrist axes d_int = {dec.d_int_mean_if:.6g} interface units, "
                                  f"L = {dec.L_phys_m*1e3:.3f} mm -> lambda_hat = {dec.lambda_hat_m:.6g} m per unit "
                                  f"(95% {dec.lambda_ci_m[0]:.6g}..{dec.lambda_ci_m[1]:.6g}, widened by u_rel = {dec.u_rel:g} to {lo:.6g}..{hi:.6g}); "
                                  f"eq. (6) against the client's unit {dec.expected_unit_m}: |1-s| r_ws in {dec.predicted_error_ci_m[0]*1e3:.3f}..{dec.predicted_error_ci_m[1]*1e3:.3f} mm "
                                  f"vs epsilon = {self.tol.epsilon_m*1e3:.3f} mm")
        res.duration_s = time.time() - t0
        return res
