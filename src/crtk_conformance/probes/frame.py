"""FrameSemanticsProbe — spatial binding class.

Question answered: in which frame are the *unqualified* Cartesian topics (`measured_cp`,
`servo_cp`) expressed, relative to the arm-base (RCM-origin) frame that CRTK calls `local/`?

Estimator (when `local/measured_cp` exists, e.g. dVRK):  T_hat = measured_cp * local_measured_cp^-1
averaged over samples; this is exactly the dVRK `base_frame` (measured_cp = base_frame * local).
When `local/` does not exist the binding cannot be verified through the interface: the probe
reports the frame_id strings, any `T_b_w` transform and any /tf chain it can find, and returns
UNDETERMINED. It never guesses.
"""
from __future__ import annotations

import math
import time
from typing import List

import numpy as np

from .. import geometry as G
from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..stats import estimate
from ..thresholds import Tolerance
from .base import Outcome, ProbeResult, decide


class FrameSemanticsProbe:
    name = "FrameSemanticsProbe"

    def __init__(self, adapter: PlatformAdapter, tol: Tolerance, trials: int = 10, samples_per_trial: int = 5, timeout_s: float = 2.0):
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.m = samples_per_trial
        self.timeout = timeout_s

    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "spatial", Outcome.UNDETERMINED)
        disc = self.a.discovery or self.a.discover()
        topics = disc["topics"]
        res.observations["discovery"] = {k: v for k, v in topics.items() if k in ("measured_cp", "local/measured_cp", "T_b_w", "setpoint_cp", "servo_cp")}

        if not topics["measured_cp"]["present"]:
            res.notes.append("measured_cp not present: nothing to test")
            res.decision_basis = "missing measured_cp"
            res.duration_s = time.time() - t0
            return res

        buf_m = self.a.subscribe("measured_cp")
        frame_ids = {}
        msg = self.a.wait_for(buf_m, self.timeout)
        if msg is None:
            res.notes.append("measured_cp present but no message received within timeout")
            res.decision_basis = "no data"
            res.duration_s = time.time() - t0
            return res
        frame_ids["measured_cp"] = msg.header.frame_id

        if topics["local/measured_cp"]["present"]:
            buf_l = self.a.subscribe("local/measured_cp")
            ml = self.a.wait_for(buf_l, self.timeout)
            if ml is None:
                res.notes.append("local/measured_cp present but silent")
                res.decision_basis = "no local data"
                res.observations["frame_ids"] = frame_ids
                res.duration_s = time.time() - t0
                return res
            frame_ids["local/measured_cp"] = ml.header.frame_id
            trial_T: List[np.ndarray] = []
            trial_pred: List[float] = []
            trial_tnorm: List[float] = []
            trial_theta: List[float] = []
            for _ in range(self.trials):
                Ts = []
                deadline = time.time() + self.timeout
                while len(Ts) < self.m and time.time() < deadline:
                    mm = self.a.wait_for(buf_m, self.timeout)
                    if mm is None:
                        break
                    # pair with the local sample closest in header stamp
                    cands = buf_l.since(time.monotonic() - 0.5)
                    if not cands:
                        continue
                    ml = min(cands, key=lambda c: abs((c[1].header.stamp - mm.header.stamp).to_sec()))[1]
                    if abs((ml.header.stamp - mm.header.stamp).to_sec()) > 0.05:
                        continue
                    Ts.append(pose_msg_to_matrix(mm) @ G.invert(pose_msg_to_matrix(ml)))
                if not Ts:
                    continue
                T = G.average_pose(Ts)
                trial_T.append(T)
                tn = float(np.linalg.norm(T[:3, 3]))
                th = G.rotation_angle(T[:3, :3])
                trial_tnorm.append(tn)
                trial_theta.append(th)
                trial_pred.append(self.tol.spatial_error(tn, th))
            if not trial_T:
                res.notes.append("could not pair measured_cp with local/measured_cp samples")
                res.decision_basis = "no paired data"
            else:
                T_hat = G.average_pose(trial_T)
                e_t = estimate(trial_tnorm)
                e_th = estimate([math.degrees(x) for x in trial_theta])
                e_pred = estimate(trial_pred)
                res.estimates = {
                    "binding_translation_m": [float(v) for v in T_hat[:3, 3]],
                    "binding_translation_norm_m": e_t.to_dict(),
                    "binding_rotation_deg": e_th.to_dict(),
                    "binding_matrix": T_hat.tolist(),
                    "predicted_abs_error_at_workspace_edge_m": e_pred.to_dict(),
                }
                res.predicted_error_m = e_pred.mean
                res.predicted_error_ci = [e_pred.ci_low, e_pred.ci_high]
                res.outcome = decide(e_pred.ci_low, e_pred.ci_high, self.tol.epsilon_m)
                res.decision_basis = (
                    f"eq. (3): ||t_hat|| + 2 sin(theta_hat/2) r_ws = {e_pred.mean*1e3:.3f} mm "
                    f"(95% CI {e_pred.ci_low*1e3:.3f}..{e_pred.ci_high*1e3:.3f}) vs epsilon = {self.tol.epsilon_m*1e3:.3f} mm"
                )
                if frame_ids["measured_cp"] == frame_ids["local/measured_cp"] and e_t.mean > 3 * max(e_t.std, 1e-9):
                    res.notes.append("measured_cp and local/measured_cp carry the same frame_id but differ by a non-zero transform: frame_id does not disambiguate the binding")
        else:
            res.notes.append("no local/measured_cp topic: the binding of the unqualified topics relative to the arm base cannot be verified through the interface")
            res.decision_basis = "no local/ reference available -> undetermined by construction"
            if topics["T_b_w"]["present"]:
                buf_b = self.a.subscribe("T_b_w")
                mb = self.a.wait_for(buf_b, self.timeout)
                if mb is not None:
                    Tb = pose_msg_to_matrix(mb)
                    frame_ids["T_b_w"] = mb.header.frame_id
                    res.estimates["T_b_w_translation_m"] = [float(v) for v in Tb[:3, 3]]
                    res.estimates["T_b_w_rotation_deg"] = math.degrees(G.rotation_angle(Tb[:3, :3]))
                    res.notes.append("T_b_w present: interface exposes base-in-world, which suggests (but does not prove) an arm-base binding of measured_cp")
            if disc.get("tf_present"):
                self.a.subscribe_tf()
                time.sleep(min(1.0, self.timeout))
                T_tf = self.a.tf_lookup("world", frame_ids["measured_cp"])
                if T_tf is not None:
                    res.estimates["tf_world_to_measured_frame_translation_m"] = [float(v) for v in T_tf[:3, 3]]
                    res.estimates["tf_world_to_measured_frame_rotation_deg"] = math.degrees(G.rotation_angle(T_tf[:3, :3]))
                    res.notes.append("/tf provides world->measured_cp.frame_id; not a substitute for local/ (does not identify the RCM frame)")
        res.observations["frame_ids"] = frame_ids
        res.duration_s = time.time() - t0
        return res
