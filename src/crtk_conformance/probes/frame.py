"""FrameSemanticsProbe — spatial binding class (0.1.1: client-relative decision).

Question answered: in which frame are the *unqualified* Cartesian topics expressed, relative to the
arm-base (RCM-origin) frame that CRTK calls `local/`?

Estimator (when `local/measured_cp` exists, e.g. dVRK):  T_hat = measured_cp * local_measured_cp^-1
averaged over stamp-paired samples; this is exactly the dVRK `base_frame` (measured_cp = base_frame * local).
The probe is PASSIVE: it never publishes servo_cp, so it observes the binding of measured_cp only. That
servo_cp shares the binding is supported for the dVRK and SRC by primary evidence (paper Sec. 4.1), not
by this probe.

Decision (0.1.2): only against a declared client expectation (expectations.py):
  identity / expected_transform  -> residual E = T_hat * T_expected^-1; EXACT maximum positional error over the
                                    ball ||p|| <= r_ws (eq. 3', spatial.py) with an interval propagated from
                                    Hotelling T^2 regions of the per-trial residual parameters, vs epsilon.
                                    The translation is converted from interface units with the client's declared
                                    unit (an assumption the report states).  Positional only unless
                                    spatial.orientation_tolerance_deg is declared.
  discover_only (default)        -> T_hat reported, outcome UNDETERMINED (no expectation, no verdict).
When `local/` does not exist the binding cannot be verified through the interface: the probe reports the
frame_id strings, any `T_b_w` transform and any /tf chain it can find, and returns UNDETERMINED whatever
the expectation. It never guesses.
"""
from __future__ import annotations

import math
import time
from typing import List, Optional

import numpy as np

from .. import geometry as G
from ..adapter import PlatformAdapter, pose_msg_to_matrix
from ..stats import estimate
from ..thresholds import Tolerance
from ..expectations import Expectations
from .base import Outcome, ProbeResult, decide


class FrameSemanticsProbe:
    name = "FrameSemanticsProbe"

    def __init__(self, adapter: PlatformAdapter, tol: Tolerance, trials: int = 10, samples_per_trial: int = 5, timeout_s: float = 2.0,
                 expectations: Optional[Expectations] = None, pairing_window_s: float = 0.05):
        self.a = adapter
        self.tol = tol
        self.trials = trials
        self.m = samples_per_trial
        self.timeout = timeout_s
        self.exp = expectations or Expectations()
        self.pairing_window = pairing_window_s  # implementation constant: max |stamp difference| for a measured/local pair

    def run(self) -> ProbeResult:
        t0 = time.time()
        res = ProbeResult(self.name, "spatial", Outcome.UNDETERMINED)
        res.observations["expectation"] = self.exp.to_dict()["spatial"]
        T_exp = self.exp.spatial.matrix()
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
            trial_tnorm: List[float] = []
            trial_theta: List[float] = []
            unpaired = 0
            zero_stamps = 0
            for _ in range(self.trials):
                Ts = []
                deadline = time.time() + self.timeout
                while len(Ts) < self.m and time.time() < deadline:
                    mm = self.a.wait_for(buf_m, self.timeout)
                    if mm is None:
                        break
                    # 0.1.6: an unset header stamp (0) cannot be paired by time.  The released dVRK publishes an identity
                    # pose with stamp 0 on both topics until the arm is homed; two zero stamps would pair trivially and
                    # report T_hat = I (RC9 feasibility check).  Such samples are skipped and counted.
                    if mm.header.stamp.to_nsec() == 0:
                        zero_stamps += 1
                        continue
                    # pair with the local sample closest in header stamp
                    cands = [c for c in buf_l.since(time.monotonic() - 0.5) if c[1].header.stamp.to_nsec() != 0]
                    if not cands:
                        continue
                    ml = min(cands, key=lambda c: abs((c[1].header.stamp - mm.header.stamp).to_sec()))[1]
                    if abs((ml.header.stamp - mm.header.stamp).to_sec()) > self.pairing_window:
                        unpaired += 1
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
            if zero_stamps:
                res.notes.append(f"{zero_stamps} measured_cp sample(s) with an unset (zero) header stamp were skipped: they cannot be paired by time")
            res.observations["zero_stamp_samples_skipped"] = zero_stamps
            if not trial_T:
                res.notes.append("could not pair measured_cp with local/measured_cp samples")
                res.decision_basis = "no paired data"
            else:
                # 0.1.2: the quotient's translation is in INTERFACE units; the client's declared unit converts it to
                # metres.  A spatial verdict therefore assumes that unit (recorded), it does not establish it.
                unit = float(self.exp.dimensional.expected_unit_m)
                trial_T = [T.copy() for T in trial_T]
                for T in trial_T:
                    T[:3, 3] *= unit
                T_hat = G.average_pose(trial_T)
                # 0.1.3: the per-trial norms are formed AFTER the unit conversion (0.1.2 formed them in interface
                # units while the matrix was converted; identical for a declared unit of 1.0, wrong otherwise)
                e_t = estimate([float(np.linalg.norm(T[:3, 3])) for T in trial_T])
                e_th = estimate([math.degrees(x) for x in trial_theta])
                res.estimates = {
                    "binding_translation_m": [float(v) for v in T_hat[:3, 3]],
                    "binding_translation_norm_m": e_t.to_dict(),
                    "binding_rotation_deg": e_th.to_dict(),
                    "binding_matrix": T_hat.tolist(),
                    "unpaired_samples": unpaired,
                    "assumed_interface_unit_m": unit,
                }
                res.observations["assumptions"] = [
                    f"translations interpreted with the declared interface unit {unit} m per unit (dimensional.expected_unit_m); not established by this probe",
                    "the probe is passive: it observes the binding of measured_cp relative to local/measured_cp; that servo_cp is interpreted in the same frame is not tested here",
                    "the verdict is positional (maximum positional error over the ball ||p|| <= r_ws); orientation is decided only if spatial.orientation_tolerance_deg is declared",
                ]
                if T_exp is None:
                    res.decision_basis = "no spatial expectation declared (discover-only): binding T_hat reported, no conformance verdict"
                    res.notes.append("observed binding of measured_cp relative to local/measured_cp: ||t_hat|| = %.3f mm, theta_hat = %.3f deg" % (float(np.linalg.norm(T_hat[:3, 3])) * 1e3, math.degrees(G.rotation_angle(T_hat[:3, :3]))))
                else:
                    from ..spatial import spatial_decision, rotation_angle_interval
                    T_exp_inv = G.invert(T_exp)
                    residuals = [T @ T_exp_inv for T in trial_T]
                    sd = spatial_decision(residuals, self.tol.workspace_radius_m)
                    res.estimates["expected_binding_matrix"] = T_exp.tolist()
                    res.estimates["residual_translation_norm_m"] = float(np.linalg.norm(sd.residual_translation_m))
                    res.estimates["residual_rotation_deg"] = sd.residual_rotation_deg
                    res.estimates["spatial_decision"] = sd.to_dict()
                    # 0.1.3 (RC4 review, finding 7): the interval's distributional model is part of the report
                    res.observations["assumptions"].extend(sd.assumptions)
                    # kept for readers of 0.1.1 reports: same keys, now carrying the exact maximum error and its
                    # propagated interval (not eq. (3) and not the re-centred Student-t half-width)
                    res.estimates["predicted_abs_error_at_workspace_edge_m"] = {"n": sd.n, "mean": sd.e_max_m, "std": None, "ci_low": sd.ci_low_m, "ci_high": sd.ci_high_m, "alpha": sd.alpha,
                                                                                "statistic": "exact maximum positional error over the ball (eq. 3'), interval propagated from Hotelling T^2 regions"}
                    res.predicted_error_m = sd.e_max_m
                    res.predicted_error_ci = [sd.ci_low_m, sd.ci_high_m]
                    pos = decide(sd.ci_low_m, sd.ci_high_m, self.tol.epsilon_m)
                    if not math.isfinite(sd.ci_high_m):
                        res.notes.append("n = %d trials <= 3: the Hotelling T^2 region is undefined; at least 4 trials are needed for a verdict" % sd.n)
                    basis = (f"eq. (3') on the residual T_hat * T_expected^-1 ({self.exp.spatial.mode}): exact max positional error over ||p|| <= r_ws = {sd.e_max_m*1e3:.3f} mm "
                             f"(>= 95% region {sd.ci_low_m*1e3:.3f}..{sd.ci_high_m*1e3:.3f} mm; eq. (3) bound would be {sd.bound_eq3_m*1e3:.3f} mm) vs epsilon = {self.tol.epsilon_m*1e3:.3f} mm")
                    ot = self.exp.spatial.orientation_tolerance_deg
                    if ot is not None:
                        th, th_lo, th_hi = rotation_angle_interval(residuals)
                        res.estimates["residual_rotation_interval_deg"] = [math.degrees(th_lo), math.degrees(th_hi)]
                        ori = decide(math.degrees(th_lo), math.degrees(th_hi), float(ot))
                        basis += f"; orientation: residual angle {math.degrees(th):.3f} deg ({math.degrees(th_lo):.3f}..{math.degrees(th_hi):.3f}) vs {ot} deg"
                        res.estimates["orientation_outcome"] = ori.value
                        if ori == Outcome.DIVERGENT or pos == Outcome.DIVERGENT:
                            res.outcome = Outcome.DIVERGENT
                        elif ori == Outcome.CONFORMANT and pos == Outcome.CONFORMANT:
                            res.outcome = Outcome.CONFORMANT
                        else:
                            res.outcome = Outcome.UNDETERMINED
                    else:
                        res.outcome = pos
                        basis += "; positional verdict only (no orientation tolerance declared)"
                    res.decision_basis = basis
                if frame_ids["measured_cp"] == frame_ids["local/measured_cp"] and e_t.mean > 3 * max(e_t.std, 1e-9):
                    res.notes.append("measured_cp and local/measured_cp carry the same frame_id but differ by a non-zero transform: frame_id does not disambiguate the binding")
        else:
            res.notes.append("no local/measured_cp topic: the binding of the unqualified topics relative to the arm base cannot be verified through the interface")
            res.decision_basis = "no local/ reference available -> undetermined by construction (whatever the declared expectation)"
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
