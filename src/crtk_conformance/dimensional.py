"""Dimensional decisions that need no ROS (0.1.6).

`scale_decision` is the eq. (6) decision of ScalingUnitsProbe, moved here unchanged so that the geometry anchor and the
offline boundary study (validation/boundary_montecarlo_v016.py) use the same rule; tests/test_dimensional_v016.py
replays every archived v0.1.3 anchored-scale run through it and checks identical outcomes.

The instrument-geometry anchor (0.1.6; RC9 / external review, point 6) grounds the interface unit in a known physical
length of the instrument instead of an out-of-band SI pose.  Two wrist joints are stepped by a known angle with
`servo_jp`; from the full pose change of `measured_cp` each step is a rotation about a fixed line (its screw axis).  The
common-normal distance between the wrist-pitch and wrist-yaw axes is, in modified DH, exactly the `A` parameter of the
wrist-yaw row -- the pitch-to-yaw length L of the instrument -- and does not depend on which rigidly attached point the
interface publishes (a chord of the published point would: SRC v1.0.0 publishes a control point beyond the yaw axis).
Angles are unit-free, so lambda_hat = L_phys / d_int is the physical length of one interface unit.  The declared
relative uncertainty u_rel of L_phys (released instrument models differ: dVRK 2.4.0 uses 9.1 mm, SRC 9.0 mm) widens
the interval multiplicatively; it is an input, never tuned to a result.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .probes.base import Outcome, decide
from .stats import Estimate, estimate


def scale_decision(e_s: Estimate, expected_unit_m: float, tol) -> Tuple[Outcome, Estimate]:
    """Eq. (6) decision on an anchored unit estimate (moved from ScalingUnitsProbe.run in 0.1.6, logic unchanged).

    e_s: the estimate of the metres per interface unit (mean with a two-sided interval); expected_unit_m: the unit the
    client assumes; tol: a Tolerance.  The scale divergence relative to the client's unit is s = s_hat / u and the
    predicted error |1 - s| r_ws is formed from the MEAN estimate with its interval mapped through the model (per-trial
    |1 - s_i| would bias upward); the interval's lower end is 0 when it contains u.
    """
    u = float(expected_unit_m)
    cands = [tol.dimensional_error(v / u) for v in (e_s.ci_low, e_s.ci_high)]
    lo = 0.0 if (e_s.ci_low <= u <= e_s.ci_high) else min(cands)
    e_pred = Estimate(e_s.n, tol.dimensional_error(e_s.mean / u), float("nan"), lo, max(cands), e_s.alpha)
    return decide(e_pred.ci_low, e_pred.ci_high, tol.epsilon_m), e_pred


# ------------------------------------------------------------------ instrument-geometry anchor
def _orthonormalise(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(np.asarray(R, dtype=float))
    Rn = U @ Vt
    if np.linalg.det(Rn) < 0:
        U[:, -1] *= -1
        Rn = U @ Vt
    return Rn


def rotation_vector_angle(R: np.ndarray) -> float:
    """Rotation angle from the skew part and the trace (atan2): no acos precision floor near the identity."""
    Rn = _orthonormalise(R)
    w = np.array([Rn[2, 1] - Rn[1, 2], Rn[0, 2] - Rn[2, 0], Rn[1, 0] - Rn[0, 1]])
    return float(math.atan2(np.linalg.norm(w) / 2.0, (np.trace(Rn) - 1.0) / 2.0))


@dataclass
class ScrewAxis:
    point: List[float]  # a point on the axis (reference-frame coordinates, interface units)
    direction: List[float]  # unit direction
    angle_rad: float  # rotation angle of the step
    slide_if: float  # translation along the axis (0 for a pure rotation about a fixed line)


def screw_axis(T0: np.ndarray, T1: np.ndarray) -> ScrewAxis:
    """Axis of the rigid motion M = T1 T0^-1 (both poses of the same moving frame in the same reference frame).

    For a rotation about a fixed line through c with direction k: M p = R (p - c) + c, so t = (I - R) c and the point
    is recovered in the plane perpendicular to k with the pseudo-inverse of (I - R); the component of t along k is the
    slide (zero for a revolute joint).
    """
    M = np.asarray(T1, dtype=float) @ np.linalg.inv(np.asarray(T0, dtype=float))
    R = _orthonormalise(M[:3, :3])
    t = M[:3, 3]
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    nw = float(np.linalg.norm(w))
    if nw < 1e-12:
        raise ValueError("step produced no rotation: the screw axis is undefined")
    k = w / nw
    slide = float(k @ t)
    c = np.linalg.pinv(np.eye(3) - R) @ (t - slide * k)
    return ScrewAxis(point=c.tolist(), direction=k.tolist(), angle_rad=rotation_vector_angle(R), slide_if=abs(slide))


def common_normal(a: ScrewAxis, b: ScrewAxis) -> Tuple[float, float]:
    """(distance along the common normal, sin of the angle between the axes).  Undefined for parallel axes."""
    ka, kb = np.asarray(a.direction), np.asarray(b.direction)
    n = np.cross(ka, kb)
    s = float(np.linalg.norm(n))
    if s < 1e-6:
        raise ValueError("axes are (nearly) parallel: the common normal is not unique")
    d = float(abs((np.asarray(b.point) - np.asarray(a.point)) @ (n / s)))
    return d, s


@dataclass
class GeometryAnchorDecision:
    n: int
    d_int_mean_if: float
    lambda_hat_m: float  # metres per interface unit (mean of per-trial L / d_int)
    lambda_ci_m: List[float]  # Student-t interval of the mean, before the geometry uncertainty
    lambda_ci_widened_m: List[float]  # after the multiplicative widening by (1 +- u_rel)
    u_rel: float
    L_phys_m: float
    expected_unit_m: float
    predicted_error_m: float
    predicted_error_ci_m: List[float]
    outcome: str
    gates_passed: bool
    gate_failures: List[str] = field(default_factory=list)
    assumptions: List[str] = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


def geometry_anchor_decision(pitch_axes: Sequence[ScrewAxis], yaw_axes: Sequence[ScrewAxis], delta_q_rad: float, L_phys_m: float, u_rel: float,
                             expected_unit_m: Optional[float], tol, angle_tol_rad: float = 0.02, slide_tol_rel: float = 0.02,
                             axes_angle_deg: Optional[float] = 90.0, axes_angle_tol_deg: float = 2.0, alpha: float = 0.05) -> GeometryAnchorDecision:
    """Per-trial common-normal distances -> lambda_hat interval -> eq. (6) decision (or discover-only when
    expected_unit_m is None).

    Gates (any failure -> undetermined, reported): each step's rotation angle equals |delta_q| within angle_tol_rad (the
    commanded joint moved by the commanded angle); each step is a pure rotation (slide <= slide_tol_rel x d_int); the
    angle between the two axes matches the instrument model within axes_angle_tol_deg (90 deg for the Large Needle
    Driver).  The gates do not identify the published point; the common normal does not need it.
    """
    fails: List[str] = []
    n = min(len(pitch_axes), len(yaw_axes))
    ds, sins = [], []
    for i in range(n):
        try:
            d, s = common_normal(pitch_axes[i], yaw_axes[i])
        except ValueError as e:
            fails.append(f"trial {i}: {e}")
            continue
        ds.append(d)
        sins.append(s)
    for i, ax in enumerate(list(pitch_axes[:n]) + list(yaw_axes[:n])):
        if abs(ax.angle_rad - abs(delta_q_rad)) > angle_tol_rad:
            fails.append(f"step {i}: rotation {math.degrees(ax.angle_rad):.3f} deg differs from the commanded {math.degrees(abs(delta_q_rad)):.3f} deg")
    dmean = float(np.mean(ds)) if ds else float("nan")
    for i, ax in enumerate(list(pitch_axes[:n]) + list(yaw_axes[:n])):
        if ds and ax.slide_if > slide_tol_rel * dmean:
            fails.append(f"step {i}: translation along the axis {ax.slide_if:.3g} exceeds {slide_tol_rel:g} x d_int (not a pure rotation)")
    if axes_angle_deg is not None:
        for i, s in enumerate(sins):
            ang = math.degrees(math.asin(min(1.0, s)))
            if abs(ang - axes_angle_deg) > axes_angle_tol_deg:
                fails.append(f"trial {i}: axes at {ang:.2f} deg, model {axes_angle_deg} deg")
    lam = [L_phys_m / d for d in ds if d > 0]
    e = estimate(lam, alpha)
    lo_w, hi_w = e.ci_low * (1.0 - u_rel), e.ci_high * (1.0 + u_rel)
    assumptions = [f"the published pose is rigidly attached to the distal link of the stepped wrist joints (orientation published)",
                   f"the implementation's wrist geometry equals the instrument's: pitch-to-yaw length L = {L_phys_m*1e3:.2f} mm +- {u_rel*100:.2f} % (declared)",
                   "angles are unit-free; lambda_hat = L / d_int is the physical length of one interface unit",
                   f"per-trial estimates are iid with a Student-t interval of the mean (alpha = {alpha})"]
    gates = not fails and e.n >= 3
    if e.n < 3:
        fails.append(f"only {e.n} trial(s) produced both axes; at least 3 are needed")
    if expected_unit_m is None or not gates:
        return GeometryAnchorDecision(e.n, dmean, e.mean, [e.ci_low, e.ci_high], [lo_w, hi_w], u_rel, L_phys_m, expected_unit_m if expected_unit_m else float("nan"),
                                      float("nan"), [float("nan"), float("nan")], Outcome.UNDETERMINED.value, gates, fails, assumptions)
    widened = Estimate(e.n, e.mean, e.std, lo_w, hi_w, alpha)
    outcome, e_pred = scale_decision(widened, expected_unit_m, tol)
    return GeometryAnchorDecision(e.n, dmean, e.mean, [e.ci_low, e.ci_high], [lo_w, hi_w], u_rel, L_phys_m, float(expected_unit_m),
                                  e_pred.mean, [e_pred.ci_low, e_pred.ci_high], outcome.value, gates, fails, assumptions)


def consistency_gate(outcome: Outcome, flagged: bool) -> Tuple[Outcome, bool]:
    """0.1.7 (RC12 review, point 1): a unit verdict assumes that commands and feedback share one binding (the scale probe
    moves the arm with absolute goals; the geometry anchor measures the feedback unit).  When the command/feedback
    consistency diagnostic is raised that assumption is contradicted, and a conformant or divergent unit verdict would
    attribute an inconsistency of unknown cause to units (case K1: a command-frame error reported as a unit error).
    Returns (outcome, withheld): undetermined and True when a determinate verdict is withheld."""
    if flagged and outcome in (Outcome.CONFORMANT, Outcome.DIVERGENT):
        return Outcome.UNDETERMINED, True
    return outcome, False
