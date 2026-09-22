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


# ------------------------------------------------------------------ joint-space (product-of-exponentials) anchor, 0.1.8
# RC13 external review, point 3: on SRC v1.0.0 a commanded wrist step also moved other joints and the yaw step fell short,
# so the pose change of a step was not a rotation about one fixed axis and the single-joint anchor abstained.  The
# joint-space anchor uses the MEASURED joint changes of every joint instead: for a serial chain, with the measured
# reference configuration q_ref as the home configuration, T(q) = exp([xi_1](q_1 - q_ref,1)) ... exp([xi_n](q_n - q_ref,n))
# T(q_ref) exactly (product of exponentials), where xi_j are the spatial twists of the joints at q_ref.  Each trial steps
# every joint by +-delta_j from q_ref, fits all twists by least squares to the measured (q, T) pairs, and takes the common
# normal of the two wrist axes, a link parameter that does not depend on the configuration.  Coupled or incomplete joint
# motion is then part of the data, not a violation of the model; the model's fit residual is gated instead.

def _hat(w) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def se3_exp(w, v, theta: float) -> np.ndarray:
    """exp of the twist (w, v) scaled by theta; w a unit vector (revolute) or zero (prismatic).  Closed form (Murray, Li
    and Sastry 1994, prop. 2.9): R = I + sin(theta) W + (1 - cos(theta)) W^2, p = (I - R)(w x v) + w w^T v theta."""
    wx, wy, wz = float(w[0]), float(w[1]), float(w[2])
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    T = np.eye(4)
    if wx * wx + wy * wy + wz * wz < 1e-24:
        T[0, 3], T[1, 3], T[2, 3] = vx * theta, vy * theta, vz * theta
        return T
    s, c1 = math.sin(theta), 1.0 - math.cos(theta)
    R = [[1.0 - c1 * (wy * wy + wz * wz), -s * wz + c1 * wx * wy, s * wy + c1 * wx * wz],
         [s * wz + c1 * wx * wy, 1.0 - c1 * (wx * wx + wz * wz), -s * wx + c1 * wy * wz],
         [-s * wy + c1 * wx * wz, s * wx + c1 * wy * wz, 1.0 - c1 * (wx * wx + wy * wy)]]
    cx, cy, cz = wy * vz - wz * vy, wz * vx - wx * vz, wx * vy - wy * vx  # w x v
    wv = (wx * vx + wy * vy + wz * vz) * theta
    for r in range(3):
        T[r, 0], T[r, 1], T[r, 2] = R[r]
    T[0, 3] = cx - (R[0][0] * cx + R[0][1] * cy + R[0][2] * cz) + wx * wv
    T[1, 3] = cy - (R[1][0] * cx + R[1][1] * cy + R[1][2] * cz) + wy * wv
    T[2, 3] = cz - (R[2][0] * cx + R[2][1] * cy + R[2][2] * cz) + wz * wv
    return T


def _rotvec(R: np.ndarray) -> np.ndarray:
    Rn = _orthonormalise(R)
    ang = rotation_vector_angle(Rn)
    w = np.array([Rn[2, 1] - Rn[1, 2], Rn[0, 2] - Rn[2, 0], Rn[1, 0] - Rn[0, 1]])
    nw = float(np.linalg.norm(w))
    if nw < 1e-15:
        return np.zeros(3)
    return w / nw * ang


@dataclass
class PoeTrialFit:
    ok: bool
    d_int_if: float = float("nan")  # common-normal distance of the two wrist axes, interface units
    axes_angle_deg: float = float("nan")
    rms_rot_rad: float = float("nan")  # fit residuals over all steps
    rms_trans_rel: float = float("nan")  # translation residual / d_int
    max_rot_rad: float = float("nan")
    max_trans_rel: float = float("nan")
    wrist_excitation_rad: List[float] = field(default_factory=list)  # smallest realized |dq| of each wrist joint in its own steps
    n_steps: int = 0
    reason: str = ""
    twists: List[List[float]] = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


def poe_fit_trial(joint_types: str, q_ref: Sequence[float], T_ref: np.ndarray, qs: Sequence[Sequence[float]], Ts: Sequence[np.ndarray],
                  pitch_index: int, yaw_index: int) -> PoeTrialFit:
    """Fit the spatial twists of every joint at the measured reference configuration q_ref to the measured (q, T) steps,
    and return the common normal of the two wrist axes.  joint_types: one letter per joint in chain order, 'R' or 'P'.
    The initial guess of joint j is the screw of the step with the largest |dq_j| (dimensional.screw_axis)."""
    from scipy.optimize import least_squares
    n = len(joint_types)
    q_ref = np.asarray(q_ref, dtype=float)
    dQ = [np.asarray(q, dtype=float) - q_ref for q in qs]
    Tr = np.asarray(T_ref, dtype=float)
    if len(dQ) < 2 * n:
        return PoeTrialFit(False, n_steps=len(dQ), reason=f"{len(dQ)} steps for {n} joints; at least {2 * n} are needed")
    # initial guess and a length scale for the translation residuals
    x0 = []
    for j, jt in enumerate(joint_types):
        k = int(np.argmax([abs(d[j]) for d in dQ]))
        dj = dQ[k][j]
        if abs(dj) < 1e-9:
            return PoeTrialFit(False, n_steps=len(dQ), reason=f"joint {j} did not move in any step")
        if jt == "R":
            try:
                ax = screw_axis(Tr, Ts[k])
                w0 = np.asarray(ax.direction) * (1.0 if dj > 0 else -1.0)
                p0 = np.asarray(ax.point)
            except ValueError:
                w0, p0 = np.array([0.0, 0.0, 1.0]), Tr[:3, 3].copy()
            x0.extend(list(w0) + list(p0))
        else:
            t = (np.asarray(Ts[k])[:3, 3] - Tr[:3, 3]) / dj
            nt = float(np.linalg.norm(t))
            x0.extend(list(t / nt if nt > 0 else np.array([0.0, 0.0, 1.0])))
    x0 = np.array(x0, dtype=float)

    def unpack(x):
        tw, i = [], 0
        for jt in joint_types:
            if jt == "R":
                u, p = x[i:i + 3], x[i + 3:i + 6]
                w = u / max(np.linalg.norm(u), 1e-12)
                tw.append((w, -np.cross(w, p), p))
                i += 6
            else:
                u = x[i:i + 3]
                tw.append((np.zeros(3), u / max(np.linalg.norm(u), 1e-12), None))
                i += 3
        return tw

    # length scale: the initial wrist common normal, else the size of the translations
    try:
        tw0 = unpack(x0)
        c0 = np.cross(tw0[pitch_index][0], tw0[yaw_index][0])
        sc = abs(float((tw0[yaw_index][2] - tw0[pitch_index][2]) @ (c0 / np.linalg.norm(c0))))
    except Exception:
        sc = 0.0
    if not (sc > 0 and math.isfinite(sc)):
        sc = max(1e-9, float(np.median([np.linalg.norm(np.asarray(T)[:3, 3] - Tr[:3, 3]) for T in Ts])))

    def predict(tw, d):
        T = np.eye(4)
        for (w, v, _), th in zip(tw, d):
            T = T @ se3_exp(w, v, th)
        return T @ Tr

    Tms = [np.asarray(T, dtype=float) for T in Ts]

    def resid(x):
        tw = unpack(x)
        r = []
        for d, Tm in zip(dQ, Tms):
            Tp = predict(tw, d)
            E = Tm[:3, :3] @ Tp[:3, :3].T  # small residual rotation: its skew part (sin of the angle along the axis)
            r.extend([0.5 * (E[2, 1] - E[1, 2]), 0.5 * (E[0, 2] - E[2, 0]), 0.5 * (E[1, 0] - E[0, 1])])
            r.extend(list((Tm[:3, 3] - Tp[:3, 3]) / sc))
        i = 0
        for jt in joint_types:  # gauges (consistent with the data): unit direction vectors; the point of a revolute axis closest to the origin
            if jt == "R":
                u, p = x[i:i + 3], x[i + 3:i + 6]
                r.append(np.linalg.norm(u) - 1.0)
                r.append(float(p @ (u / max(np.linalg.norm(u), 1e-12))) / sc)
                i += 6
            else:
                r.append(np.linalg.norm(x[i:i + 3]) - 1.0)
                i += 3
        return np.array(r)

    sol = least_squares(resid, x0, method="lm", xtol=1e-10, ftol=1e-10, gtol=1e-10, max_nfev=200)
    tw = unpack(sol.x)
    rot, tra = [], []
    for d, Tm in zip(dQ, Ts):
        Tp = predict(tw, d)
        rot.append(float(np.linalg.norm(_rotvec(np.asarray(Tm)[:3, :3] @ Tp[:3, :3].T))))
        tra.append(float(np.linalg.norm(np.asarray(Tm)[:3, 3] - Tp[:3, 3])))
    wp, wy = tw[pitch_index][0], tw[yaw_index][0]
    cn = np.cross(wp, wy)
    s = float(np.linalg.norm(cn))
    if s < 1e-6:
        return PoeTrialFit(False, n_steps=len(dQ), reason="the fitted wrist axes are (nearly) parallel")
    d_int = abs(float((tw[yaw_index][2] - tw[pitch_index][2]) @ (cn / s)))
    exc = []
    for idx in (pitch_index, yaw_index):
        own = [abs(d[idx]) for d in dQ if int(np.argmax(np.abs(d))) == idx]
        exc.append(float(min(own)) if own else 0.0)
    return PoeTrialFit(True, d_int_if=d_int, axes_angle_deg=math.degrees(math.asin(min(1.0, s))), rms_rot_rad=float(np.sqrt(np.mean(np.square(rot)))),
                       rms_trans_rel=float(np.sqrt(np.mean(np.square(tra)))) / d_int if d_int > 0 else float("inf"),
                       max_rot_rad=float(max(rot)), max_trans_rel=float(max(tra)) / d_int if d_int > 0 else float("inf"),
                       wrist_excitation_rad=exc, n_steps=len(dQ), twists=[list(w) + list(v) for w, v, _ in tw])


def geometry_anchor_poe_decision(fits: Sequence[PoeTrialFit], L_phys_m: float, u_rel: float, expected_unit_m: Optional[float], tol,
                                 rot_tol_rad: float = 0.02, trans_tol_rel: float = 0.02, min_excitation_rad: float = 0.05,
                                 axes_angle_deg: Optional[float] = 90.0, axes_angle_tol_deg: float = 2.0, alpha: float = 0.05) -> GeometryAnchorDecision:
    """Per-trial PoE fits -> lambda_hat interval -> eq. (6) decision, with the interval widened by u_rel as in
    geometry_anchor_decision.  Gates (any failure -> undetermined, reported): the fit explains every step (largest rotation
    residual <= rot_tol_rad, largest translation residual <= trans_tol_rel x d_int); each wrist joint moved by at least
    min_excitation_rad in its own steps; the fitted wrist axes meet at the model's angle within axes_angle_tol_deg."""
    fails: List[str] = []
    ds = []
    for i, f in enumerate(fits):
        if not f.ok:
            fails.append(f"trial {i}: {f.reason}")
            continue
        if f.max_rot_rad > rot_tol_rad:
            fails.append(f"trial {i}: the joint-space model leaves a rotation residual of {f.max_rot_rad:.4f} rad (> {rot_tol_rad:g})")
        if f.max_trans_rel > trans_tol_rel:
            fails.append(f"trial {i}: the joint-space model leaves a translation residual of {f.max_trans_rel:.4f} x d_int (> {trans_tol_rel:g})")
        if f.wrist_excitation_rad and min(f.wrist_excitation_rad) < min_excitation_rad:
            fails.append(f"trial {i}: a wrist joint moved only {min(f.wrist_excitation_rad):.4f} rad in its own steps (< {min_excitation_rad:g})")
        if axes_angle_deg is not None and abs(f.axes_angle_deg - axes_angle_deg) > axes_angle_tol_deg:
            fails.append(f"trial {i}: fitted wrist axes at {f.axes_angle_deg:.2f} deg, model {axes_angle_deg} deg")
        ds.append(f.d_int_if)
    dmean = float(np.mean(ds)) if ds else float("nan")
    lam = [L_phys_m / d for d in ds if d > 0]
    e = estimate(lam, alpha)
    lo_w, hi_w = e.ci_low * (1.0 - u_rel), e.ci_high * (1.0 + u_rel)
    assumptions = ["the published pose is rigidly attached to the distal link; the declared joint chain (order and joint types) is the implementation's serial chain",
                   "measured_js reports the joint values that produced measured_cp (the product-of-exponentials model is fitted to measured, not commanded, joint changes)",
                   f"the implementation's wrist geometry equals the instrument's: pitch-to-yaw length L = {L_phys_m*1e3:.2f} mm +- {u_rel*100:.2f} % (declared)",
                   "angles are unit-free; lambda_hat = L / d_int is the physical length of one interface unit",
                   f"per-trial estimates are iid with a Student-t interval of the mean (alpha = {alpha})"]
    if e.n < 3:
        fails.append(f"only {e.n} trial(s) produced a fit; at least 3 are needed")
    gates = not fails and e.n >= 3
    if expected_unit_m is None or not gates:
        return GeometryAnchorDecision(e.n, dmean, e.mean, [e.ci_low, e.ci_high], [lo_w, hi_w], u_rel, L_phys_m, expected_unit_m if expected_unit_m else float("nan"),
                                      float("nan"), [float("nan"), float("nan")], Outcome.UNDETERMINED.value, gates, fails, assumptions)
    widened = Estimate(e.n, e.mean, e.std, lo_w, hi_w, alpha)
    outcome, e_pred = scale_decision(widened, expected_unit_m, tol)
    return GeometryAnchorDecision(e.n, dmean, e.mean, [e.ci_low, e.ci_high], [lo_w, hi_w], u_rel, L_phys_m, float(expected_unit_m),
                                  e_pred.mean, [e_pred.ci_low, e_pred.ci_high], outcome.value, gates, fails, assumptions)
