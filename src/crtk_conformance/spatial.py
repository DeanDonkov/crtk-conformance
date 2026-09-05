"""Spatial decision statistics (0.1.2).

Two things changed after the adversarial review of RC3 (rc4/RC3_ADVERSARIAL_REVIEW.md, findings 1 and 4):

1. The decision quantity is the EXACT maximum positional error of a residual rigid transform E = (R, t) over the
   command ball ||p|| <= r_ws, not the upper bound of eq. (3).  With theta the rotation angle of R, k its axis,
   t_par = (t . k) k and t_perp = t - t_par,

       e_max(E) = sqrt( ||t_par||^2 + ( ||t_perp|| + 2 sin(theta/2) r_ws )^2 ),                     (eq. 3')

   attained at the p that puts (R - I) p (which lies in the plane orthogonal to k, with norm 2 sin(theta/2)||p||)
   parallel to t_perp.  Eq. (3) is an upper bound of this and was used as if exceeding it proved a violation;
   the reviewer's counterexample (t_par = 0.7 mm, theta = 0.3 deg, r_ws = 0.1 m, epsilon = 1 mm: e_max = 0.874 mm,
   bound 1.224 mm) is a regression test.

2. The uncertainty of e_max is propagated from a joint confidence region for the transform parameters instead
   of re-centring a Student-t half-width of per-trial norms (which had ~45 % coverage at zero residual).  The
   per-trial residuals give n samples of the translation t_i and of the rotation vector rho_i (deviations from
   the mean rotation).  Each parameter vector gets a Hotelling T^2 confidence ellipsoid at level 1 - alpha/2
   (Bonferroni over the two), whose largest Euclidean radius is

       delta = sqrt( p (n - 1) / (n (n - p)) * F_{p, n-p}(1 - alpha/2) * lambda_max(S) ),     p = 3,

   and e_max is 1-Lipschitz in t and r_ws-Lipschitz in rho (the exponential map is 1-Lipschitz from the rotation
   vector to the bi-invariant distance on SO(3), and ||R p - R' p|| <= d(R, R') ||p||), so

       e_max(true) in [ max(0, e_max(centre) - delta_t - r_ws delta_rho),  e_max(centre) + delta_t + r_ws delta_rho ]

   with probability >= 1 - alpha under iid Gaussian trial errors.  The interval is conservative by construction;
   its empirical coverage is checked in tests/test_spatial.py (>= 0.95 at zero residual and near the boundary).
   n <= p gives an infinite interval (undetermined): the region is not defined.

The verdict is positional: an orientation tolerance, if the client declares one, is checked separately on the
rotation angle of the residual (with its own interval); without one, orientation enters the verdict only
through its positional effect over the workspace.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import List, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as _Rot
from scipy.stats import f as _f

from . import geometry as G


def rotation_axis(R: np.ndarray) -> Optional[np.ndarray]:
    """Unit rotation axis of R, or None when the angle is numerically zero."""
    theta = G.rotation_angle(R)
    if theta < 1e-12:
        return None
    rv = _Rot.from_matrix(R).as_rotvec()
    n = np.linalg.norm(rv)
    return rv / n if n > 0 else None


def exact_max_error(R: np.ndarray, t: np.ndarray, r_ws: float) -> float:
    """Eq. (3'): the maximum of ||(R - I) p + t|| over ||p|| <= r_ws (exact, attained)."""
    t = np.asarray(t, dtype=float)
    theta = G.rotation_angle(R)
    k = rotation_axis(R)
    s = 2.0 * math.sin(theta / 2.0) * r_ws
    if k is None:
        return float(np.linalg.norm(t))
    t_par = float(np.dot(t, k))
    t_perp = float(np.linalg.norm(t - t_par * k))
    return float(math.sqrt(t_par * t_par + (t_perp + s) ** 2))


def exact_min_error(R: np.ndarray, t: np.ndarray, r_ws: float) -> float:
    """The minimum of ||(R - I) p + t|| over ||p|| <= r_ws: max(||t_par||, ... ) -- the perpendicular part can be
    cancelled by (R - I) p only up to 2 sin(theta/2) r_ws.  Reported for information (the best case in the ball)."""
    t = np.asarray(t, dtype=float)
    theta = G.rotation_angle(R)
    k = rotation_axis(R)
    s = 2.0 * math.sin(theta / 2.0) * r_ws
    if k is None:
        return float(np.linalg.norm(t))
    t_par = float(np.dot(t, k))
    t_perp = float(np.linalg.norm(t - t_par * k))
    return float(math.sqrt(t_par * t_par + max(0.0, t_perp - s) ** 2))


def hotelling_radius(samples: np.ndarray, alpha: float) -> float:
    """Largest Euclidean distance from the sample mean to the boundary of the Hotelling T^2 confidence ellipsoid
    for the population mean at level 1 - alpha (n x p samples; inf when n <= p)."""
    X = np.asarray(samples, dtype=float)
    n, p = X.shape
    if n <= p:
        return float("inf")
    S = np.cov(X, rowvar=False, ddof=1)
    S = np.atleast_2d(S)
    lam = float(max(np.linalg.eigvalsh(S).max(), 0.0))
    c = p * (n - 1) / (n * (n - p)) * float(_f.ppf(1.0 - alpha, p, n - p))
    return float(math.sqrt(c * lam))


@dataclass
class SpatialDecision:
    n: int
    r_ws_m: float
    residual_translation_m: List[float]
    residual_rotation_deg: float
    e_max_m: float  # exact maximum positional error of the mean residual over the ball
    e_min_m: float  # exact minimum (best case in the ball), information only
    ci_low_m: float
    ci_high_m: float
    delta_t_m: float  # Hotelling radius of the translation region (level 1 - alpha/2)
    delta_rho_rad: float  # Hotelling radius of the rotation-vector region (level 1 - alpha/2)
    alpha: float
    bound_eq3_m: float  # the old eq. (3) upper bound, for comparison only

    def to_dict(self):
        return asdict(self)


def spatial_decision(residuals: Sequence[np.ndarray], r_ws: float, alpha: float = 0.05) -> SpatialDecision:
    """residuals: per-trial 4x4 residual transforms E_i = T_i * T_expected^-1 (n >= 1)."""
    Es = [np.asarray(E, dtype=float) for E in residuals]
    n = len(Es)
    E_bar = G.average_pose(Es)
    R_bar, t_bar = E_bar[:3, :3], E_bar[:3, 3]
    ts = np.array([E[:3, 3] for E in Es])
    rhos = np.array([_Rot.from_matrix(E[:3, :3] @ R_bar.T).as_rotvec() for E in Es])
    # the regions are centred on the sample means; the centre transform (R_bar, t_bar) may differ from them by a
    # small amount (the chordal rotation mean is not the rotation-vector mean), which is added to the radii
    d_t = hotelling_radius(ts, alpha / 2.0) + float(np.linalg.norm(ts.mean(axis=0) - t_bar))
    d_rho = hotelling_radius(rhos, alpha / 2.0) + float(np.linalg.norm(rhos.mean(axis=0)))
    e_c = exact_max_error(R_bar, t_bar, r_ws)
    slack = d_t + r_ws * d_rho
    lo = max(0.0, e_c - slack) if math.isfinite(slack) else 0.0
    hi = e_c + slack if math.isfinite(slack) else float("inf")
    return SpatialDecision(
        n=n, r_ws_m=r_ws, residual_translation_m=[float(v) for v in t_bar], residual_rotation_deg=math.degrees(G.rotation_angle(R_bar)),
        e_max_m=e_c, e_min_m=exact_min_error(R_bar, t_bar, r_ws), ci_low_m=lo, ci_high_m=hi, delta_t_m=d_t, delta_rho_rad=d_rho, alpha=alpha,
        bound_eq3_m=float(np.linalg.norm(t_bar) + 2.0 * math.sin(G.rotation_angle(R_bar) / 2.0) * r_ws),
    )


def rotation_angle_interval(residuals: Sequence[np.ndarray], alpha: float = 0.05):
    """Interval for the rotation angle of the mean residual (radians): angle(R_bar) -/+ delta_rho, since the angle is
    1-Lipschitz in the rotation vector."""
    Es = [np.asarray(E, dtype=float) for E in residuals]
    E_bar = G.average_pose(Es)
    R_bar = E_bar[:3, :3]
    rhos = np.array([_Rot.from_matrix(E[:3, :3] @ R_bar.T).as_rotvec() for E in Es])
    d_rho = hotelling_radius(rhos, alpha)
    th = G.rotation_angle(R_bar)
    return th, max(0.0, th - d_rho), th + d_rho
