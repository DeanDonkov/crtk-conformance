"""Spatial decision statistics (0.1.2; distributional scope stated in 0.1.3).

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

   with probability >= 1 - alpha under the model stated below.  The interval is conservative by construction
   under that model; its empirical coverage is checked in tests/test_spatial.py (>= 0.95 at zero residual and
   near the boundary, Gaussian trial errors).  n <= p gives an infinite interval (undetermined): the region is
   not defined.

Distributional model (0.1.3, RC4 adversarial review finding 7).  The guarantee above is NOT distribution-free and
independence of the trial errors alone does not give it.  It rests on three separate things, reported with every
decision (SpatialDecision.assumptions):

   (a) the parameter-region theorem: the per-trial translation errors t_i - t are iid multivariate normal, and so
       are the rotation-vector deviations rho_i; Hotelling's T^2 with the F distribution is exact for that model
       (Anderson, An Introduction to Multivariate Statistical Analysis, ch. 5) and for no wider class in finite
       samples.  Heavy-tailed or mixture errors break it: the reviewer's stress case (n = 10, x-error a mixture of
       -20 um w.p. 0.95 and +380 um w.p. 0.05, plus 1 um Gaussian noise) has 42.8 % coverage at zero residual and
       declares divergence at a 1 um tolerance 57.2 % of the time (tests/adversarial/
       rc4_reviewer_non_gaussian_scope_check.json; reproduced as a documented limitation in tests/test_spatial.py).
       A client whose trial errors are not approximately normal must not read the interval as a 95 % region.
   (b) the rotation approximation: the rho_i are rotation vectors of R_i R_bar^T, i.e. deviations from the
       chordal mean rotation mapped to R^3 by the logarithm; treating them as iid Euclidean-normal samples is a
       small-angle approximation (the logarithm is close to linear for angles << 1 rad; all validated
       configurations have per-trial rotation deviations below 1 deg).  The Lipschitz step from the rotation-
       vector region to e_max is exact, the normality of the rho_i is not a theorem.
   (c) the measured coverage: the 240/240 and 120/120 containment counts of the v0.1.2 campaign are empirical
       results for their four specified Gaussian-noise configurations, not a distribution-free guarantee.

The verdict is positional: an orientation tolerance, if the client declares one, is checked separately on the
rotation angle of the residual (with its own interval); without one, orientation enters the verdict only
through its positional effect over the workspace.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict, field
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
    max_rotation_deviation_deg: float = float("nan")  # largest per-trial rotation deviation from the mean (small-angle check)
    model: str = "hotelling-t2-mvn/small-angle-rotation/lipschitz"
    assumptions: List[str] = field(default_factory=list)

    def to_dict(self):
        return asdict(self)


def spatial_assumptions(n: int, alpha: float, max_dev_deg: float) -> List[str]:
    """The distributional assumptions under which [ci_low, ci_high] is a >= 1 - alpha region (module docstring)."""
    return [
        f"the per-trial translation errors are iid multivariate normal (Hotelling T^2 / F region, n = {n}, level 1 - {alpha}); "
        "the region is not distribution-free: heavy-tailed or mixture errors reduce its coverage (RC4 review stress case: 42.8 %)",
        f"the per-trial rotation deviations from the mean rotation, mapped to rotation vectors, are iid multivariate normal; "
        f"this is a small-angle approximation (largest deviation in this run {max_dev_deg:.3f} deg)",
        "e_max is 1-Lipschitz in the translation and r_ws-Lipschitz in the rotation vector (exact); the interval is conservative under (a) and (b)",
        "the coverage reported for the validation campaign is empirical for its Gaussian-noise configurations, not a guarantee for other error distributions",
    ]


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
    max_dev = math.degrees(float(np.linalg.norm(rhos, axis=1).max())) if n else float("nan")
    return SpatialDecision(
        n=n, r_ws_m=r_ws, residual_translation_m=[float(v) for v in t_bar], residual_rotation_deg=math.degrees(G.rotation_angle(R_bar)),
        e_max_m=e_c, e_min_m=exact_min_error(R_bar, t_bar, r_ws), ci_low_m=lo, ci_high_m=hi, delta_t_m=d_t, delta_rho_rad=d_rho, alpha=alpha,
        bound_eq3_m=float(np.linalg.norm(t_bar) + 2.0 * math.sin(G.rotation_angle(R_bar) / 2.0) * r_ws),
        max_rotation_deviation_deg=max_dev, assumptions=spatial_assumptions(n, alpha, max_dev),
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


# ------------------------------------------------------------------ 0.1.8: the frame probe's correlation guard
# RC13 external review, point 1: the Hotelling region assumes independent trials, and the probe took its ten trials back
# to back (about 0.5 s at 100 Hz).  Strongly correlated trial errors (AR(1), rho = 0.9 between trials) gave 11 % false
# divergence at the boundary offline.  0.1.8 therefore (1) measures the pose-error correlation of the paired residual on
# a resting window before the trials, (2) spaces the trials so that their means are nearly independent, and (3)
# withholds the verdict when the correlation time is not resolved by the window, when the required spacing exceeds the
# sampling budget, or when the implied effective number of independent trials is below the minimum.

DETERMINISTIC_FLOOR = 1e-12  # a residual component with a smaller standard deviation carries no noise (metres or rad)


def integrated_autocorr_time(x: Sequence[float], c: float = 5.0):
    """Sokal's automatic-window estimate of the integrated autocorrelation time tau_int = 1 + 2 sum_{k>=1} rho(k), in
    samples, with the window M the smallest lag with M >= c tau_int(M).  Returns (tau_int, acf[0..M]).  A series without
    variance returns (1.0, [1.0])."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 4:
        return 1.0, np.array([1.0])
    x = x - x.mean()
    v = float(x @ x) / n
    if not (v > 0 and math.isfinite(v)):
        return 1.0, np.array([1.0])
    f = np.fft.rfft(x, 2 * n)
    acf = np.fft.irfft(f * np.conj(f))[:n] / (v * n)
    tau = 1.0
    M = n - 1
    for m in range(1, n):
        tau = 1.0 + 2.0 * float(np.sum(acf[1:m + 1]))
        if m >= c * tau:
            M = m
            break
    return max(tau, 1.0), acf[:M + 1]


def block_mean_correlation(acf: Sequence[float], m: int, spacing: int) -> float:
    """Correlation between the means of two blocks of m consecutive samples whose starts are `spacing` samples apart,
    from an autocorrelation function (taken as zero beyond its window)."""
    acf = np.asarray(acf, dtype=float)

    def rho(k):
        k = abs(int(k))
        return float(acf[k]) if k < len(acf) else 0.0
    num = sum(rho(spacing + j - i) for i in range(m) for j in range(m))
    den = sum(rho(j - i) for i in range(m) for j in range(m))
    return num / den if den > 0 else 0.0


def effective_trials(n: int, r: float) -> float:
    """Effective number of independent trials for a mean of n equally correlated neighbours with lag-1 correlation r
    (AR(1)-like decay); r <= 0 gives n."""
    if not math.isfinite(r) or r <= 0:
        return float(n)
    r = min(r, 0.999999)
    return float(n) * (1.0 - r) / (1.0 + r)


@dataclass
class CorrelationPlan:
    samples: int
    tau_int_samples: float
    component_tau: List[float]
    deterministic: bool
    resolved: bool  # samples >= window_factor x tau_int
    spacing_samples: int
    between_trial_correlation: float
    effective_trials: float
    reason: str = ""

    def to_dict(self):
        return asdict(self)


def residual_series(Ts: Sequence[np.ndarray]) -> np.ndarray:
    """(n, 6) series of a paired-residual stream: translation (3) and the rotation vector of R_t R_bar^T (3)."""
    from . import geometry as G
    Rbar = G.average_pose([np.asarray(T, dtype=float) for T in Ts])[:3, :3]
    out = []
    for T in Ts:
        T = np.asarray(T, dtype=float)
        out.append(np.concatenate([T[:3, 3], _Rot.from_matrix(T[:3, :3] @ Rbar.T).as_rotvec()]))
    return np.array(out)


def correlation_plan(series: np.ndarray, m: int, n_trials: int, window_factor: float = 20.0, spacing_factor: float = 2.0) -> CorrelationPlan:
    """Plan the trial spacing from a resting residual series (n, k): tau_int is the largest over the components with
    variance; the spacing between trial starts is max(m, ceil(spacing_factor x tau_int)) samples."""
    series = np.asarray(series, dtype=float)
    n = len(series)
    taus, acfs = [], []
    for j in range(series.shape[1]):
        if float(np.std(series[:, j])) <= DETERMINISTIC_FLOOR:
            continue
        t, a = integrated_autocorr_time(series[:, j])
        taus.append(t); acfs.append(a)
    if not taus:
        return CorrelationPlan(n, 1.0, [], True, True, int(m), 0.0, float(n_trials), "the paired residual carries no noise (deterministic relation)")
    k = int(np.argmax(taus))
    tau = float(taus[k])
    spacing = max(int(m), int(math.ceil(spacing_factor * tau)))
    r = block_mean_correlation(acfs[k], int(m), spacing)
    ne = effective_trials(n_trials, r)
    resolved = n >= window_factor * tau
    return CorrelationPlan(n, tau, [float(t) for t in taus], False, bool(resolved), spacing, float(r), float(ne),
                           "" if resolved else f"the resting window ({n} samples) is shorter than {window_factor:g} x tau_int ({tau:.1f} samples)")


def correlation_guard(plan: CorrelationPlan, n_trials: int, rate_hz: float, max_sampling_s: float = 60.0, neff_min: float = 4.0):
    """(ok, reason): the verdict is withheld when the correlation time is unresolved, when the spaced trials would exceed
    the sampling budget, or when the effective number of independent trials is below neff_min."""
    if not plan.resolved:
        return False, plan.reason
    need_s = n_trials * plan.spacing_samples / max(rate_hz, 1e-9)
    if need_s > max_sampling_s:
        return False, f"trials spaced by {plan.spacing_samples} samples need {need_s:.1f} s (> {max_sampling_s:g} s budget)"
    if plan.effective_trials < neff_min:
        return False, f"effective number of independent trials {plan.effective_trials:.2f} < {neff_min:g}"
    return True, ""
