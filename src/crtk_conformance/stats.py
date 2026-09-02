"""Repeated-trial statistics used by every probe.

Every estimate the suite reports is the mean of n repeated trials with its sample standard
deviation and a two-sided (1 - alpha) confidence interval from the Student t distribution.
n is a user parameter (default 10, see thresholds.DEFAULT_TRIALS and the README for why).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Sequence

import numpy as np
from scipy import stats as _st


@dataclass
class Estimate:
    n: int
    mean: float
    std: float
    ci_low: float
    ci_high: float
    alpha: float = 0.05

    def to_dict(self):
        return asdict(self)

    @property
    def half_width(self) -> float:
        return (self.ci_high - self.ci_low) / 2.0


def estimate(samples: Sequence[float], alpha: float = 0.05) -> Estimate:
    x = np.asarray([float(s) for s in samples], dtype=float)
    x = x[np.isfinite(x)]
    n = int(x.size)
    if n == 0:
        return Estimate(0, math.nan, math.nan, math.nan, math.nan, alpha)
    m = float(x.mean())
    if n == 1:
        return Estimate(1, m, math.nan, -math.inf, math.inf, alpha)
    sd = float(x.std(ddof=1))
    tcrit = float(_st.t.ppf(1 - alpha / 2.0, n - 1))
    hw = tcrit * sd / math.sqrt(n)
    return Estimate(n, m, sd, m - hw, m + hw, alpha)


def rate_estimate(count: int, total: int, alpha: float = 0.05) -> dict:
    """Wilson score interval for a binomial proportion."""
    if total == 0:
        return {"count": 0, "total": 0, "p": math.nan, "ci_low": math.nan, "ci_high": math.nan}
    z = float(_st.norm.ppf(1 - alpha / 2.0))
    p = count / total
    denom = 1 + z * z / total
    centre = (p + z * z / (2 * total)) / denom
    hw = z * math.sqrt(p * (1 - p) / total + z * z / (4 * total * total)) / denom
    return {"count": count, "total": total, "p": p, "ci_low": max(0.0, centre - hw), "ci_high": min(1.0, centre + hw)}
