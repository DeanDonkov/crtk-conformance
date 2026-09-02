import math
import numpy as np
from scipy import stats as st
from crtk_conformance.stats import estimate, rate_estimate


def test_estimate_matches_scipy():
    x = np.random.default_rng(0).normal(1.0, 0.1, 10)
    e = estimate(x)
    lo, hi = st.t.interval(0.95, len(x) - 1, loc=x.mean(), scale=st.sem(x))
    assert abs(e.ci_low - lo) < 1e-12 and abs(e.ci_high - hi) < 1e-12
    assert e.n == 10 and abs(e.mean - x.mean()) < 1e-12


def test_estimate_degenerate():
    assert estimate([]).n == 0 and math.isnan(estimate([]).mean)
    e = estimate([1.0]); assert e.n == 1 and math.isinf(e.ci_high)


def test_wilson():
    r = rate_estimate(9, 10)
    assert 0.55 < r["ci_low"] < 0.7 and r["ci_high"] > 0.98 and abs(r["p"] - 0.9) < 1e-12
    assert math.isnan(rate_estimate(0, 0)["p"])
