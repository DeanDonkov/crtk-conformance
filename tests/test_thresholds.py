import math
from crtk_conformance.thresholds import Tolerance
from crtk_conformance.probes.base import decide, Outcome


def test_decide_three_valued():
    assert decide(0.0001, 0.0005, 0.001) == Outcome.CONFORMANT
    assert decide(0.002, 0.003, 0.001) == Outcome.DIVERGENT
    assert decide(0.0005, 0.002, 0.001) == Outcome.UNDETERMINED
    assert decide(float("nan"), 0.1, 0.001) == Outcome.UNDETERMINED


def test_tolerance_derivations():
    t = Tolerance(epsilon_m=0.0005, workspace_radius_m=0.1, speed_m_s=0.05, client_rate_hz=100, jitter_max_s=0.005)
    assert abs(t.required_rate_hz() - 100) < 1e-9
    assert abs(t.spatial_error(0.2, math.radians(150)) - (0.2 + 2 * math.sin(math.radians(75)) * 0.1)) < 1e-12
    assert abs(t.dimensional_error(0.1) - 0.09) < 1e-12
    assert t.liveness_ok(0.5) and not t.liveness_ok(0.014)
