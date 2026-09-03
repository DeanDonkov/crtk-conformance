"""Decision thresholds derived from a user-stated task-space tolerance.

The suite has no built-in "1 mm / 5 % / 95 %" constants. The user states a task-space tolerance
epsilon (metres), a workspace radius r_ws (metres, the largest ||p_c|| the client will command
in the interface frame) and a nominal end-effector speed v (m/s). The decision thresholds follow
from the error model of the paper (Section 5):

  spatial      predicted worst-case absolute-command error of the residual binding (observed vs expected)
                                                              e_hat = ||t_E|| + 2 sin(theta_E/2) r_ws     (eq. 3)
  dimensional  predicted error at the workspace edge          e_hat = |1 - s_hat/u| r_ws                  (eq. 6)
  temporal     required command rate                          f_req = v / epsilon                         (eq. 9)
               required liveness period                       1/f_client + J_max < tau_w_hat             (eq. 7)

A quantity is CONFORMANT if the upper confidence limit of its predicted error is below epsilon,
DIVERGENT if the lower confidence limit is above epsilon, and UNDETERMINED otherwise (including
when the estimate could not be formed, and for interval ends within a 1e-9 relative guard band of
epsilon). See probes/base.py for the outcome type.

These are the *task thresholds*: every one is derived from the user's tolerance inputs. They are distinct
from the *implementation constants* of the probes (settle tolerance, pairing window, response timeout,
match tolerance, stream duration, ...), which affect whether an estimate can be formed but never move the
decision boundary; they are listed in the README ("Implementation constants") and exposed as CLI options.
Equation numbers refer to the manuscript: (3) spatial bound, (6) unit-scale error, (7) bounded-jitter rate,
(9) zero-order-hold lag.

0.1.1: a decision is only taken against a *declared* client expectation (see expectations.py).
"""
from __future__ import annotations

from dataclasses import dataclass, asdict


DEFAULT_TRIALS = 10  # see README: at n = 10 the 95 % CI half-width is ~0.72 sigma; divergences an order of
# magnitude above measurement noise are resolved, and the runtime stays at a few seconds per probe.


@dataclass
class Tolerance:
    epsilon_m: float = 0.001  # task-space tolerance (metres). Default 1 mm is a *placeholder the user must set*.
    workspace_radius_m: float = 0.10
    speed_m_s: float = 0.05
    client_rate_hz: float = 100.0  # the rate at which the client under test will publish servo commands
    jitter_max_s: float = 0.005  # J_max the client can guarantee (eq. 4)

    def to_dict(self):
        return asdict(self)

    def required_rate_hz(self) -> float:
        return self.speed_m_s / self.epsilon_m

    def spatial_error(self, t_norm: float, theta_rad: float) -> float:
        import math

        return t_norm + 2.0 * math.sin(theta_rad / 2.0) * self.workspace_radius_m

    def dimensional_error(self, s: float) -> float:
        return abs(1.0 - s) * self.workspace_radius_m

    def zoh_error(self, rate_hz: float) -> float:
        return self.speed_m_s / rate_hz if rate_hz > 0 else float("inf")

    def liveness_ok(self, tau_w_s: float) -> bool:
        """Eq. (7) of the manuscript: the client's period plus its jitter bound must stay below the timeout."""
        return (1.0 / self.client_rate_hz + self.jitter_max_s) < tau_w_s
