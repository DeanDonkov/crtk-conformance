"""Common result types for probes."""
from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional


class Outcome(str, enum.Enum):
    CONFORMANT = "conformant"
    DIVERGENT = "divergent"
    UNDETERMINED = "undetermined"


@dataclass
class ProbeResult:
    probe: str
    binding_class: str  # spatial | dimensional | temporal
    outcome: Outcome
    estimates: Dict[str, Any] = field(default_factory=dict)  # name -> Estimate.to_dict() or scalar
    predicted_error_m: Optional[float] = None
    predicted_error_ci: Optional[List[float]] = None
    decision_basis: str = ""
    observations: Dict[str, Any] = field(default_factory=dict)  # raw per-trial data
    notes: List[str] = field(default_factory=list)
    started_at: float = field(default_factory=time.time)
    duration_s: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["outcome"] = self.outcome.value
        return d


BOUNDARY_REL_GUARD = 1e-9  # relative guard band around epsilon: interval ends within it are boundary ties


def decide(pred_low: float, pred_high: float, epsilon: float) -> Outcome:
    """Three-valued decision on a predicted-error confidence interval against a tolerance.

    CONFORMANT if the upper confidence limit is <= epsilon, DIVERGENT if the lower limit is > epsilon,
    UNDETERMINED otherwise.  Interval ends within a relative guard band of 1e-9 around epsilon are treated
    as boundary ties and yield UNDETERMINED, so that floating-point representation (e.g. 4.999999999999999 mm
    against 5 mm) cannot decide a case either way (0.1.1; Reviewer #2 minor 5).
    """
    import math

    if any(map(lambda v: v is None or (isinstance(v, float) and math.isnan(v)), (pred_low, pred_high))):
        return Outcome.UNDETERMINED
    g = BOUNDARY_REL_GUARD * max(abs(epsilon), 1e-12)
    if abs(pred_high - epsilon) <= g or abs(pred_low - epsilon) <= g:
        return Outcome.UNDETERMINED
    if pred_high < epsilon:
        return Outcome.CONFORMANT
    if pred_low > epsilon:
        return Outcome.DIVERGENT
    return Outcome.UNDETERMINED
