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


def decide(pred_low: float, pred_high: float, epsilon: float) -> Outcome:
    """Three-valued decision on a predicted-error confidence interval against a tolerance."""
    import math

    if any(map(lambda v: v is None or (isinstance(v, float) and math.isnan(v)), (pred_low, pred_high))):
        return Outcome.UNDETERMINED
    if pred_high <= epsilon:
        return Outcome.CONFORMANT
    if pred_low > epsilon:
        return Outcome.DIVERGENT
    return Outcome.UNDETERMINED
