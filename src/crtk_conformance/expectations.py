"""Client expectations — what the client under test declares it requires of each semantic binding.

Semantic rule of this release (0.1.1):

    `conformant` means that the discovered binding satisfies an explicitly declared client
    expectation within the stated tolerance.  No declared expectation -> no conformance verdict:
    the probe reports what it observed and returns `undetermined`.

The expectations are read from a small YAML file (``--expectations expectations.yaml``)::

    spatial:
      mode: expected_transform        # identity | expected_transform | discover_only (default)
      translation_m: [0.0, 0.0, 0.0]  # T_expected = (R, t): coordinates of a point in the frame of the
      quaternion_xyzw: [0, 0, 0, 1]   # unqualified topics map to the local/ (arm-base) frame as R p + t
    dimensional:
      mode: si                        # si | discover_only (default). 'si' needs --anchor-topic.
      expected_unit_m: 1.0            # metres per interface unit the client assumes (default 1.0)
    temporal:
      state_machine: required         # required | forbidden | any (default)
      stop_behaviour: hold            # hold | release | fault | any (default)
      rate: required                  # required | any (default): the client needs f >= v/epsilon at its rate

The default file (no file given) is discover-only in every class.

Convention of T_expected: the same as the probe's estimate T_hat = measured_cp * local/measured_cp^-1,
i.e. the dVRK `base_frame` (measured_cp = base_frame * local).  The manuscript's model transform is
T = base_frame^-1; the probe compares like with like, so the user states the base_frame-like transform
(what a `measured_cp`/`local/measured_cp` pair would show), which is what a dVRK configuration file
contains.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional

import numpy as np

SPATIAL_MODES = ("identity", "expected_transform", "discover_only")
DIMENSIONAL_MODES = ("si", "discover_only")
STATE_MACHINE = ("required", "forbidden", "any")
STOP_BEHAVIOUR = ("hold", "release", "fault", "any")
RATE = ("required", "any")


class ExpectationError(ValueError):
    pass


@dataclass
class SpatialExpectation:
    mode: str = "discover_only"
    translation_m: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion_xyzw: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])

    @property
    def declared(self) -> bool:
        return self.mode in ("identity", "expected_transform")

    def matrix(self) -> Optional[np.ndarray]:
        from . import geometry as G

        if self.mode == "identity":
            return np.eye(4)
        if self.mode == "expected_transform":
            x, y, z, w = self.quaternion_xyzw
            n = math.sqrt(x * x + y * y + z * z + w * w)
            if n == 0:
                raise ExpectationError("spatial.quaternion_xyzw must be non-zero")
            R = G.quat_to_rot(x / n, y / n, z / n, w / n)
            return G.make_pose(R, self.translation_m)
        return None


@dataclass
class DimensionalExpectation:
    mode: str = "discover_only"
    expected_unit_m: float = 1.0

    @property
    def declared(self) -> bool:
        return self.mode == "si"


@dataclass
class TemporalExpectation:
    state_machine: str = "any"
    stop_behaviour: str = "any"
    rate: str = "any"

    @property
    def declared(self) -> bool:
        return self.state_machine != "any" or self.stop_behaviour != "any" or self.rate != "any"


@dataclass
class Expectations:
    spatial: SpatialExpectation = field(default_factory=SpatialExpectation)
    dimensional: DimensionalExpectation = field(default_factory=DimensionalExpectation)
    temporal: TemporalExpectation = field(default_factory=TemporalExpectation)
    source: str = "default (discover-only in every class)"

    def to_dict(self) -> Dict[str, Any]:
        d = asdict(self)
        d["spatial"]["declared"] = self.spatial.declared
        d["dimensional"]["declared"] = self.dimensional.declared
        d["temporal"]["declared"] = self.temporal.declared
        return d

    @staticmethod
    def from_dict(d: Dict[str, Any], source: str = "dict") -> "Expectations":
        d = d or {}
        sp = dict(d.get("spatial") or {})
        di = dict(d.get("dimensional") or {})
        te = dict(d.get("temporal") or {})
        e = Expectations(source=source)
        if sp:
            mode = sp.get("mode", "discover_only")
            if mode not in SPATIAL_MODES:
                raise ExpectationError(f"spatial.mode must be one of {SPATIAL_MODES}, got {mode!r}")
            e.spatial = SpatialExpectation(mode=mode,
                                           translation_m=[float(v) for v in sp.get("translation_m", [0.0, 0.0, 0.0])],
                                           quaternion_xyzw=[float(v) for v in sp.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])])
            if len(e.spatial.translation_m) != 3 or len(e.spatial.quaternion_xyzw) != 4:
                raise ExpectationError("spatial.translation_m needs 3 values and spatial.quaternion_xyzw 4")
            if mode == "expected_transform":
                e.spatial.matrix()  # validates
        if di:
            mode = di.get("mode", "discover_only")
            if mode not in DIMENSIONAL_MODES:
                raise ExpectationError(f"dimensional.mode must be one of {DIMENSIONAL_MODES}, got {mode!r}")
            unit = float(di.get("expected_unit_m", 1.0))
            if unit <= 0:
                raise ExpectationError("dimensional.expected_unit_m must be positive")
            e.dimensional = DimensionalExpectation(mode=mode, expected_unit_m=unit)
        if te:
            sm = te.get("state_machine", "any")
            sb = te.get("stop_behaviour", "any")
            ra = te.get("rate", "any")
            for v, allowed, name in ((sm, STATE_MACHINE, "state_machine"), (sb, STOP_BEHAVIOUR, "stop_behaviour"), (ra, RATE, "rate")):
                if v not in allowed:
                    raise ExpectationError(f"temporal.{name} must be one of {allowed}, got {v!r}")
            e.temporal = TemporalExpectation(state_machine=sm, stop_behaviour=sb, rate=ra)
        return e

    @staticmethod
    def load(path: Optional[str]) -> "Expectations":
        if not path:
            return Expectations()
        import yaml

        with open(path) as f:
            d = yaml.safe_load(f) or {}
        if not isinstance(d, dict):
            raise ExpectationError("expectations file must contain a mapping")
        return Expectations.from_dict(d, source=path)


def combine(sub_verdicts: Dict[str, Optional[str]]) -> str:
    """Combine per-expectation sub-verdicts into one outcome.

    Each value is 'satisfied', 'violated', 'undetermined' or None (no expectation declared for it).
    Rule: any 'violated' -> divergent; otherwise, if at least one expectation was declared and every
    declared one is 'satisfied' -> conformant; otherwise (nothing declared, or something undetermined)
    -> undetermined.
    """
    declared = [v for v in sub_verdicts.values() if v is not None]
    if any(v == "violated" for v in declared):
        return "divergent"
    if declared and all(v == "satisfied" for v in declared):
        return "conformant"
    return "undetermined"
