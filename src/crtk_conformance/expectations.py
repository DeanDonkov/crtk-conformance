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
      orientation_tolerance_deg: 2.0  # optional (0.1.2): checked on the residual rotation angle; otherwise the
                                      # spatial verdict is POSITIONAL only (orientation enters through eq. 3')
    dimensional:
      mode: si                        # si | discover_only (default). 'si' needs --anchor-topic.
      expected_unit_m: 1.0            # metres per interface unit the client assumes (default 1.0).  0.1.2: the
                                      # spatial probe also uses it to convert the quotient translation to metres;
                                      # a spatial verdict therefore ASSUMES this unit and says so in the report
    temporal:
      state_machine: required         # required | forbidden | any (default)
      stop_behaviour: hold            # hold | fault | drift | release | any (default), see below
      horizon_s: 2.0                  # 0.1.2: the silence (s) up to which the stop expectation is claimed; the
                                      # probe tests silences up to its --gap-max-s and cannot decide beyond it
      rate: required                  # required | any (default): the client needs f >= v/epsilon at its rate;
                                      # 0.1.2: decided on the accepted-command channel (setpoint_cp) by the longest
                                      # stale interval; undetermined when the interface has no setpoint_cp

Stop-behaviour semantics (0.1.2, RC3 adversarial review finding 6).  What the probe can OBSERVE through the pose
and state topics is: `held` (the pose stays within the hold tolerance during the silence and a later command
is acted on), `drifted` (the pose leaves the hold tolerance during the silence), `rejected` (a later command
is not acted on), `faulted` (rejected and the operating state shows FAULT/DISABLED).  The expectations map to
these observations: `hold` <-> held through the horizon; `fault` <-> faulted or rejected within the horizon;
`drift` <-> drifted within the horizon.  `release` (actuation released) is a physical mode that the pose
alone does not identify -- a released, balanced or friction-held mechanism can stay still, and a held one
can drift -- so a `release` expectation is always `undetermined` through this interface; the report states
the drift observation beside it.  A silence longer than the tested horizon is never decided.

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
DIMENSIONAL_MODES = ("si", "geometry_anchor", "discover_only")  # 0.1.6: geometry_anchor (instrument-geometry unit anchor, probes/geometry.py)
STATE_MACHINE = ("required", "forbidden", "any")
STOP_BEHAVIOUR = ("hold", "fault", "drift", "release", "any")
RATE = ("required", "any")


class ExpectationError(ValueError):
    pass


@dataclass
class SpatialExpectation:
    mode: str = "discover_only"
    translation_m: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion_xyzw: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    orientation_tolerance_deg: Optional[float] = None  # 0.1.2: optional; without it the verdict is positional only

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
    # 0.1.6, mode geometry_anchor: the instrument dimension that grounds the unit (probes/geometry.py, dimensional.py)
    L_m: Optional[float] = None  # physical pitch-to-yaw length of the instrument (m), with its source recorded in L_source
    u_rel: Optional[float] = None  # declared relative uncertainty of L_m (e.g. 0.015); widens the interval multiplicatively
    L_source: str = ""
    pitch_joint: str = "wrist_pitch"  # measured_js name (substring match) or an integer index as a string
    yaw_joint: str = "wrist_yaw"
    delta_q_rad: float = 0.5
    axes_angle_deg: Optional[float] = 90.0  # angle between the two wrist axes in the instrument model (gate)
    reference_joints: Optional[List[float]] = None  # joint configuration the steps start from (default: the current one)

    @property
    def declared(self) -> bool:
        return self.mode in ("si", "geometry_anchor")


@dataclass
class TemporalExpectation:
    state_machine: str = "any"
    stop_behaviour: str = "any"
    rate: str = "any"
    horizon_s: Optional[float] = None  # 0.1.2: the silence up to which the stop expectation is claimed; None = the probe's tested maximum gap

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
            ot = sp.get("orientation_tolerance_deg")
            if ot is not None and float(ot) <= 0:
                raise ExpectationError("spatial.orientation_tolerance_deg must be positive")
            e.spatial = SpatialExpectation(mode=mode,
                                           translation_m=[float(v) for v in sp.get("translation_m", [0.0, 0.0, 0.0])],
                                           quaternion_xyzw=[float(v) for v in sp.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])],
                                           orientation_tolerance_deg=(float(ot) if ot is not None else None))
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
            if mode == "geometry_anchor":
                L = di.get("L_m")
                u = di.get("u_rel")
                if L is None or float(L) <= 0:
                    raise ExpectationError("dimensional.L_m (the instrument's physical pitch-to-yaw length, m) is required for geometry_anchor")
                if u is None or not (0.0 <= float(u) < 1.0):
                    raise ExpectationError("dimensional.u_rel (declared relative uncertainty of L_m, 0 <= u_rel < 1) is required for geometry_anchor")
                dq = float(di.get("delta_q_rad", 0.5))
                if not (0.0 < abs(dq) < math.pi):
                    raise ExpectationError("dimensional.delta_q_rad must lie in (0, pi)")
                ref = di.get("reference_joints")
                aa = di.get("axes_angle_deg", 90.0)
                e.dimensional.L_m, e.dimensional.u_rel, e.dimensional.delta_q_rad = float(L), float(u), dq
                e.dimensional.L_source = str(di.get("L_source", ""))
                e.dimensional.pitch_joint = str(di.get("pitch_joint", "wrist_pitch"))
                e.dimensional.yaw_joint = str(di.get("yaw_joint", "wrist_yaw"))
                e.dimensional.axes_angle_deg = (float(aa) if aa is not None else None)
                e.dimensional.reference_joints = ([float(v) for v in ref] if ref is not None else None)
        if te:
            sm = te.get("state_machine", "any")
            sb = te.get("stop_behaviour", "any")
            ra = te.get("rate", "any")
            for v, allowed, name in ((sm, STATE_MACHINE, "state_machine"), (sb, STOP_BEHAVIOUR, "stop_behaviour"), (ra, RATE, "rate")):
                if v not in allowed:
                    raise ExpectationError(f"temporal.{name} must be one of {allowed}, got {v!r}")
            hz = te.get("horizon_s")
            if hz is not None and float(hz) <= 0:
                raise ExpectationError("temporal.horizon_s must be positive")
            e.temporal = TemporalExpectation(state_machine=sm, stop_behaviour=sb, rate=ra, horizon_s=(float(hz) if hz is not None else None))
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
