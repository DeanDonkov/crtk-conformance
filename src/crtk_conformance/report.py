"""Reporter: structured JSON (validated against schema/report.schema.json) plus a text summary.

No PDF output is implemented.
"""
from __future__ import annotations

import datetime as _dt
import json
import math
import os
from typing import Any, Dict, List

import jsonschema

from . import __version__
from .probes.base import ProbeResult, Outcome
from .thresholds import Tolerance

SCHEMA_PATH = os.path.join(os.path.dirname(__file__), "schema", "report.schema.json")


def _clean(o):
    """Make numpy / NaN / inf JSON-safe."""
    import numpy as np

    if isinstance(o, dict):
        return {str(k): _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    if isinstance(o, (np.floating,)):
        o = float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (np.bool_,)):
        return bool(o)
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None
    if isinstance(o, Outcome):
        return o.value
    return o


def build_report(namespace: str, tol: Tolerance, discovery: Dict[str, Any], results: List[ProbeResult], ros_master_uri: str = "",
                 expectations=None, parameters: Dict[str, Any] = None) -> Dict[str, Any]:
    summary = {"spatial": "undetermined", "dimensional": "undetermined", "temporal": "undetermined"}
    for r in results:
        summary[r.binding_class] = r.outcome.value
    rep = {
        "tool": "crtk-conformance",
        "version": __version__,
        "generated_at": _dt.datetime.now(_dt.timezone.utc).isoformat(),
        "namespace": namespace,
        "ros_master_uri": ros_master_uri,
        "tolerance": tol.to_dict(),
        "expectations": expectations.to_dict() if expectations is not None else None,
        "parameters": parameters or {},
        "discovery": discovery,
        "probes": [r.to_dict() for r in results],
        "summary": summary,
        "semantics": "conformant = the discovered binding satisfies an explicitly declared client expectation within the stated tolerance; "
                     "no declared expectation -> undetermined (observations reported, no verdict)",
    }
    rep = _clean(rep)
    with open(SCHEMA_PATH) as f:
        jsonschema.validate(rep, json.load(f))
    return rep


def text_summary(rep: Dict[str, Any]) -> str:
    lines = [f"crtk-conformance {rep['version']} — namespace {rep['namespace']} — {rep['generated_at']}"]
    t = rep["tolerance"]
    lines.append(f"tolerance epsilon = {t['epsilon_m']*1e3:.3f} mm, r_ws = {t['workspace_radius_m']:.3f} m, v = {t['speed_m_s']*1e3:.0f} mm/s, client {t['client_rate_hz']:.0f} Hz, J_max {t['jitter_max_s']*1e3:.1f} ms")
    e = rep.get("expectations")
    if e:
        lines.append(f"expectations ({e.get('source')}): spatial {e['spatial']['mode']}, dimensional {e['dimensional']['mode']}, temporal state_machine={e['temporal']['state_machine']} stop_behaviour={e['temporal']['stop_behaviour']} rate={e['temporal']['rate']}")
    for p in rep["probes"]:
        lines.append(f"[{p['binding_class']:11s}] {p['probe']:24s} -> {p['outcome'].upper():12s} ({p['duration_s']:.1f} s)")
        lines.append(f"    basis: {p['decision_basis']}")
        for n in p["notes"]:
            lines.append(f"    note : {n}")
    s = rep["summary"]
    lines.append(f"summary: spatial={s['spatial']} dimensional={s['dimensional']} temporal={s['temporal']}")
    return "\n".join(lines)
