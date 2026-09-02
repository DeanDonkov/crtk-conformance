import json
from crtk_conformance.report import build_report, text_summary, SCHEMA_PATH
from crtk_conformance.probes.base import ProbeResult, Outcome
from crtk_conformance.thresholds import Tolerance


def test_report_validates_against_schema():
    r = ProbeResult("FrameSemanticsProbe", "spatial", Outcome.DIVERGENT, estimates={"x": float("nan")}, predicted_error_m=0.2, predicted_error_ci=[0.19, 0.21], decision_basis="test")
    rep = build_report("/PSM1", Tolerance(), {"topics": {}}, [r])
    assert rep["summary"]["spatial"] == "divergent" and rep["probes"][0]["estimates"]["x"] is None
    assert "FrameSemanticsProbe" in text_summary(rep)
    json.dumps(rep)
