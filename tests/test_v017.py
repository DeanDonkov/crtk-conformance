"""0.1.7 decision rules (RC12 review, points 1 and 2), replayed on the archived v0.1.6 campaigns where possible.

1. Consistency gate: a raised command/feedback consistency diagnostic withholds the unit verdict and the command-semantic
   reading of the frame verdict (case K1 reported a command-frame error as a unit error).
2. Sound fault bound: every faulted trial bounds tau_w by the time its FAULT was observed; the 0.1.6 bound gap + L is
   kept only as an estimate conditional on the post-gap command having arrived (one 0.1.6 interval excluded tau_w).
"""
import glob
import json
import os

from crtk_conformance.dimensional import consistency_gate
from crtk_conformance.liveness import (attributable, fault_horizon_decision, fault_observation_window, timeout_interval_from_trials,
                                       trip_upper_bound)
from crtk_conformance.probes.base import Outcome, ProbeResult
from crtk_conformance.report import build_report
from crtk_conformance.thresholds import Tolerance

V16 = os.path.join(os.path.dirname(__file__), "..", "validation", "v0.1.6", "mock")


def test_consistency_gate():
    assert consistency_gate(Outcome.DIVERGENT, True) == (Outcome.UNDETERMINED, True)
    assert consistency_gate(Outcome.CONFORMANT, True) == (Outcome.UNDETERMINED, True)
    assert consistency_gate(Outcome.UNDETERMINED, True) == (Outcome.UNDETERMINED, False)
    for o in Outcome:
        assert consistency_gate(o, False) == (o, False)


def _consistency(r):
    return r["result"]["estimates"]["command_feedback_consistency"]["flag_non_shared_binding_or_tracking_deficit"]


def test_k1_replay_withholds_only_the_k1_unit_verdicts():
    files = sorted(glob.glob(os.path.join(V16, "K", "K*.json")))
    assert len(files) == 9
    for p in files:
        r = json.load(open(p))["scale"]
        flagged = _consistency(r)
        out, withheld = consistency_gate(Outcome(r["result"]["outcome"]), flagged)
        if os.path.basename(p).startswith("K1_"):
            assert flagged and withheld and out == Outcome.UNDETERMINED and r["result"]["outcome"] == "divergent"
        else:
            assert not flagged and not withheld and out.value == r["result"]["outcome"] == "conformant"


def test_report_verdict_kinds():
    tol = Tolerance(epsilon_m=0.001, workspace_radius_m=0.1, speed_m_s=0.05, client_rate_hz=100.0, jitter_max_s=0.005)
    frame = ProbeResult("frame_semantics", "spatial", Outcome.CONFORMANT)
    scale = ProbeResult("scale_units", "dimensional", Outcome.DIVERGENT,
                        estimates={"command_feedback_consistency": {"flag_non_shared_binding_or_tracking_deficit": True}})
    disc = {"topics": {}}
    rep = build_report("/PSM1", tol, disc, [frame, scale])
    assert rep["summary"]["spatial"] == "conformant" and rep["summary"]["dimensional"] == "undetermined"
    vk = rep["verdict_kinds"]
    assert vk["spatial_feedback_binding"] == "conformant" and vk["spatial_command_semantic"] == "undetermined"
    assert vk["shared_binding_assumption"].startswith("contradicted")
    rep6 = build_report("/PSM1", tol, disc, [frame, scale], consistency_gate=False)
    assert rep6["summary"]["dimensional"] == "divergent" and rep6["verdict_kinds"]["spatial_command_semantic"] == "conformant"
    rep0 = build_report("/PSM1", tol, disc, [frame])
    assert rep0["verdict_kinds"]["shared_binding_assumption"] == "assumed; not tested"


def _t(cls, gap, **kw):
    d = {"class": cls, "gap_s": gap, "last_stream_latency_s": 0.002}
    d.update(kw)
    return d


def test_fault_bound_017():
    lost_then_fault = _t("faulted", 0.228, state_observed_at_s=0.528)  # the post-gap command was lost; the fault fired in the wait
    assert trip_upper_bound(lost_then_fault, L=0.014, rule="0.1.6") == 0.228 + 0.014     # gap + L: excludes tau_w = 0.25
    assert trip_upper_bound(lost_then_fault, L=0.014, rule="0.1.7") == 0.528             # sound
    old = _t("faulted", 0.3)                                                             # a v0.1.3 record: no observation time
    assert trip_upper_bound(old, L=0.014, rule="0.1.7") is None and not attributable(old, 0, "0.1.7")
    fw = fault_observation_window(0.3, 0.01)
    assert abs(fw - 0.51) < 1e-12 and abs(trip_upper_bound(old, L=0.014, rule="0.1.7", fault_window_s=fw) - 0.81) < 1e-12
    assert attributable(old, 0, "0.1.7", fw)
    drift = _t("drifted", 0.6, drift_onset_s=0.5)
    assert trip_upper_bound(drift, L=0.014, rule="0.1.7") == trip_upper_bound(drift, L=0.014, rule="0.1.6") == 0.5
    rej = _t("rejected", 0.3, confirmation={"confirmed": True})
    assert trip_upper_bound(rej, L=0.014, rule="0.1.7") == trip_upper_bound(rej, L=0.014, rule="0.1.6") == 0.3 + 0.014


def _replay(p, rule):
    d = json.load(open(p))
    obs = d["result"]["observations"]; Lv = obs["liveness"]; R = obs["resolution"]; tau = Lv.get("tau_w_estimate_s") or {}
    lost = int(Lv.get("baseline_command_loss", {}).get("lost", 0))
    iv = timeout_interval_from_trials(Lv["trials"], L=tau["latency_allowance_s"], G=tau["granularity_allowance_s"], fp=float(R.get("feedback_period_s") or 0.01),
                                      hold_tol=Lv["hold_tolerance_m"], stop_class=Lv["stop_class"], baseline_loss=lost, rule=rule,
                                      fault_window_s=fault_observation_window(Lv["response_timeout_s"], tau["granularity_allowance_s"]) if rule == "0.1.7" else None)
    return d, iv


def test_loss_campaign_replay_017_contains_every_timeout():
    files = sorted(glob.glob(os.path.join(V16, "L", "L16_*_fault250_*.json")))
    assert len(files) == 64
    for p in files:
        d, iv = _replay(p, "0.1.7")
        tau = d["result"]["observations"]["liveness"]["tau_w_estimate_s"]
        if "latency_allowance_s" not in tau or iv["status"] != "formed":
            continue
        assert iv["interval_low_s"] <= d["truth"]["tau_w_s"] <= iv["interval_high_s"], (os.path.basename(p), iv["interval_low_s"], iv["interval_high_s"])


def test_the_016_excluding_interval():
    p = os.path.join(V16, "L", "L16_016_fault250_iid5_03.json")
    d, iv6 = _replay(p, "0.1.6")
    assert iv6["interval_high_s"] < 0.25                                   # 0.1.6: [0.228, 0.242] s excludes tau_w
    d, iv7 = _replay(p, "0.1.7")
    assert iv7["interval_low_s"] <= 0.25 <= iv7["interval_high_s"]         # 0.1.7 contains it
    need = 1.0 / 100.0 + 0.005
    assert fault_horizon_decision({"status": "ok", "interval_low_s": iv6["interval_low_s"], "interval_high_s": iv6["interval_high_s"]}, 0.245, need)[0] == "satisfied"
    assert fault_horizon_decision({"status": "ok", "interval_low_s": iv7["interval_low_s"], "interval_high_s": iv7["interval_high_s"]}, 0.245, need)[0] == "undetermined"


def test_fault_horizon_decision():
    need = 0.015
    tau = {"status": "ok", "interval_low_s": 0.24, "interval_high_s": 0.55}
    assert fault_horizon_decision(tau, 0.1, need)[0] == "violated"
    assert fault_horizon_decision(tau, 0.25, need)[0] == "undetermined"
    assert fault_horizon_decision(tau, 1.0, need)[0] == "satisfied"
    assert fault_horizon_decision({"status": "ok", "interval_low_s": 0.01, "interval_high_s": 0.02}, 1.0, need)[0] == "undetermined"  # eq. (7) straddled
    assert fault_horizon_decision({"status": "upper_bound", "upper_bound_s": 0.012}, 1.0, need)[0] == "violated"
    assert fault_horizon_decision(None, 1.0, need)[0] == "undetermined"
