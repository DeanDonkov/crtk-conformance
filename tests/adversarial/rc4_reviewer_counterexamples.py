"""Independent RC4 decision checks. No ROS, robot, or simulator is started.

Usage: python reproduce_counterexamples.py /path/to/extracted/crtk-conformance
Requires numpy/scipy. Synthetic traces exercise the unchanged released estimator.
The horizon check replays archived observations through the unchanged run() method,
replacing only data collection and ROS imports with explicit offline stubs.
"""
import hashlib
import json
from pathlib import Path
import sys
import types

import numpy as np

repo = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(repo / "src"))
from crtk_conformance.rate_estimator import estimate_acceptance, rate_subverdict_v013_archival as rate_subverdict  # 0.1.4: the rule this review was answered under
from crtk_conformance.spatial import spatial_decision
from crtk_conformance.geometry import make_pose, axis_angle
from crtk_conformance.probes.base import decide
from crtk_conformance.expectations import Expectations
from crtk_conformance.thresholds import Tolerance

out = {}

# True accepted updates every 24 ms; publication every 12 ms. All commands
# have unique targets and the client sustains a 6 ms period with no stalls.
send = np.arange(200) * .006
targets = np.zeros((200, 3))
targets[:, 0] = np.arange(200) * .0003
st = np.arange(101) * .012
ix = np.minimum((np.arange(101) // 2) * 4, 196)
acc = estimate_acceptance(targets, targets[ix], st, send, 1e-7, st[-1])
out["publication_allowance_false_satisfaction"] = {
    "description": "Updates every 24 ms; 12 ms publication period; required period 20 ms.",
    "estimate": acc.to_dict(), "subverdict": rate_subverdict(acc, 50),
    "true_update_period_s": .024, "true_zoh_supremum_m_at_50mm_s": .05 * .024,
    "epsilon_m": .001,
}
assert rate_subverdict(acc, 50) == "satisfied"
assert acc.max_stale_s > .020

# A genuine low-level setpoint channel receives the client's 100 Hz stream
# through a queue at 50 Hz. Goals remain piecewise constant and in order;
# their source age grows although the inter-update gaps remain short.
send = np.arange(100) * .01
targets = np.zeros((100, 3))
targets[:, 0] = send * .05
st = np.arange(403) * .005
ix = np.minimum(np.floor((st - .01 + 1e-10) / .02).astype(int), 99)
samples = np.zeros((len(st), 3))
samples[:, 0] = -.001
valid = ix >= 0
samples[valid] = targets[ix[valid]]
acc = estimate_acceptance(targets, samples, st, send, 1e-7, st[-1])
t_check = .99
j = int(np.floor((t_check - .01 + 1e-10) / .02))
out["source_age_false_satisfaction"] = {
    "description": "100 Hz source, ordered 50 Hz delivery/application, first delay 10 ms, genuine held low-level setpoints.",
    "estimate": acc.to_dict(), "subverdict": rate_subverdict(acc, 50),
    "check_time_s": t_check, "held_target_index": j,
    "source_timestamp_of_held_target_s": float(send[j]),
    "actual_trajectory_error_m": float(.05 * (t_check - send[j])),
    "reported_zoh_bound_m": .05 * acc.max_stale_s,
    "epsilon_m": .001,
}
assert rate_subverdict(acc, 50) == "satisfied"
assert .05 * (t_check - send[j]) > .024

# A setpoint left over from a previous sweep equals this sweep's final target.
# Every NEW command is subsequently applied promptly; the old sample must not
# be counted as acceptance of a command that will only be sent at t=.99.
send = np.arange(100) * .01
targets = np.zeros((100, 3))
targets[:, 0] = (np.arange(100) + 1) * .00002
st = np.arange(202) * .005
samples = np.empty((len(st), 3))
samples[0] = targets[-1]
ix = np.minimum(np.floor((st[1:] - .005 + 1e-10) / .01).astype(int), 99)
samples[1:] = targets[ix]
acc = estimate_acceptance(targets, samples, st, send, 1e-7, st[-1])
out["preexisting_setpoint_false_violation"] = {
    "description": "Old setpoint equals the final target; all 100 new commands apply after 5 ms, every 10 ms.",
    "true_new_commands_accepted": 100,
    "estimate": acc.to_dict(), "subverdict": rate_subverdict(acc, 50),
}
assert acc.accepted == 1 and rate_subverdict(acc, 50) == "violated"

# Independent check that the old exact-spatial-error counterexample is fixed.
E = make_pose(axis_angle([0, 0, 1], np.deg2rad(.3)), [0, 0, .0007])
sd = spatial_decision([E.copy() for _ in range(10)], .1)
out["rc3_spatial_counterexample_now_fixed"] = {
    "exact_error_m": sd.e_max_m, "ci_m": [sd.ci_low_m, sd.ci_high_m],
    "verdict": decide(sd.ci_low_m, sd.ci_high_m, .001).value,
}
assert out["rc3_spatial_counterexample_now_fixed"]["verdict"] == "conformant"

archive = repo / "validation/v0.1.2/mock"
meta = json.loads((archive / "meta.json").read_text())
out["archive_hash_check"] = {
    "files": len(meta["sha256"]),
    "mismatches": [f for f, h in meta["sha256"].items()
                   if hashlib.sha256((archive / f).read_bytes()).hexdigest() != h],
}

out["archived_delay_runs"] = []
for path in sorted(archive.glob("T_delay_*.json")):
    r = json.loads(path.read_text())
    rows = r["result"]["observations"]["effective_rate"]["per_rate"]
    row = min(rows, key=lambda x: abs(x["command_rate_requested_hz"] - 100))
    out["archived_delay_runs"].append({"file": path.name, "truth": r["truth"],
        "subverdicts": r["result"]["estimates"]["sub_verdicts"], "acceptance": row["acceptance"]})

# Stub only ROS/data collection. The released expectation decision code is
# imported unchanged, and run() consumes the real L_fault_008 observations.
adapter = types.ModuleType("crtk_conformance.adapter")
adapter.PlatformAdapter = object
adapter.pose_msg_to_matrix = lambda x: x
sys.modules[adapter.__name__] = adapter
common = types.ModuleType("crtk_conformance.probes.common")
common.ensure_enabled = lambda *a, **k: None
common.wait_settled = lambda *a, **k: (np.eye(4), None)
sys.modules[common.__name__] = common
from crtk_conformance.probes.rate import RateSensitivityProbe

class OfflineAdapter:
    discovery = {"topics": {x: {"present": True} for x in ("measured_cp", "servo_cp")}}
    def subscribe(self, *a): return object()
    def wait_for(self, *a): return np.eye(4)

r = json.loads((archive / "L_fault_008.json").read_text())
obs = r["result"]["observations"]
exp = Expectations.from_dict({"temporal": {"stop_behaviour": "fault", "horizon_s": .1}})
probe = RateSensitivityProbe(OfflineAdapter(), Tolerance(), expectations=exp, gap_max_s=1.5)
probe.measure_resolution = lambda *a: obs["resolution"]
probe.probe_state_precondition = lambda *a: obs["state_precondition"]
probe.probe_liveness = lambda *a: obs["liveness"]
probe.probe_effective_rate = lambda *a: {"per_rate": []}
result = probe.run()
out["fault_horizon_false_satisfaction"] = {
    "source_archive": "L_fault_008.json", "truth": r["truth"],
    "declared_horizon_s": .1, "tested_horizon_s": 1.5,
    "timeout_estimate": obs["liveness"]["tau_w_estimate_s"],
    "outcome": result.outcome.value, "subverdicts": result.estimates["sub_verdicts"],
    "decision_basis": result.decision_basis,
}
assert obs["liveness"]["tau_w_estimate_s"]["interval_low_s"] > .1
assert result.estimates["sub_verdicts"]["stop_behaviour"] == "satisfied"

# A sample p95 of ten response latencies is the ninth order statistic here.
# For an otherwise normal 5% tail, calibration can miss every slow response.
out["latency_allowance_is_not_a_bound"] = {
    "response_probes": 10, "rare_slow_probability": .05,
    "probability_of_zero_slow_calibration_observations": .95 ** 10,
    "probability_of_at_least_one_slow_in_100_later_transports": 1 - .95 ** 100,
    "note": "Analytic probability illustration, not a ROS or liveness replay.",
}

# Test the manuscript's independence-only wording, not the code docstring's
# narrower iid Gaussian conditional statement. Independent zero-mean errors
# can have a rare tail that a sample covariance usually does not see.
rng = np.random.default_rng(20260909)
hits = false_div = 0
reps = 2000
for _ in range(reps):
    ts = rng.normal(0, 1e-6, (10, 3))
    ts[:, 0] += np.where(rng.random(10) < .05, 380e-6, -20e-6)
    sd = spatial_decision([make_pose(np.eye(3), t) for t in ts], .1)
    hits += sd.ci_low_m <= 0 <= sd.ci_high_m
    false_div += sd.ci_low_m > 1e-6
scope = {
    "replicates": reps, "n": 10, "seed": 20260909,
    "zero_mean_iid_translation_noise": "x: 95% -20um + 5% +380um, plus independent 1um Gaussian on each axis",
    "coverage": hits / reps, "false_divergence_at_1um": false_div / reps,
    "interpretation": "Stress check of manuscript independence-only statement; does NOT refute the implementation docstring Gaussian conditional model.",
}
out["non_gaussian_scope_check"] = scope
Path(__file__).with_name("non-gaussian-scope-check.json").write_text(json.dumps(scope, indent=2) + "\n")

dest = Path(__file__).with_name("counterexample-results.json")
dest.write_text(json.dumps(out, indent=2) + "\n")
print(json.dumps(out, indent=2))
