"""Independent RC3 decision-level checks; no ROS, simulator, or robot is started.

Run with Python + numpy + scipy, passing the extracted RC3 repository:
  python reproduce_counterexamples.py /absolute/path/to/crtk-conformance

These checks exercise the released pure decision functions and the frame CI
formula. They are not new platform measurements or a ROS integration campaign.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.stats import t

repo = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(repo / "src"))
from crtk_conformance.thresholds import Tolerance
from crtk_conformance.probes.base import decide
from crtk_conformance.rate_estimator import estimate_rate, rate_subverdict_v013_archival as rate_subverdict  # 0.1.4: the rule this review was answered under

out = {}
tol = Tolerance(epsilon_m=.001, workspace_radius_m=.1)
angle, translation = math.radians(.3), .0007
upper = tol.spatial_error(translation, angle)
exact = math.hypot(translation, 2 * math.sin(angle / 2) * .1)
out["spatial_upper_bound_counterexample"] = {
    "translation_parallel_to_rotation_axis_m": translation,
    "rotation_deg": .3, "workspace_radius_m": .1, "epsilon_m": .001,
    "exact_max_error_m": exact, "paper_bound_m": upper,
    "code_verdict_with_exact_estimate": decide(upper, upper, .001).value,
}
assert exact < .001 < upper

p = np.array([.03, .04, .05])
R = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
tr = np.array([.2, 0, 0])
executed = R.T @ (p - tr)
feedback = R @ executed + tr
out["closed_loop_frame_invisibility"] = {
    "command": p.tolist(), "executed_local": executed.tolist(),
    "feedback": feedback.tolist(),
    "reported_tracking_error_m": float(np.linalg.norm(feedback - p)),
    "physical_error_m": float(np.linalg.norm(executed - p)),
}

# Exact-rotation, independent Gaussian translation special case of frame.py:
# centre = norm(mean(t_i)), half width = tcrit * sd(norm(t_i))/sqrt(n).
rng = np.random.default_rng(20260905)
n, reps = 10, 100000
xyz = rng.normal(size=(reps, n, 3)) * 1e-4
centre = np.linalg.norm(xyz.mean(axis=1), axis=1)
hw = t.ppf(.975, n - 1) * np.linalg.norm(xyz, axis=2).std(axis=1, ddof=1) / math.sqrt(n)
lo = np.maximum(0, centre - hw)
out["frame_CI_coverage_identity_translation_only"] = {
    "replicates": reps, "trials_per_replicate": n,
    "per_axis_trial_noise_m": 1e-4, "nominal_coverage": .95,
    "actual_zero_error_coverage": float(np.mean(lo <= 0)),
    "false_divergent_fraction_at_epsilon_1um": float(np.mean(lo > 1e-6)),
    "note": "Reproduces RC3 interval-centering rule in exact-rotation, independent isotropic Gaussian translation special case; not a ROS rerun.",
}

# First 99 commands dropped. Only final command delivered at t=.99 s.
# The resulting 50 mm/s continuous trajectory crosses every earlier target.
targets = np.zeros((100, 3))
targets[:, 0] = np.arange(1, 101) * .0005
samples = np.vstack([np.zeros((100, 3)), targets])
est = estimate_rate(targets, samples, np.arange(200) * .01,
                    np.arange(100) * .01, 100., .00005,
                    "synthetic independent counterexample", 0., 2.)
out["rate_counterexample_final_command_only"] = {
    "description": "First 99 commands dropped; final goal accepted at 0.99s; straight-line 50 mm/s motion crosses all prior target positions. 100 prior stationary samples then 100 moving samples.",
    "commands_actually_accepted": 1, "estimate": est.to_dict(),
    "required_rate_hz": 50, "code_subverdict": rate_subverdict(est, 50),
}
assert est.transitions == 100 and rate_subverdict(est, 50) == "satisfied"

archive = repo / "validation/v0.1.1/mock"
meta = json.loads((archive / "meta.json").read_text())
mismatches = [f for f, h in meta["sha256"].items()
              if hashlib.sha256((archive / f).read_bytes()).hexdigest() != h]
out["mock_archive_hash_check"] = {"files": len(meta["sha256"]), "mismatches": mismatches}
rows = list(csv.DictReader((archive / "tables/L_liveness.csv").open()))
valid = [r for r in rows if r["tau_status"] == "ok"]
covered = [r["run"] for r in valid
           if float(r["tau_ci_low"]) <= float(r["tau_true_s"]) <= float(r["tau_ci_high"])]
out["archived_liveness_CI_truth_coverage"] = {
    "estimates": len(valid), "contain_injected_timeout": len(covered), "covered_runs": covered,
}
r = next(r for r in valid if r["run"] == "L_fault_002")
margin = .051
out["liveness_margin_counterexample"] = {
    "archive": "L_fault_002.json", "true_timeout_s": float(r["tau_true_s"]),
    "reported_lower_CI_s": float(r["tau_ci_low"]),
    "client_period_plus_jitter_s": margin,
    "code_accepts_margin": margin < float(r["tau_ci_low"]),
    "actual_margin_satisfied": margin < float(r["tau_true_s"]),
}
dest = Path(__file__).with_name("counterexample-results.json")
dest.write_text(json.dumps(out, indent=2) + "\n")
print(json.dumps(out, indent=2))
