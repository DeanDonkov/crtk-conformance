# First v0.1.1 mock campaign (kept as data; superseded by `../mock/`)

Probe code: crtk-conformance 8a965ea (probes at d8ef24cb). Complete campaign per `VALIDATION_PLAN_V011.md`,
2696 s. It exposed one probe defect, visible in `L_release_000.json` and `tables/L_liveness.csv`: the
release policy with τ_w = 0.1 s and 20 mm/s drift was classified `no_policy_within_range` (outcome
`conformant` against a `hold` expectation — a false conformant). Cause: the temporal probe measured its
resting feedback noise during a 1 s window of command silence, so a release policy shorter than the window
fired *during* the noise measurement, the drift was taken for noise (σ̂ = 7.4 mm instead of 0.04 mm), and the
hold tolerance inflated to 74 mm, above the 28 mm the arm drifted during the 1.5 s gap. The 0.5 s release
policy, longer than the window, was detected correctly (τ̂_w = 0.554 s).

Fix (next commit): the noise window is taken while holding the current pose under a command stream at the
client rate, so no silence-triggered policy can fire during it; a regression test
(`test_liveness_short_release_with_drift_is_detected`) was added, and the general false-conformant
definition in `analyze_v011.py` now also covers liveness runs whose classified stop policy is not the injected
one (this directory's `tables/SUMMARY.md` was regenerated with that definition and reports the run).
The campaign was rerun in full on the fixed code; the live runs were repeated on the same commit.

The same campaign also contains the scale-probe defect found later in the second campaign
(`PRESET_emul-ambf-object-watchdog_authored.json`: dimensional `divergent`, ŝ far from 1, for an emulated unit of 1;
see `../mock-run2-superseded/README.md`); the regenerated `tables/SUMMARY.md` lists it under "False divergent".

Nothing in this directory was edited after the run except the regenerated `tables/` (analysis output).
