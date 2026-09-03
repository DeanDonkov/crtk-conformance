# Changelog

## Unreleased (branch `rc2-targeted-revision`, 2026-09-03) — documentation only

- `validation/analyze.py`: two Figure 3 label strings corrected to the manuscript's equation numbering
  ("incremental (3b)" -> "(5)"; panel (c) "eq. (6)" -> "eq. (9)"). Re-rendered offline from the archived
  `validation/results/`; plotted data unchanged (pixel-identical outside the two labels).
- Comment/docstring-only equation-number corrections in `geometry.py`, `thresholds.py`, `probes/rate.py`
  (module docstring) and `tests/test_geometry.py`; README section "Equation numbering" added.
- No probe, mock, threshold, schema or `decision_basis` string changed; `validation/results/` untouched.

### Known issues (to be addressed in a future release with a fresh validation archive)

- `RateSensitivityProbe.probe_effective_rate`: the `observation_bounded_by_publish_rate` criterion
  (`eff >= 0.9 * publish_rate_hz`) does not flag a run whose observable rate is reduced well below the
  publish rate by the probe's own send-loop shortfall combined with sampling coincidence (archived example:
  `T_rate_000.json`, nominal 200 Hz client, 85 distinct positions over 1.398 s = 60.8 Hz, flag `false`).
  The estimate is an observable effective command/state rate, not the internal execution rate. Changing the
  criterion alters the serialised flag and therefore requires re-running the validation grid.
- The probe's Python send loop limits the client rates it can present (795 Hz achieved at a nominal 1 kHz).

## 0.1.0 — 2026-09-02

First release, accompanying the preprint.

- `crtk_conformance`: ROS 1 adapter (master-API discovery, subscriptions, publications, one-hop /tf lookup);
  `FrameSemanticsProbe`, `ScalingUnitsProbe`, `RateSensitivityProbe` (state precondition, liveness, effective rate);
  tolerance-derived thresholds; Student-t statistics over repeated trials; JSON report with schema; CLI.
- `crtk_mock`: kinematic CRTK-compatible node with injectable binding transform, unit scale, operating-state machine,
  liveness timeout (fault/release), loop and publish rates, delay/jitter, drops, noise, missing topics; five presets.
- `validation/`: harness, grid runner (F, S, T, R, P, M experiments), analysis and figure scripts, archived results.
- Not implemented: ROS 2 backend, PDF report, any TF-tree reasoning beyond one hop.
