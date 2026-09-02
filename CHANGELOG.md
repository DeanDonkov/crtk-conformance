# Changelog

## 0.1.0 — 2026-09-02

First release, accompanying the preprint.

- `crtk_conformance`: ROS 1 adapter (master-API discovery, subscriptions, publications, one-hop /tf lookup);
  `FrameSemanticsProbe`, `ScalingUnitsProbe`, `RateSensitivityProbe` (state precondition, liveness, effective rate);
  tolerance-derived thresholds; Student-t statistics over repeated trials; JSON report with schema; CLI.
- `crtk_mock`: kinematic CRTK-compatible node with injectable binding transform, unit scale, operating-state machine,
  liveness timeout (fault/release), loop and publish rates, delay/jitter, drops, noise, missing topics; five presets.
- `validation/`: harness, grid runner (F, S, T, R, P, M experiments), analysis and figure scripts, archived results.
- Not implemented: ROS 2 backend, PDF report, any TF-tree reasoning beyond one hop.
