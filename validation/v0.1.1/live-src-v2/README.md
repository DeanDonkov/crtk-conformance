# Live run against the released Surgical Robotics Challenge v2.0.0 CRTK interface (reported run)

This is the run reported in the manuscript (RC3, Section 7.3, [M-live]) for SRC v2.0.0. Nothing in this
directory was edited after the run except this README. Earlier attempts are kept beside it, each with a README:
`../live-src-v2-run1-home-pose/`, `../live-src-v2-run2-preliminary/`, `../live-src-v2-run3-failed-startup/`,
`../live-src-v2-run{4,5}-superseded/`.

| Item | Value |
|---|---|
| Target | `surgical_robotics_challenge` v2.0.0 = 03befbf1028d22b0a6495059af51e397646570cf, unmodified |
| Simulator | AMBF branch `ambf-2.0` @ 16a8151816407dfedff6785748ae72323c280f67, headless (`-g false`, Xvfb), the release's own launch args (`-l 0,1,6,7,8,9 -p 200 -t 1 --override_max_comm_freq 100 --override_min_comm_freq 100`) |
| Probe | crtk-conformance e546af16297ce5c83a7a916b00e4b3c7ac712c62 (v0.1.1) |
| Environment | `focal-crtk:rc3` (see `../../environment/`), Python 3.8.10, ROS Noetic built from source; `meta_env.json` |
| Arm | PSM1 only (`--two False --ecm False --scene False`), moved to q = (0, 0, 0.1, 0, 0, 0) (metres; 40 % insertion) by `live_aux.py` before probing |
| Commands | `../../environment/run_live.sh v2 <this dir>`; three probe runs A/B/C with the options listed in `logs/run_*.log` and the report `parameters` |

## What was observed (all numbers are in `live_aux.json` and the three `report_*.json`)

* Same topic inventory as v1.0.0 (six present, six absent, no `/tf`); `measured_cp.header.frame_id = psm1/baselink`;
  `T_b_w.header.frame_id = world` (translation (0.130, 0.393, 0.873) in this scene). `measured_cp` at ≈ 119.3 Hz.
* Joint-error de-perturbation: printed errors (−0.00049, −0.03699, 0.00415, 0, 0, 0); `measured_js` minus the
  simulator's joint state (`/ambf/env/psm1/baselinksimple/State`) = (0.00049, 0.03699, −0.00415, 0, 0, 0).
* ‖measured_cp‖ at q = (0, 0, 0.1) = 0.0861 m, equal to the release's own FK; see `../CROSS_VERSION_FK_NOTE.md`.
* Resting feedback noise σ̂ ≈ 3.5–4.1 × 10⁻⁵ m; a 10 mm `servo_cp` step settles with a residual of 1 µm; internal
  command/measurement ratio 0.9987 [0.9974, 1.0001], 1.0007 [0.9976, 1.0038], 0.9986 [0.9972, 0.9999] (three runs,
  n = 10, 5 mm streamed steps). No unit is decided (no anchor).
* Run A: every class `undetermined` by construction. Run B (SRC-authored client): temporal `undetermined` — state
  and stop behaviour (hold; no policy within 2 s, n = 5, drift ≤ 0.17 mm) satisfied, rate `undetermined` because the
  feedback trajectory passed within δ = 5σ̂ of the commanded targets for at most 4 % of samples (the release
  interpolates towards each setpoint, median response latency 70–87 ms). Run C (dVRK-authored client): temporal
  `divergent`.
* Resolution floor 9.7–10.4 ms.

## What this run does not show

No unit scale, no spatial binding relative to the arm base, no execution rate, nothing about the dVRK or the
Raven-II, and nothing about the v2.0.0 interface under other launch options.
