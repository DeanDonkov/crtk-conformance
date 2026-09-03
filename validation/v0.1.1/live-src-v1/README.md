# Live run against the released Surgical Robotics Challenge v1.0.0 CRTK interface (reported run)

This is the run reported in the manuscript (RC3, Section 7, [M-live]) for SRC v1.0.0. Nothing in this
directory was edited after the run; the earlier attempts that led to it are kept as
`../live-src-v1-run{1..7}-preliminary/` and `../live-src-v1-run6-home-pose/`, each with a README
explaining the probe defect or configuration problem it exposed.

| Item | Value |
|---|---|
| Target | `surgical_robotics_challenge` v1.0.0 = 158b554e1a3e7cd618eadedf0505865891e25f27, unmodified |
| Simulator | AMBF branch `ambf-2.0` @ 16a8151816407dfedff6785748ae72323c280f67, headless (`-g false`, Xvfb), launch args in `logs/run.log` |
| Probe | crtk-conformance d8ef24cb6de340e8c949abb549b588e941f518ad (v0.1.1) |
| Environment | `focal-crtk:rc3` (see `../../environment/`), Python 3.8.10, ROS Noetic built from source; `meta_env.json` |
| Arm | PSM1 only (`--two False --ecm False --scene False`), moved to q = (0, 0, 1.0, 0, 0, 0) interface units (40 % insertion) by `live_aux.py` before probing |
| Commands | `../../environment/run_live.sh v1 <this dir>`; three probe runs A/B/C with the options listed in `logs/run_*.log` and the report `parameters` |

## What was observed (all numbers are in `live_aux.json` and the three `report_*.json`)

* Topics under `/CRTK/psm1/`: `measured_cp`, `servo_cp`, `measured_js`, `servo_jp`, `T_b_w`, `measured_cv`
  present; `local/measured_cp`, `setpoint_cp`, `local/setpoint_cp`, `servo_cr`, `operating_state`,
  `state_command` absent; no `/tf`. `measured_cp.header.frame_id = psm1/baselink`; `T_b_w.header.frame_id = world`.
* `measured_cp` published at ≈ 119.5 Hz (inter-arrival mean 8.3 ms).
* Joint-error de-perturbation (manuscript Section 4.4, M2): the interface printed the joint errors it injected
  (`logs/launch_crtk_interface.log`, `Joint Errors:` line); `measured_js` minus the simulator's own joint
  state equals their negatives to 1e-5 (`live_aux.json` → `joint_configs.*.reported_minus_simulated_joint`).
* ‖measured_cp‖ at the working configuration = 0.9673 interface units (used for the cross-version
  comparison with v2.0.0 in `../live-src-v2/`).
* Run A (no expectations): every class `undetermined` by construction (no `local/`, no anchor, no expectation).
  Run B (SRC-authored client: identity spatial, SI, no state machine, hold, rate required): spatial and
  dimensional `undetermined` (nothing to decide against), temporal `undetermined` — state (none exposed) and
  stop behaviour (hold, no policy within 2 s, n = 5) satisfied, rate `undetermined` because the resting
  feedback noise (σ̂ ≈ 1–4 × 10⁻³ units) leaves fewer than five separable targets within the 0.1-unit excursion
  (`targets_not_separable`). Run C (dVRK-authored client: state machine required): temporal `divergent`
  because no operating-state topic is exposed and commands execute without one.
* Resolution floor 9–11 ms, response latency median 57–73 ms; internal command/measurement ratio
  1.12 [0.96, 1.28], 1.22 [0.99, 1.46], 0.85 [0.75, 0.95] (three runs, n = 10, noise-scaled steps): the
  interface tracks a step only approximately at this configuration; no run gives the unit (no anchor).

## What this run does not show

No unit scale (no independent SI anchor exists in the simulator's interface), no spatial binding relative
to the arm base (no `local/` topic), no execution rate (targets not separable at this noise level), and
nothing about the dVRK or the Raven-II.
