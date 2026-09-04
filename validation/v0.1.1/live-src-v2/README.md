# Live run against the released Surgical Robotics Challenge v2.0.0 CRTK interface (reported run)

This is the run reported in the manuscript (RC3, Section 7, [M-live]) for SRC v2.0.0. Nothing in this
directory was edited after the run; the earlier attempts are kept as `../live-src-v2-run1-home-pose/`
(probing at the singular home pose), `../live-src-v2-run2-preliminary/` (auxiliary script subscribed to the
v1.0.0 body name) and `../live-src-v2-run3-failed-startup/` (start-up race in the released interface).

| Item | Value |
|---|---|
| Target | `surgical_robotics_challenge` v2.0.0 = 03befbf1028d22b0a6495059af51e397646570cf, unmodified |
| Simulator | AMBF branch `ambf-2.0` @ 16a8151816407dfedff6785748ae72323c280f67, headless (`-g false`, Xvfb), launch args in `logs/run.log` (the release's own: `-l 0,1,6,7,8,9 -p 200 -t 1 --override_max_comm_freq 100 --override_min_comm_freq 100`) |
| Probe | crtk-conformance d8ef24cb6de340e8c949abb549b588e941f518ad (v0.1.1) |
| Environment | `focal-crtk:rc3` (see `../../environment/`), Python 3.8.10, ROS Noetic built from source; `meta_env.json` |
| Arm | PSM1 only (`--two False --ecm False --scene False`), moved to q = (0, 0, 0.1, 0, 0, 0) (metres; 40 % insertion) by `live_aux.py` before probing |
| Commands | `../../environment/run_live.sh v2 <this dir>`; three probe runs A/B/C with the options listed in `logs/run_*.log` and the report `parameters` |

## What was observed (all numbers are in `live_aux.json` and the three `report_*.json`)

* Same topic inventory as v1.0.0: `measured_cp`, `servo_cp`, `measured_js`, `servo_jp`, `T_b_w`, `measured_cv`
  present; `local/measured_cp`, `setpoint_cp`, `local/setpoint_cp`, `servo_cr`, `operating_state`,
  `state_command` absent; no `/tf`. `measured_cp.header.frame_id = psm1/baselink`; `T_b_w.header.frame_id = world`
  (T_b_w translation (0.130, 0.393, 0.873) in this scene versus (1.0, 1.346, 1.350) in the v1.0.0 scene).
* `measured_cp` published at ≈ 118.9 Hz (inter-arrival mean 8.4 ms).
* Joint-error de-perturbation (Section 4.4): interface-printed joint errors (0.000614, −0.045228, −0.001544, 0, 0, 0)
  (`logs/launch_crtk_interface.log`); `measured_js` minus the simulator's own joint state
  (`/ambf/env/psm1/baselinksimple/State`) = (−0.00061, 0.04523, 0.00154, 0, 0, 0): the negatives, to 1e-5.
* ‖measured_cp‖ at q = (0, 0, 0.1) = 0.0861 m, equal to the release's own FK; see `../CROSS_VERSION_FK_NOTE.md`
  for the comparison with v1.0.0 (0.967 interface units) and why the ratio is 11.23 rather than 10.
* Resting feedback noise σ̂ ≈ 2–5 × 10⁻⁵ m (v1.0.0: 1–4 × 10⁻³ units); a 10 mm `servo_cp` step settles with a
  residual of 1 µm; internal command/measurement ratio 0.9994 [0.9988, 1.0000], 0.9996 [0.9991, 1.0000],
  0.9995 [0.9989, 1.0002] (three runs, n = 10, 5 mm steps): the interface tracks a step to within 0.1 %.
  No unit is decided (no anchor).
* Run A (no expectations): every class `undetermined` by construction. Run B (SRC-authored client): temporal
  `undetermined` — state (none exposed) and stop behaviour (hold; no policy within 2 s, n = 5, drift ≤ 0.24 mm
  during the gap) satisfied, rate `undetermined` because the feedback trajectory did not pass within
  δ = 5σ̂ of the commanded targets (unmatched fraction 0.97–1.00 at 50–500 Hz): the release interpolates
  towards each setpoint with a median response latency of 86–97 ms, so distinct commands cannot be counted
  through `measured_cp`. Run C (dVRK-authored client, state machine required): temporal `divergent`.
* Resolution floor 9–13 ms; response latency median 86–97 ms (p95 125–128 ms).

## What this run does not show

No unit scale, no spatial binding relative to the arm base, no execution rate, nothing about the dVRK or the
Raven-II, and nothing about the v2.0.0 interface under other launch options (the release's `-p 200` physics
rate and comm-frequency overrides were used unchanged).
