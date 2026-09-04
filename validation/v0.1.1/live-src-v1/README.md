# Live run against the released Surgical Robotics Challenge v1.0.0 CRTK interface (reported run)

This is the run reported in the manuscript (RC3, Section 7.3, [M-live]) for SRC v1.0.0. Nothing in this
directory was edited after the run except this README. Earlier attempts are kept beside it, each with a README:
`../live-src-v1-run{1..5,7}-preliminary/`, `../live-src-v1-run6-home-pose/`, `../live-src-v1-run{8,9}-superseded/`.

| Item | Value |
|---|---|
| Target | `surgical_robotics_challenge` v1.0.0 = 158b554e1a3e7cd618eadedf0505865891e25f27, unmodified |
| Simulator | AMBF branch `ambf-2.0` @ 16a8151816407dfedff6785748ae72323c280f67, headless (`-g false`, Xvfb), launch args in `logs/run.log` |
| Probe | crtk-conformance e546af16297ce5c83a7a916b00e4b3c7ac712c62 (v0.1.1) |
| Environment | `focal-crtk:rc3` (see `../../environment/`), Python 3.8.10, ROS Noetic built from source; `meta_env.json` |
| Arm | PSM1 only (`--two False --ecm False --scene False`), moved to q = (0, 0, 1.0, 0, 0, 0) interface units (40 % insertion) by `live_aux.py` before probing |
| Commands | `../../environment/run_live.sh v1 <this dir>`; three probe runs A/B/C with the options listed in `logs/run_*.log` and the report `parameters` |

## What was observed (all numbers are in `live_aux.json` and the three `report_*.json`)

* Topics under `/CRTK/psm1/`: `measured_cp`, `servo_cp`, `measured_js`, `servo_jp`, `T_b_w`, `measured_cv`
  present; `local/measured_cp`, `setpoint_cp`, `local/setpoint_cp`, `servo_cr`, `operating_state`,
  `state_command` absent; no `/tf`. `measured_cp.header.frame_id = psm1/baselink`; `T_b_w.header.frame_id = world`.
* `measured_cp` published at ≈ 119.3 Hz.
* Joint-error de-perturbation (Section 4.4): the interface printed joint errors (−0.02668, 0.07286, −0.02691, 0, 0, 0)
  (`logs/launch_crtk_interface.log`); `measured_js` minus the simulator's own joint state = (0.02668, −0.07286,
  0.02691, 0, 0, 0.0002): the negatives to 1e-5 on the masked joints.
* ‖measured_cp‖ at the working configuration = 0.9664 interface units (own FK of the reported joints 0.9673; see
  `../CROSS_VERSION_FK_NOTE.md`).
* Run A (no expectations): every class `undetermined` by construction. Run B (SRC-authored client): temporal
  `undetermined` — state (none exposed) and stop behaviour (hold; no policy within 2 s, n = 5) satisfied, rate
  `undetermined` (`targets_not_separable`: resting noise σ̂ ≈ 3–7 × 10⁻³ units leaves fewer than five separable
  targets in the 0.1-unit excursion). Run C (dVRK-authored client, state machine required): temporal `divergent`.
* Resolution floor 9.0–9.7 ms; response latency median 59–87 ms; internal command/measurement ratio
  0.99 [0.92, 1.06], 1.20 [0.98, 1.42], 0.92 [0.69, 1.16] (three runs, n = 10, noise-scaled streamed steps).

## What this run does not show

No unit scale (no independent SI anchor), no spatial binding relative to the arm base (no `local/`), no execution
rate (targets not separable at this noise level), and nothing about the dVRK or the Raven-II.
