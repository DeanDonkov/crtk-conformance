# validation/v0.1.6: the RC9 archive (runs made with crtk-conformance 0.1.6.dev0; released as 0.1.6, whose probe and decision code are unchanged)

All runs here were planned in `PREREGISTRATION.md` (committed at `efdd062`) and its addenda A–D, each committed before the runs it affects. Exploratory runs are kept apart and are never pooled with results.

| Path | Content |
|---|---|
| `PREREGISTRATION.md` | Cases, parameters, declarations and predicted outcomes; addenda A–D |
| `environment/` | Build chain from the Ubuntu archive and GitHub sources (no Docker Hub, no packages.ros.org), image ids, wheel hashes, run helpers |
| `dvrk_sim/` | Released dVRK 2.4.0 console in kinematic simulation. Configs and declarations derived from `dvrk_config_jhu` by script; `derived_truths.json`. <br>`runs/`: pre-registered campaign at `99583e4` (jhu launches 1–3, nobase launches 2–3) and the addendum-C launches (jhu 4–6 for `T_fault025`, nobase 4). The launch 1–3 `T_fault025` logs are the crash (no report). `nobase_launch1` is a failed bring-up. <br>`runs-failed-bringup/`: the first attempt, before addendum A. <br>`smoke/`: WP1, no probe. <br>`harness_diag/`: exploratory, no probe. |
| `src_live/` | SRC v1.0.0/v2.0.0 on AMBF with the geometry anchor (`live-src-v1`, `live-src-v2`). <br>`live-src-v1-run1-failed-startup/`: before addendum D. <br>`exploratory-geometry-diag-v*`: run after the pre-registered runs, not a result. |
| `mock/K` | WP4, case K1 and shared-binding controls |
| `mock/B` | WP5 live boundary confirmation, 360 runs. A host restart interrupted the campaign after 302 runs; the other 58 were run with their own seeds via `--rerun`. See `KB_stdout_part1_interrupted.log` and `B_stdout_part2_resumed.log`. |
| `mock/L` | WP6 loss experiment, 128 runs. The first run was lost to a start-up failure and re-run with its seed: `L_attempt1_stdout.log`, `L_stdout.log`, `meta_L_rerun.json`. |
| `boundary/` | Offline Monte Carlo of the released decision rules, N = 2000 per cell |
| `reanalysis/` | Archived v0.1.3 liveness intervals under the 0.1.6 rule; width/τ_w |
| `rescoring/` | Fig. 1(d) sweeps with exact decimal truth, including ε = 1 mm; mixture decisions |
| `analysis/` | `analyze_v016.py` outputs, and `L_state_bounded_posthoc.json` (**post hoc**, not pre-registered). `B_live_boundary.csv` was regenerated for RC10: its coverage column had turned legitimate 0.0 lower interval ends into non-coverage (scale 27/25/27 → 30/28/30 of 30); counts and verdicts are unchanged |
| `figures/` | `fig05_validation` (0.1.6) |
| `tests/` | pytest logs; the two intermittent timing tests |

Scripts are in `validation/`: `dvrk_sim_v016.py`, `src_live_v016.py`, `run_validation_v016.py`, `boundary_montecarlo_v016.py`, `reanalyze_liveness_v016.py`, `rescore_v016.py`, `analyze_v016.py`, `reanalyze_loss_v016.py`, `tables_v016.py`, `fig_v016.py` and `verify_reporting_v016.py`.
