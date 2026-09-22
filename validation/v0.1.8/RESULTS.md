# Results: campaigns of crtk-conformance 0.1.8 (F, D, S)

Pre-registration: `PREREGISTRATION.md` (commit `79b3f71`, before any run). Code: `src/` and `tests/` of `a52e89e` (version 0.1.8), unchanged since. Runs: `mock/F` (160), `dvrk_sim/D` (27), `src_live/live-src-v1` and `live-src-v2` (18 anchor runs, 6 resting traces). Campaign logs: `logs/campaign_v018_{F,D,S1,S2,S2_rerun_launch2}.log`. Analysis: `python3 validation/analyze_v018.py` → `analysis/campaigns_v018.json`; tables: `python3 validation/tables_v018.py <dir>`; descriptive trace analysis: `python3 validation/src_traces_v018.py` → `analysis/src_traces_v018.json`.

**Start-up failure.** SRC v2.0.0 launch 2: `launch_crtk_interface.py` exited at start-up (`ERROR! Requested Joint Idx ... outside valid range`, then a `TypeError` in `measured_jp`), so no CRTK topic existed; the trace recorded 0 samples and the three anchor runs report `measured_cp missing`. As pre-registered, the records are archived in `src_live/live-src-v2/failed_startup_launch2/` (not pooled) and the launch was repeated with `run_validation_v018.py S v2 --only-launch 2` (harness option added in `da00219`). No other failure occurred.

## Predictions

| ID | Outcome | Observed |
|---|---|---|
| F-1 | matched | iid noise: 20/20 conformant at 0.95ε under both procedures; 0 false verdicts at ε (all 20 undetermined) under both |
| F-2 | matched | φ = 0.99, 0.1.7 procedure, E/ε = 1.00: **4 false divergent** of 20 (8 determinate) |
| F-3 | matched | φ = 0.99, 0.1.8: 0 false verdicts in 40 |
| F-4 | matched | φ = 0.99, 0.1.8, E/ε = 1.00: 20/20 undetermined |
| F-5 | matched | iid, 0.1.8: planned spacing 5 samples in all 40 |
| F-6 | matched | φ = 0.99, 0.1.8: 23 withheld (τ_int unresolved within 30 s); the 17 others spaced by 62–275 samples |
| D-1 | matched | frame cases as v0.1.6 in 3/3 launches (F2 D, F4 C, F5a C, F5b C, F5c D, F5d D); deterministic residual, spacing 5, in all 18 |
| D-2 | matched | anchor gates passed; d_int = 9.1000 mm (within 1e-13 mm) and λ̂ = 1.000000 in all 9; SI 1 mm U, SI 5 mm C, mm 1 mm D |
| **S-1** | **deviates** | SRC v1.0.0: **3/9** as predicted (launch 1: gates passed, λ̂ = 0.1011 m/unit, divergent for both clients at 1 and 5 mm); launches 2 and 3: residual gates failed in all six runs → undetermined (the pre-registered contingency); λ̂ 0.096–0.101 |
| S-2 | matched | SRC v2.0.0: gates passed in 9/9 (residuals < 1e-5); d_int = 0.0090000 units, λ̂ = 1.0111 m/unit; SRC-authored 1 mm U, 5 mm C; dVRK-authored 1 mm U |
| S-3 | matched | resting traces of 2394–2402 `measured_cp` samples in all six pooled launches (v2.0.0 constant; v1.0.0 wrist-yaw chatter, see below) |

**Deviation (S-1).** In launches 2 and 3 of SRC v1.0.0 the joint-space fit left translation residuals of 8.4–9.1 % and 5.2–6.0 % of d_int (gate 2 %) in 4–5 of the 5 trials of every run; in launch 2 the rotation residual (up to 0.108 rad) and in two runs the axes angle also failed. No run gave a false verdict. The resting traces show that the wrist-yaw joint chatters, alternating sample to sample (lag-1 correlation −0.73 to −0.82), with standard deviation 0.038, 0.349 and 0.116 rad in launches 1–3; `measured_js` reports the chatter. The probe averages pose and joints over 0.3-s windows, and averaging a pose that swings about the yaw axis pulls the published point (10.6 mm beyond the axis) toward it by 10.6(1 − mean cos δ) mm: 0.007, 0.649 and 0.070 mm from the traces (0.1, 7.2 and 0.8 % of d_int). This accounts for launch 2 but not fully for launch 3, where the chatter during the steps may exceed that at rest; the cause was not isolated further.

**Exploratory, after the campaign: was the joint-space fit necessary?** `rederive_single_joint_S_v018.py` applies the single-joint estimator (unchanged) to the same recorded poses (+Δq wrist steps against the reference pose, commanded 0.25 rad, single-joint gates) → `analysis/single_joint_on_S_v018.json`. v1.0.0 launch 1: divergent in 2 of 3 runs (λ̂ 0.1019–0.1020); the third failed the step gate (a yaw step of 12.0° against 14.3°). Launch 1 tracked well (wrist steps 84–103 % of the command; other wrist joints ≤ 0.017 rad). Launches 2–3: gates failed in every run, λ̂ biased to 0.105–0.125 m/unit (joint-space: 0.096–0.101). v2.0.0: both estimators agree in 9/9. The launch-1 detection is therefore grounded, but it does not show that the joint-space fit was needed for it.

## Other results

- **F, unguarded outcomes of the 23 withheld runs:** 10 conformant, 1 divergent (at ε, false), 12 undetermined.
- **F, τ_int estimates** (true ≈ 199 samples): resolved runs 31–137, withheld runs 153–286. At this correlation the guard decides only when it underestimates τ_int.
- **Wall time:** F about 31 s per guarded φ = 0.99 run, 5 s per iid guarded run, 1 s per back-to-back run; D 5 s per frame case, 88 s per anchor case; S 155 s per anchor run.

## Known issue: the joint-space fit's budget depends on the SciPy version (found after the campaigns)

`poe_fit_trial` calls `least_squares(method="lm", max_nfev=200)`. SciPy 1.10.1 (the campaign image) counts the finite-difference Jacobian's residual evaluations in `max_nfev`, so the fit stops after about six iterations (status 0); SciPy 1.17.1 counts residual evaluations only and converges. The final ROS test run in the image therefore fails `test_poe_fit_is_unbiased_under_coupled_and_incomplete_joint_motion` (150 passed, 1 failed). `poe_scipy_budget_v018.py` (exploratory) → `analysis/poe_scipy_budget_{container,host}.json`: 81 of the 135 D and S fits stopped at the budget in the image; refitted under SciPy 1.17.1 (none stopped) every run keeps its gate outcome and verdict, and λ̂ agrees to 7e-8 where the gates passed. On the anchor study's coupling levels the early stop lowers the gate-pass rate at 10 % coupling from 89 % to 76 % (no false verdict). `src/` is not changed (frozen by the pre-registration).

# Offline studies (not pre-registered; `studies/`)

- `frame_guard_study_v018.py` (seed 78000, 1000 replicates per cell): back to back, false divergence at ε grows with trial correlation, to 20.3 % (σ 0.02 mm) and 22.4 % (0.1 mm) at φ = 0.99; with the guard ≤ 0.2 % in every cell and 0 at 0.95ε and 1.05ε; withheld 37–41 % at φ = 0.99, σ 0.02 mm.
- `operating_characteristic_v018.py` (seed 80000): decision rate and error among determinate verdicts of the released frame and scale decisions over σ 0.02–0.2 mm, 5–40 trials, E/ε 0.5–2. Frame: no false verdict in any cell. Scale: 2.2 % / 3.3 % false divergence at ε (σ 0.02 / 0.1 mm, 9 trials); at σ 0.1 mm ≤ 13 % decided within 10 % of ε and 13–21 % of determinate verdicts false at 5 % from ε.
- `anchor_poe_study_v018.py` (seed 79000, 100 replicates per level): pose noise ≤ 0.02 mm passes the gates, 0.05 mm 5 %, 0.1 mm 0 %; reported 44 % yaw reach unchanged; reported coupling 5/10 % fails the residual gate in 8/14 %; a 2 % wrist joint-reading scale error biases λ̂ by −2.0 % and passes; 5 % gives false divergence at 1 mm in every replicate; a 3 % length mismatch gives false divergence at 1 mm in 60 %.
- `deadline_resolution_v018.py`: the sound fault bound refutes horizons more than 18 ms (median) below τ_w and confirms them from τ_w + 310 ms without loss (125 / 249 ms under iid / GE 5 % loss); the conditional estimate from τ_w + 21 ms; archived records 0.51–0.52 s; drift stops ≤ 36 ms band. Replay of `fault_horizon_decision` consistent in all 32 confirmatory runs.
