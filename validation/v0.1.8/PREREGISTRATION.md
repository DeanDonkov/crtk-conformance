# Pre-registration: campaigns of crtk-conformance 0.1.8 (RC14)

**Status.** Written and committed before any run of the campaigns F, D and S below. The commit that adds this file is the pre-registration commit. It also adds the campaign script `validation/run_validation_v018.py`; the analysis script `validation/analyze_v018.py` is committed with it as well. Results are committed afterwards.

**Why.** An external review of the RC13 manuscript asked for:
1. a defensible sampling procedure for the frame probe, whose trials were taken back to back although correlated trial errors raise its false-decision rate (offline: 11 % false divergence at the boundary for a trial correlation of 0.9);
2. at least one detection of a real semantic mismatch under imperfect tracking, with ground truth independent of the probe. On SRC v1.0.0, the released interface with a 0.1-m unit (its own changelog), the single-joint anchor abstained, because its wrist steps also moved other joints and the yaw step fell short.

Version 0.1.8 (commit `a52e89e`) answers with two probe changes: the frame probe's **correlation guard** and the **joint-space (product-of-exponentials) anchor** (`CHANGELOG.md`). Both were designed after the RC13 failures had been seen. The campaigns below test them on fresh runs.

**Frozen code.** `src/` and `tests/` of commit `a52e89e` (version 0.1.8), default rules: liveness rule 0.1.7, consistency gate on, correlation guard on, anchor method `poe`. Nothing in `src/` changes between this pre-registration and the results.

**Exploratory, not pooled.** Before this pre-registration a feasibility smoke run of the harness on the dVRK console (one frame case, one anchor case) checked that the new code runs through ROS (`validation/v0.1.8/dvrk_sim/exploratory-smoke/`). The offline studies `frame_guard_study_v018.py` and `anchor_poe_study_v018.py` (not pre-registered) characterize the two changes and informed the predictions of F. No run on SRC with 0.1.8 took place before this commit.

## 1. Design

Task parameters as before: ε = 1 mm (unless stated), r_ws = 0.10 m, v = 50 mm/s, client 100 Hz, J_max = 5 ms.

### F: reference node, frame probe under correlated noise (160 runs)

- Preset `reference`, identity declaration, injected translation E along x with E/ε ∈ {0.95, 1.00}; closed-boundary truth.
- Position noise 0.02 mm per channel, either iid Gaussian (φ = 0) or AR(1) per publish step (`noise_model: ar1`, φ = 0.99 at 100 Hz, τ_int ≈ 199 samples).
- Two procedures on each condition: **0.1.7** (`correlation_guard=False`: ten back-to-back trials of five samples) and **0.1.8** (the guard with its defaults).
- N = 20 runs per cell (2 φ × 2 E × 2 procedures); seeds 81000 + k in run order; private ROS master on port 11418.

### D: released dVRK console 2.4.0 in kinematic simulation, controls (27 runs)

- The v0.1.6 `jhu` configuration and declarations, unchanged (`validation/v0.1.6/dvrk_sim/configs`, `expectations`); three launches; bring-up before every case as in v0.1.6.
- Nine cases, with the 0.1.8 defaults:
  - frame cases `F2_identity`, `F4_0874`, `F5a_095`, `F5b_098`, `F5c_102`, `F5d_105`;
  - anchor cases `U_si` at 1 mm and 5 mm, `U_mm` at 1 mm.

### S: released SRC v1.0.0 and v2.0.0 on AMBF (18 anchor runs)

- The v0.1.6 SRC stack and launch sequence (`src_live_v016.Stack`), no source modified.
- Three independent launches per release. SRC draws random joint errors at start-up, so launches differ.
- In each launch:
  - a 20-s resting trace of `measured_cp` and `measured_js` at the geometry reference configuration;
  - the joint-space anchor (5 trials, 2.0-s settle) for three declarations: SRC-authored at 1 mm and at 5 mm, and dVRK-authored at 1 mm. These are the v0.1.6 declarations, with `anchor_method: poe` and `joint_types: RRPRRR` made explicit.
- L = 9.1 mm and u_L = 1.5 %, as before.

**Start-up failures.** A launch or run whose stack or node does not come up is repeated. It is logged and not pooled.

## 2. Predictions (primary)

### F

| ID | Prediction |
|---|---|
| F-1 | iid noise, both procedures: E/ε = 0.95 conformant in ≥ 19/20 runs each; E/ε = 1.00 no false verdict in 20/20 each. |
| F-2 | φ = 0.99, 0.1.7 procedure, E/ε = 1.00: at least one false divergence in 20 runs (offline 20 %). |
| F-3 | φ = 0.99, 0.1.8: no false verdict in its 40 runs. |
| F-4 | φ = 0.99, 0.1.8, E/ε = 1.00: undetermined in ≥ 18/20 runs. |
| F-5 | iid noise, 0.1.8: the planned trial spacing is 5 or 6 samples in all 40 runs. |
| F-6 | φ = 0.99, 0.1.8: every run either spaces its trials by ≥ 20 samples (four times the back-to-back spacing) or withholds its verdict. |

### D

| ID | Prediction |
|---|---|
| D-1 | Frame, 3/3 launches each: F2 D, F4 C, F5a C, F5b C, F5c D, F5d D (as v0.1.6); the correlation plan reports a noise-free residual (spacing 5) in all 18 runs. |
| D-2 | Anchor, 3/3 launches each: gates pass; d_int = 9.100 mm within 0.001 mm; U_si 1 mm U, U_si 5 mm C, U_mm 1 mm D. |

### S

| ID | Prediction |
|---|---|
| S-1 | **SRC v1.0.0 (detection).** In 9/9 runs the gates pass, λ̂ lies in [0.0995, 0.1027] m per interface unit (0.1011 expected from the SRC model's 9.0 mm), and the unit verdict is divergent: SRC-authored at 1 and 5 mm, dVRK-authored at 1 mm. |
| S-2 | **SRC v2.0.0 (control).** In 9/9 runs the gates pass, λ̂ lies in [1.000, 1.022] m per unit, and the verdicts are SRC-authored U at 1 mm and C at 5 mm, dVRK-authored U at 1 mm. |
| S-3 | Secondary: each launch records a resting trace with ≥ 1000 measured_cp samples. |

**Contingency.** A run whose anchor gate fails is undetermined. It counts as not matching its primary prediction and is reported with its failed gates.

## 3. Deviations

Any prediction not met is reported as a deviation, with its mechanism if it can be identified. No prediction, parameter or case is changed after the first run.

## 4. Analysis

`python3 validation/analyze_v018.py` reads `validation/v0.1.8/{mock/F, dvrk_sim/D, src_live}`, prints every prediction with its outcome, and writes `validation/v0.1.8/analysis/campaigns_v018.json`.
