# Pre-registration: confirmatory campaign of crtk-conformance 0.1.7

**Status.** Written and committed before any run of this campaign. The commit that adds this file is the pre-registration commit. It also adds the campaign script `validation/run_validation_v017.py` and the analysis script `validation/analyze_v017.py`; results are committed afterwards.

**Why.** The two 0.1.7 decision rules were introduced after the pre-registered v0.1.6 campaigns, in response to an external review, and were first only re-derived offline (`validation/rederive_v017.py`):

1. the **consistency gate**: a raised command/feedback consistency flag withholds the unit verdict and the command-semantic reading of the frame verdict;
2. the **sound fault bound**: each faulted trial bounds τ_w by the time its FAULT was observed.

A second external review asked that one software version be the evaluated system. This campaign therefore re-runs, with the frozen 0.1.7 code and fresh seeds, the two experiments whose outcomes those rules can change.

**Frozen code.** `src/` and `tests/` are identical to commit `2fac51b` ("Release 0.1.7"), version 0.1.7, default rules: liveness rule `0.1.7`, consistency gate on. Full ROS test suite at `2fac51b`: 144 passed. Nothing in `src/` changes between this pre-registration and the results.

**Not re-run, and why.** The 0.1.7 rules change a verdict only when:
- the consistency flag is raised (only the scale probe computes it); or
- a trial is classified faulted.

The offline re-derivation found no verdict change outside K1 and the fault-policy runs of the loss experiment. The following are therefore reported as executed (0.1.6.dev0 code), with their verdicts unchanged under 0.1.7:
- the dVRK-sim campaign: no flag, and no faulted trial (the console held);
- the SRC runs: no scale probe, and no faulted trial;
- the boundary campaign: no flag in 90 scale runs; the frame probe is unaffected;
- the archived v0.1.3 campaign.

## 1. Design

Environment: the campaign image `focal-rc9:live` of the v0.1.6 campaigns (ROS Noetic from source, Python 3.8), private ROS 1 master on port 11417, reference node `crtk-mock`. Task parameters as before: ε = 1 mm, r_ws = 0.10 m, v = 50 mm/s, client 100 Hz, J_max = 5 ms.

### 1.1 K (WP4 under 0.1.7): 9 runs

- The same cases as the v0.1.6 WP4:
  - **K1 × 3**: feedback binding JHU, declared correctly; command binding JHU·E, with E = 20 mm and 10°;
  - **identity controls × 3**;
  - **JHU shared-binding controls × 3**.
- Probe settings are those of v0.1.6: frame 10 trials × 5 samples; scale 9 trials, step 0.005, settle 0.6 s.
- The consistency gate is on (the 0.1.7 default), and each run archives the report-level `summary` and `verdict_kinds`.
- Seeds: 70010–70012 (K1), 70020–70022, 70030–70032.

### 1.2 L (WP6 under 0.1.7): 64 runs

- Rule `0.1.7`, 30 calibration commands.
- Policies:
  - **hold**: the node holds; expectation `hold`, horizon 2 s;
  - **fault250**: FAULT after 250 ms of silence; expectation `fault`, default horizon.
- Loss: none; independent 2 % and 5 %; Gilbert–Elliott (p_gb = 0.0105, p_bg = 0.2; stationary 5 %, mean burst 5).
- N = 8 per cell. Stop sub-probe: 5 trials, largest gap 2 s, step 0.002, rate sweep skipped.
- Seeds 90000 + k, with k running over the 64 runs in order (hold before fault; none, iid2, iid5, ge5; j = 0…7).

**Start-up failures.** A run whose node is not discovered after the probe's retries is re-run with its own seed (`--rerun`). It is logged and not pooled.

## 2. Predictions (primary)

### K

| ID | Prediction |
|---|---|
| K-1 | K1: consistency flag raised in 3/3. |
| K-2 | K1: unit verdict undetermined (withheld) in 3/3, with ungated outcome divergent. |
| K-3 | K1: frame (feedback-binding) verdict conformant in 3/3. |
| K-4 | K1: report `spatial_command_semantic` undetermined, `summary.dimensional` undetermined, and `shared_binding_assumption` contradicted, in 3/3. |
| K-5 | Controls: no flag; frame, unit and command-semantic reading conformant, in 6/6. |

### L

| ID | Prediction |
|---|---|
| L-1 | Hold: held through the range in 32/32 (no false trip class). |
| L-2 | Hold: stop sub-verdict satisfied in 32/32. |
| L-3 | Fault: no formed interval excludes 250 ms. |
| L-4 | Fault: no contradictory interval. |
| L-5 | Fault: stop sub-verdict satisfied in every run with a formed interval; violated in none. |
| L-6 | Fault: median width without loss between 300 and 400 ms (the offline re-derivation gave 334 ms). |
| L-7 | Fault: a conditional (0.1.6) estimate is reported with every formed interval. |
| L-8 | Secondary (analysis only): at a declared horizon of 245 ms, which is false for τ_w = 250 ms, no formed 0.1.7 interval decides "satisfied". |

## 3. Deviations

Any prediction not met is reported as a deviation, with its mechanism if it can be identified. No prediction, parameter or case is changed after the first run.

## 4. Analysis

`python3 validation/analyze_v017.py` reads `validation/v0.1.7/mock/{K,L}`, prints every prediction with its outcome, and writes `validation/v0.1.7/analysis/confirmatory_v017.json`.
