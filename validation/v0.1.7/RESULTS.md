# Results: confirmatory campaign of crtk-conformance 0.1.7

Pre-registration: `PREREGISTRATION.md` (commit `ed12de7`, before any run). Code: `src/` and `tests/` identical to `2fac51b` (Release 0.1.7). Runs: `mock/K`, `mock/L`; logs `mock/campaign_KL.log`, `mock/campaign_L_rerun.log`. Analysis: `python3 validation/analyze_v017.py` → `analysis/confirmatory_v017.json`; tables: `python3 validation/tables_v017.py`.

## Start-up failure

`L17_017_hold_none_00`, the first L run, found no node after three attempts (`NO LIVENESS OBSERVATION: missing topics`). As pre-registered, its record is kept in `mock/L_failed_startup/` and not pooled, and the run was repeated with its own seed (`--rerun L17_017_hold_none_00`; `mock/meta_L_rerun.json`).

## Predictions

| ID | Outcome | Observed |
|---|---|---|
| K-1 | matched | flag raised in 3/3 K1 runs |
| K-2 | matched | unit withheld (undetermined) in 3/3; ungated outcome divergent |
| K-3 | matched | frame (feedback-binding) conformant in 3/3 |
| K-4 | matched | command-semantic reading and dimensional summary undetermined, assumption contradicted, 3/3 |
| K-5 | matched | 6/6 controls: no flag; frame, unit, command-semantic conformant |
| **L-1** | **deviates** | 31/32 hold runs held through the range; `L17_017_hold_iid5_01` **not observable** |
| **L-2** | **deviates** | 31/32 satisfied; `L17_017_hold_iid5_01` **undetermined** |
| L-3 | matched | 32/32 fault intervals formed; all contain 250 ms |
| L-4 | matched | no contradictory interval |
| L-5 | matched | 32/32 stop sub-verdicts satisfied; none violated |
| L-6 | matched | median width without loss 328 ms |
| L-7 | matched | a conditional estimate with every formed interval |
| L-8 | matched | no formed interval decides a 245-ms horizon satisfied |

**Deviation (L-1, L-2).** In `L17_017_hold_iid5_01` (independent 5 % loss) the calibration answered 25 of 30 commands. The one-sided 95 % bound on the loss rate is then 31.9 %, which would need five confirmations per gap; the maximum is four, so any non-response in the run is unattributable (the rule of 0.1.6, unchanged). One post-gap command at the 2-s gap was lost. The stop class is therefore *not observable* and the hold verdict *undetermined*. This is an abstention, not a false verdict: no hold run was classified as a trip. The prediction had not allowed for a calibration that loses too many commands for confirmation. The chance of at least 5 losses in 30 at 5 % is about 1.6 % per run.

## Other results

- **K1 measurements** repeat the v0.1.6 campaign: goal residual 24.1 mm (lower 95 % bound ≥ 23.9 mm), internal ratio 4.99, anchored scale 5.52, ungated predicted error 452 mm; frame interval upper end ≤ 0.036 mm; control residuals 5.0–7.5 µm.
- **Unconfirmed non-responses** discarded in hold runs: 4 (iid 2 %: 1; iid 5 %: 2; GE: 1), as under 0.1.6.
- **Conditional (0.1.6-rule) estimates** of the 32 fault runs: 26 formed and containing, **2 excluding** 250 ms (`L17_017_fault250_ge5_02` [225.0, 240.4] ms and `_ge5_07` [232.6, 239.5] ms; both with a loss-free calibration, and both would decide a false 245-ms horizon *satisfied*), and 4 contradictory (`iid2_03`, `iid2_05`, `iid2_07`, `ge5_06`). The 0.1.7 decision intervals of the same runs all contain 250 ms.
- **Widths** (median, ms; 0.1.7 decision interval / conditional estimate): none 328 / 39; iid 2 % 328; iid 5 % 149; GE 270. Over all 32 runs the 0.1.7 median is 317 ms (range 80–334).
- Calibration saw loss in 9, 12 and 9 of 16 runs at iid 2 %, iid 5 % and GE 5 %.

# Offline sensitivity studies (not pre-registered; `studies/`)

- `boundary_sensitivity_v017.py` (seed 76000, 2000 replicates per cell, σ = 0.1 mm): the released frame and scale decisions under AR(1) correlation across trials (ρ = 0.5, 0.9), a constant bias of 0.5σ, a drift of span 2σ, and Student-t₃ errors.
  - Scale: false verdicts at most 2.8 % at ε in every model (iid Gaussian 2.7 %); correlation made it more conservative (AR 0.9: 0.35 %), because consecutive trials step along different axes with alternating signs.
  - Frame: no false verdict under bias, drift or t₃; AR 0.5: 0.15 % false divergence at ε; **AR 0.9: 11.1 % false divergence at ε** (95 % CI 9.7–12.5 %), 1.8 % at 0.95ε and 1.2 % false conformance at 1.05ε; coverage 0.82.
- `anchor_sensitivity_v017.py` (seed 77000, 500 replicates per level; simulated wrist, released functions; SI truth):
  - Coupling: up to 1 % of the step passes the gates with no bias above Monte Carlo noise; 2 % or more fails the axes-angle gate in every replicate (coupling tilts both estimated axes toward each other).
  - Axes angle: the common normal does not depend on it; 88° passes in 3 %, 85° in none.
  - Pose noise: 0.05 mm (with 5 mrad orientation noise) passes the gates in 17 % of replicates (mostly the axial-slide gate fails), 0.1 mm in none; the anchor then abstains.
  - Step: 0.1 rad passes in 66 %; 0.25 rad gives 89 % conformant at 5 mm; 0.5 rad 100 %.
  - Length mismatch passes every gate and biases λ̂ by the full mismatch: 9.0 vs 9.1 mm (1.1 %) is covered by u_L = 1.5 % (coverage 0.998); **3 % gives false divergence at 1 mm in 23 %** of replicates (coverage 0.13).
