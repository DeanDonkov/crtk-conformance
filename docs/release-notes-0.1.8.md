# crtk-conformance 0.1.8

Two probe changes on top of 0.1.7, made in response to an external review of the RC13 manuscript. Both change how
evidence is collected, not how an interval is decided; the 0.1.7 decision rules (consistency gate, sound fault bound)
remain the defaults.

1. **Frame probe: correlation guard.** The frame probe's ten trials were taken back to back, although its
   confidence region assumes independent trials; under correlated pose error the region narrows without the error
   narrowing. 0.1.8 first records a resting window of paired residuals (at least 3 s, extended until it spans 20
   integrated autocorrelation times, at most 30 s), estimates τ_int with Sokal's automatic window, spaces the trials by
   max(5, ⌈2τ_int⌉) samples, and withholds the verdict when τ_int is unresolved, when the spaced trials would exceed
   60 s, or when fewer than four effectively independent trials remain. A deterministic residual keeps back-to-back
   trials. `--no-correlation-guard` restores 0.1.7.
2. **Geometry anchor: joint-space method** (new default). Each trial steps every joint of the declared chain and fits a
   product-of-exponentials model to the measured joint changes and poses, so coupled or incomplete joint motion that
   `measured_js` reports no longer biases the wrist axes. Gates: largest fit residual (0.02 rad; 2 % of d_int), wrist
   excitation (≥ 0.05 rad), axes angle (90 ± 2°), at least three trials. `--anchor-method single_joint` restores 0.1.7.

Also: `crtk-mock` AR(1) position noise (`noise_model: ar1`, `ar_phi`); the archived campaign scripts pin their
procedures.

## Evidence (pre-registered: `validation/v0.1.8/PREREGISTRATION.md`, commit `79b3f71`; results `validation/v0.1.8/RESULTS.md`)

| Campaign | Runs | Outcome |
|---|---|---|
| F: reference node, AR(1) φ = 0.99 and iid noise, 0.1.7 vs 0.1.8 | 160 | 6/6 predictions matched. Back to back: 4/20 false divergent at ε under AR(1). Guard: 0/40 false, 23 withheld. iid: identical, back-to-back spacing kept |
| D: dVRK console 2.4.0, kinematic simulation | 27 | 2/2 matched: frame verdicts as 0.1.6 with deterministic plans; anchor d_int = 9.1000 mm, λ̂ = 1.000000 |
| S: SRC v1.0.0 and v2.0.0 on AMBF | 18 (+3 start-up failure, repeated) | S-1 deviates: on v1.0.0 the 0.1-m unit was detected (divergent, gates passed) in launch 1 and withheld in launches 2 and 3, where the wrist yaw chattered; no false verdict (exploratory: on the launch-1 poses the single-joint estimator also decides divergent in 2 of 3 runs). S-2 (v2.0.0: U at 1 mm, C at 5 mm, 9/9) and S-3 matched |

Offline studies (`validation/v0.1.8/studies/`, not pre-registered): the guard (≤ 0.2 % false verdicts in every cell;
back to back up to 22 %), the operating characteristic of the frame and scale decisions, the joint-space anchor's
operating range (pose noise ≤ 0.02 mm; dependence on correct joint readings), and the deadlines the sound fault bound
can decide (refutes ~18 ms below τ_w, confirms from ~τ_w + 0.3 s).

## Known issue

The joint-space fit calls `least_squares(method="lm", max_nfev=200)`. SciPy 1.10.1 (the campaign image) counts the
finite-difference Jacobian's evaluations in that budget, so the fit stops after about six iterations; newer SciPy (1.17.1
checked) converges. One unit test fails under SciPy 1.10.1 for this reason. Refitting every campaign trial to convergence
changed no gate outcome or verdict (`validation/poe_scipy_budget_v018.py`); at high reported coupling the early stop makes
the anchor abstain more often. A later version should bound iterations, not evaluations.

## Tests

- New: `tests/test_v018.py` (6, ROS-free) and one integration test of the guard under AR(1) noise.
- Full ROS suite at the release commit: `pytest_final_<commit>.log` in the RC14 package.

## Also

Everything in 0.1.7 (`docs/release-notes-0.1.7.md`) and 0.1.6 (`docs/release-notes-0.1.6.md`).
