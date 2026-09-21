# crtk-conformance 0.1.6

Client-relative black-box conformance probes for CRTK/ROS 1 interfaces, with the reference node `crtk-mock`.
This release carries the decision rules and the evidence of the RC9/RC10 manuscript
*Structural Conformance Is Not Semantic Conformance*. The full list of changes is in `CHANGELOG.md`.

## What is new since 0.1.5

- **Instrument-geometry unit anchor** (`--probes geometry`, expectation mode `dimensional.mode: geometry_anchor`).
  It grounds the interface unit in a known link length: the common normal of the wrist-pitch and wrist-yaw screw axes,
  compared with the declared L and its relative uncertainty u_rel. Gates on the step angle, the axial slide and the
  angle between the axes return undetermined when the geometry does not behave as declared.
- **Liveness under command loss.** The calibration sends 30 commands (was 12). A non-response without a state change
  counts as a stop policy only after it recurs r = max(2, ceil(ln α / ln p_up)) times at the same gap, where p_up is the
  one-sided 95 % Clopper–Pearson loss bound (α = 0.01, r ≤ 4). The 0.1.5 rule stays selectable
  (`--liveness-rule 0.1.5 --calibration-commands 12`).
- **State commands are sent one at a time** (`ensure_enabled`). The released dVRK console's ROS 1 bridge keeps only
  the latest state command, so 0.1.5's back-to-back `enable` + `home` never re-enabled a disabled arm.
- **Command/feedback consistency diagnostic** in the scale probe. It flags a command binding that differs from the
  feedback binding, or a tracking deficit. It is not a verdict.
- The frame probe skips samples with an unset (zero) header stamp; probes of the same class are combined in the report.
- The reference node gains a separate command binding, mixture noise and Gilbert–Elliott loss. All default to the
  archived behaviour, and a fixture test checks every preset.
- `--skip-rate-sweep`, `--enable-timeout-s`; `dimensional.scale_decision` (the eq. (6) decision, moved unchanged).

## Evidence in this release (`validation/v0.1.6/`)

The pre-registration and its addenda A–D are in `validation/v0.1.6/PREREGISTRATION.md`, each committed before the runs
it concerns.

- **dVRK 2.4.0 console in kinematic simulation**, built from source and run headless, no hardware: 51/51 pre-registered
  runs matched their predictions.
- **SRC v2.0.0** with the geometry anchor ran as predicted. On **SRC v1.0.0** the anchor gates failed, so its verdict
  was undetermined instead of the predicted divergent.
- **K1**: correct feedback with a wrong command binding.
- **Boundary**:
  - offline Monte Carlo, N = 2000 per cell;
  - live confirmation, N = 30 per cell;
  - Fig. 1(d) re-scored with exact decimal truth.
- **Command loss**: 128 runs under rules 0.1.5 and 0.1.6.
- **The archived v0.1.3 intervals** re-derived under 0.1.6: unchanged.

`validation/v0.1.3` and the 0.1.5 verification campaign are byte-identical to v0.1.5.

**RC10 reporting correction (no run repeated, no verdict changed).** The live boundary analysis had counted a
legitimate 0.0 lower end of the scale interval as not covering the truth. The live scale coverage is 30/30, 28/30 and
30/30 (0.93–1.00), not 0.83–0.90. `validation/verify_reporting_v016.py` now recomputes it from the raw records.

## Known limitations

- **Two timing tests fail intermittently** on a 2-vCPU host under pytest, with the 0.1.5 code as well:
  - `test_rate_accepted_channel_satisfied_on_reference`
  - `test_fifo_queue_slower_than_the_client_is_violated_and_logged` (its archival v0.1.3 rate-verdict assertion)

  See `validation/v0.1.6/tests/README.md`.
- **The fault-policy τ_w interval** assumes that the post-gap command of every faulted trial arrived. If that command
  is lost and the fault fires during the response wait while calibration saw no loss, the interval can exclude the
  true timeout (1 of 16 runs at 5 % loss). A post hoc, state-bounded variant is sound but about 7× wider
  (`validation/reanalyze_loss_v016.py`). Fault confirmation by repetition is not implemented.
- **No physical robot was measured.** The dVRK results are "released dVRK software in kinematic-simulation mode".

## Reproduce

See `README.md` (section *Validate against the mock*) and `validation/v0.1.6/environment/`: container recipes from the Ubuntu archive
and GitHub sources. The paper's numbers are checked from the raw archive by:

```
python3 validation/verify_reporting_v016.py --paper <manuscript directory> --tag rc10
```
