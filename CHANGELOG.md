# Changelog

## 0.1.2 — 2026-09-05 (branch `rc4-decision-semantics`)

Response to the adversarial review of the RC3 manuscript (`preprint/rc4/RC3_ADVERSARIAL_REVIEW.md`; the
reviewer's counterexample script is kept verbatim under `tests/adversarial/`). Executable behaviour changed
in the three decision procedures; the v0.1.0 and v0.1.1 validation archives are unchanged and a new archive
was produced under `validation/v0.1.2/`.

### Decision semantics
- **Spatial** (`spatial.py`, `probes/frame.py`; review findings 1 and 4). The verdict is taken on the *exact*
  maximum positional error of the residual binding over the command ball, eq. (3')
  `e_max = sqrt(t_par^2 + (t_perp + 2 sin(theta/2) r_ws)^2)`, not on the eq. (3) upper bound (which 0.1.1
  used as if exceeding it proved a violation; the reviewer's case t_par = 0.7 mm, theta = 0.3 deg, r_ws = 0.1 m
  is now conformant at 1 mm, as it should be). Its uncertainty is propagated from Hotelling T^2 confidence
  regions of the translation and rotation-vector residuals (Bonferroni, Lipschitz bounds) instead of a
  re-centred Student-t half-width of per-trial norms (~45 % coverage at zero residual). The interval is
  conservative; its coverage is measured (`tests/test_spatial.py`, and the v0.1.2 campaign). Fewer than four
  trials is `undetermined` by construction. The verdict is positional; `spatial.orientation_tolerance_deg`
  adds a separate orientation verdict. The translation of a declared expected transform is interpreted in
  the unit the client declares (`dimensional.expected_unit_m`, recorded as an assumption in the report).
- **Rate** (`rate_estimator.py`, `probes/rate.py`; finding 2). Feedback target crossings are no longer
  evidence of executed commands (a controller that executes only the final command produces every crossing).
  The rate expectation is decided on an *accepted-command channel*: the implementation's `setpoint_cp`, with
  the criterion that the longest stale interval between accepted commands does not exceed the required
  period (a mean rate does not bound the stale interval). Without `setpoint_cp` the expectation is
  `undetermined` (both SRC releases). The crossings statistic is kept as a diagnostic
  (`feedback_target_crossings_hz`, `feedback_count_is_evidence_of_execution: false`).
- **Liveness** (`probes/rate.py`; findings 3 and 6). The timeout is reported as an interval built from the
  per-trial bounds under a deterministic-timeout model with explicit allowances for transport latency (L),
  the implementation's evaluation granularity (G, one feedback period, an assumption stated in the report) and
  the detection delay of a drift (hold tolerance / drift speed); drift onsets and speeds are estimated inside
  the gap (no fixed 0.2 s window); an inconsistent set of bounds is reported as `inconsistent`. Eq. (7)
  margins are decided against the interval (`undetermined` when the client's period + J_max lies inside it).
  Stop classes are observational: `held`, `drifted`, `rejected`, `faulted`, `not_observable`, and
  `held_through_range` for the whole probe; "no liveness policy" is not a class. A `hold` expectation is
  claimed up to `temporal.horizon_s` (default: the tested range) and is `undetermined` beyond what was tested;
  a `release` expectation is `undetermined` from the pose alone; `drift` is a declarable expectation.
- **Mock** (`crtk_mock`): publishes `setpoint_cp` (last accepted goal) and can accept every k-th command
  (`accept_every_k`); the SRC and AMBF-watchdog emulations do not publish `setpoint_cp` (live inventories).

### Packaging
- `MockConfig` moved to the ROS-free `crtk_mock/config.py`; `crtk_mock` imports the node lazily, so analysis
  scripts read preset parameters without `rospy`.
- Version 0.1.2. `validation/run_validation_v012.py` / `analyze_v012.py` (labels derived from eq. (3'), spatial
  interval coverage, accepted-channel rate cases, liveness interval coverage against injected timeouts,
  Fig. 5(d) rate labels: FPR = false divergent, FNR = false conformant). `analyze_v011.py` is kept as it was
  used for the v0.1.1 archive (its Fig. 5(d) legend had the two labels swapped; corrected in v012).

## 0.1.1 — 2026-09-03 (branch `rc3-major-revision`)

Response to the external review of the RC2 manuscript (Reviewer #2, M3–M6 and minor issues). Executable
behaviour changed; the v0.1.0 validation archive is kept unchanged under `validation/archive-v0.1.0/` and a
new archive was produced under `validation/v0.1.1/` (mock campaign plus live runs against the released
Surgical Robotics Challenge software).

### Semantics
- **Client expectations** (`expectations.py`, `--expectations expectations.yaml`). `conformant` now means that
  the discovered binding satisfies an *explicitly declared* client expectation within the stated tolerance;
  with no declared expectation a probe reports its observations and returns `undetermined`. Spatial:
  `identity | expected_transform | discover_only`; dimensional: `si` (with the unit the client assumes) |
  `discover_only`; temporal: `state_machine required|forbidden|any`, `stop_behaviour hold|release|fault|any`,
  `rate required|any`. The old `--expect-state-machine yes|no|any` is kept as an alias.
- The spatial decision is taken on the residual `T_hat * T_expected^-1` (M3); the dimensional decision on
  `s_hat / expected_unit`; the temporal outcome combines sub-verdicts per declared expectation.
- `decide()` treats interval ends within a 1e-9 relative guard band of the tolerance as boundary ties
  (`undetermined`) so that floating-point representation cannot decide a case (minor 5).

### Probes
- **Observable-rate estimator rewritten** (`rate_estimator.py`, design `rc3/RATE_ESTIMATOR_DESIGN.md`, M4):
  feedback samples are classified to the nearest commanded target within a matching tolerance (user-supplied
  or 5 x the measured resting noise) and monotone target transitions are counted; transitions never exceed
  commands sent; the client's achieved rate and the feedback publish rate are measured and reported; large
  unmatched fractions or inseparable targets yield `undetermined`. When the resting noise forces fewer
  targets than the requested rate needs, the requested rate is kept and the window shortened; fewer than 5
  separable targets within the allowed excursion send nothing and report `targets_not_separable` (found on the
  live SRC v1.0.0 instance, resting noise ~10 mm interface units). Optional secondary channel on `setpoint_cp`.
- **Liveness / stop-behaviour probe redesigned** (`rc3/LIVENESS_PROBE_DESIGN.md`, M6): the probe measures its
  own timing resolution (sleep, send, feedback period, response latency) and reports a resolution floor; no
  fixed post-stream sleep; monotonic timing; configurable response timeout (default 5 x p95 latency);
  requested gaps below the floor are `below_resolution`; stop behaviour is classified as `hold`,
  `release`, `rejected`, `fault`, `not_observable` or `no_policy_within_range`; a release/drift is a stop
  policy. The tau_w estimate needs n >= 3 bisections with a CI lower bound above the floor, otherwise
  `undetermined` (M5.4). A command counts as `rejected` only when no motion toward it is observed; a tracking
  error is reported as `not attained`, not as a rejection (found on the live SRC v1.0.0 instance).
  The resting feedback noise is measured while holding the pose under a command stream at the client rate, so
  that a silence-triggered release policy cannot fire inside the noise window and inflate the hold tolerance
  (found in the first v0.1.1 mock campaign, `validation/v0.1.1/mock-run1-superseded/`).
- Probe steps scale with the measured resting noise of the feedback channel (scale probe: step >= 20 sigma,
  displacement measured as the difference of averaged windows; temporal probe: step >= 20 sigma).
- Scale probe: a trial with no detected motion is `no_response` and excluded; fewer than 3 valid trials
  gives `undetermined` (minor 3). The probe streams its goal at the client rate during settling and measurement and holds the
  start pose under a stream before each step, as a servo client would; a single command followed by silence let the
  AMBF-watchdog emulation's release drift be measured as a unit scale of 1.9 in the first v0.1.1 campaign. Frame probe: pairing window exposed; unpaired samples counted; passive
  (documented).
- Every implementation constant that affects an outcome is a CLI option and is recorded in the report
  (`parameters`); the README lists them (minor 2). `--temporal-trials` separates the liveness trial count
  from `--trials` (minor 4).

### Mock
- Delayed commands are queued and executed in order (latest-wins among *due* commands; stale commands
  discarded) instead of being overwritten (M5.3).
- New fields: `orientation_noise_deg`, `stamp_skew_s`, `max_speed_m_s` (continuous motion toward the goal),
  `anchor_noise_m` (validation only).
- Preset `emul-dvrk-jhu-psm2` uses -150 deg about x, the rotation of the JHU configuration file (0.1.0 used
  the transpose; minor 1). Presets cite ledger rows; the release preset's drift speed is labelled a validation
  parameter (minor 13).

### Report / packaging
- Report carries `expectations`, `parameters` and a `semantics` sentence; schema updated.
- Tracked `.pyc` and `egg-info` files removed; `.gitignore` added. Hard-coded paths removed from the
  validation helpers; the execution environment is described by `validation/environment/` (from-source ROS
  Noetic container recipe, pinned source manifests, wheel hashes, image ids). `meta.json` records numpy,
  scipy, jsonschema, matplotlib, container and ROS manifest.
- Tests: `tests/test_rate_estimator.py` (synthetic feedback, 18 tests incl. the Reviewer #2 120-of-100
  regression), `tests/test_expectations.py` (8), integration tests extended to 23 (identity / non-identity /
  missing expectation; required / forbidden / discover-only state machine; fault / release / hold /
  below-resolution liveness; noisy, interpolating, delayed, dropped rate cases).

### Compatibility
- Report JSON: new top-level keys; probe `estimates` keys changed (`sub_verdicts`, `stop_class`,
  `residual_*`, `expected_*`); `decision_basis` strings changed. v0.1.0 archives are read by
  `validation/analyze.py`, v0.1.1 archives by `validation/analyze_v011.py`.
- Python >= 3.8 (validated on 3.8.10 in the container and 3.11 for the unit tests); ROS 1 only.

## 0.1.0 — 2026-09-02

First release, accompanying the preprint.

- `crtk_conformance`: ROS 1 adapter (master-API discovery, subscriptions, publications, one-hop /tf lookup);
  `FrameSemanticsProbe`, `ScalingUnitsProbe`, `RateSensitivityProbe` (state precondition, liveness, effective rate);
  tolerance-derived thresholds; Student-t statistics over repeated trials; JSON report with schema; CLI.
- `crtk_mock`: kinematic CRTK-compatible node with injectable binding transform, unit scale, operating-state machine,
  liveness timeout (fault/release), loop and publish rates, delay/jitter, drops, noise, missing topics; five presets.
- `validation/`: harness, grid runner (F, S, T, R, P, M experiments), analysis and figure scripts, archived results.
- Not implemented: ROS 2 backend, PDF report, any TF-tree reasoning beyond one hop.
- Known defects found by external review after release (fixed in 0.1.1): rate sub-probe counted
  `round(x, 9)`-distinct samples (noise-fragile); liveness floor ~50 ms from a fixed sleep; spatial
  `conformant` decided against an implicit identity binding; `--expect-state-machine any` default; delayed
  commands overwritten in the mock; liveness estimate accepted at n = 2 with a negative CI.
