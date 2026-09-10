# Changelog

## 0.1.3 — 2026-09-10 (branch `rc5-temporal-semantics`)

Response to the adversarial review of the RC4 manuscript (`preprint/rc5/RC4_ADVERSARIAL_REVIEW.md`, verdict
"major revision"; the reviewer's counterexample script and results are kept verbatim under `tests/adversarial/`).
Executable behaviour changed in the temporal decision procedures and in the reference implementation; the
v0.1.0, v0.1.1 and v0.1.2 archives are unchanged and a new archive was produced under `validation/v0.1.3/`.

### Decision semantics
- **Rate** (`rate_estimator.py`, `probes/rate.py`; review findings 1–3). The verdict is taken on the *source age
  of the applied setpoint*: CRTK defines `setpoint_cp` as the current setpoint sent to the low-level controller
  (api-robot-motion.rst, ledger P64), so a sample of that channel that shows commanded target k means "target
  k is applied now", and its age is now − send(k). The 0.1.2 statistic (the longest interval between changes
  of the channel) was not this quantity: it credited a channel whose stale value was a target the client sent
  long ago (the reviewer's "applied late" case, 0.58 ms first-application delay on a channel that showed the
  previous command), it added the channel's own period as leniency, it ignored the age of the setpoint that
  was in force before the first command of the window, and it measured a mean rate where a maximum age was
  claimed. 0.1.3 brackets the age between causally matched channel samples ([age_lower, age_upper]: the age is
  known at each sample and bounded until the next one), matches each sample to the nearest *earlier* sent
  target, charges the pre-window state as "nothing applied" from the first send, ends the window when the
  last target is first seen applied, and decides `satisfied` iff age_upper ≤ 1/f_req, `violated` iff
  age_lower > 1/f_req while the client's own stream sustained the rate (mean rate and longest send gap),
  `undetermined` otherwise — including a channel that interpolates between targets (> 50 % of the samples
  after the first application match no sent target) and a client stall. No channel-period leniency. The
  probe archives the send stamps and the channel trace of every window (`trace`).
- **Liveness** (`probes/rate.py`; findings 4 and 6). A `fault` or `drift` expectation with a declared
  `horizon_s` is `satisfied` only if the tau_w interval lies entirely within the horizon *and* the eq. (7)
  margin lies below the interval; `violated` if the interval lies beyond the horizon or the margin at or
  above it; `undetermined` when either straddles. 0.1.2 decided the horizon on the stop class alone (a 1.0 s
  fault policy satisfied "fault within 0.1 s"). A horizon beyond the tested silences is `undetermined` only
  when no trip was observed (a trip evidenced inside the tested range decides). Each trial's bound now uses
  the response latency of *its own last streamed command* (the stream ends with an observable half-step
  offset whose response is timed); the allowance L for transactions that drew no response is a client-supplied
  bound (`--latency-bound-s`) or, failing that, the run maximum of the observed response latencies, in which
  case the interval is labelled *conditional* and the verdict note says so. The estimate lists its
  assumptions (policy evaluated at least once per feedback period; deterministic timeout; latency allowance;
  minimum drift speed during detection).
- **Spatial scope** (`spatial.py`, `probes/frame.py`; finding 7). No change to the statistic. The
  distributional model is now stated where the interval is formed and in every report
  (`SpatialDecision.assumptions`, `observations.assumptions`): the Hotelling T² region is exact for iid
  *multivariate-normal* trial errors and for no wider class in finite samples (independence alone is not
  enough — the reviewer's mixture-noise stress case has 42.8 % coverage, reproduced as a documented
  limitation in `tests/test_spatial.py`); the rotation-vector deviations are treated as Euclidean-normal
  samples under a small-angle approximation (the largest per-trial deviation is reported); the campaign's
  containment counts are empirical for their Gaussian-noise configurations. The frame probe's
  `binding_translation_norm_m` is now formed after the unit conversion (0.1.2 formed it in interface units;
  identical for the declared unit 1.0 of every archived run).

### Reference implementation (`crtk_mock`)
- `setpoint_cp` is published as the goal *applied* by the execution loop (with the client's `header.stamp`),
  not as the last accepted goal; a queued command is therefore not shown applied before it is.
- Event log (`event_log_path`, JSON lines): `receive`, `drop`, `reject`, `ignore`, `apply`, `supersede`, with
  the monotonic time, the client's header stamp, the sequence number and the commanded position. The
  v0.1.3 analysis derives the rate truth from this log (finding 5), independently of the estimator.

### Packaging
- Version 0.1.3. `numpy.ndarray.ptp` replaced by `numpy.ptp` (removed in NumPy 2). `validation/run_validation_v013.py`
  / `analyze_v013.py` (event-log truth for every rate window; soundness / completeness / unscored reported
  separately; bracket containment and the lower-end excess; Fig. 6 bracket-vs-truth; horizon cases scored against
  the injected policy and the declared horizon). `--latency-bound-s`.
- Tests: 68 unit (no ROS) + 35 integration (ROS 1), all passing in the validation container at the measurement
  commit; `tests/test_liveness_verdict.py` replays the v0.1.2 archive through the 0.1.3 verdict logic offline.

### Archive (`validation/v0.1.3/`)
- `mock/` (commit 1c06238, 64.2 min, seeds offset 20000; one `*.events.jsonl` per temporal run), `live-src-v1/`,
  `live-src-v2/` (the same commit): the runs reported by manuscript RC5. The v0.1.0, v0.1.1 and v0.1.2 archives are
  untouched (`preprint/verify_rc4_baseline.py`).
- Rate: 180 windows, 160 scored against the event log (20 without a channel or without separable targets); truth
  ok 69 / violated 52 / client-limited 39; 106 determinate verdicts agree with the truth, 54 undetermined, 0 false
  conformant, 0 false divergent; completeness 106/121; every client-stall window undetermined; the bracket contains
  the true age in 98 of 150 windows, every miss an excess of the lower end by the channel's transport delay
  (≤ 1.33 ms outside client stalls, ≤ 8.6 ms during them; 95th percentile 0.88 ms).
- Liveness: 23 intervals with status ok, 23 contain the injected timeout (22 conditional on the run-maximum
  allowance, 1 with a client-supplied bound); 0 wrong definite stop verdicts; horizon below / at / above the policy
  → violated / undetermined / satisfied for both a fault and a drift policy; the RC4 reviewer's replay (1.0 s fault
  against "fault within 0.1 s") → violated.
- Spatial and dimensional: 120/120 and 240/240 containment (Gaussian noise), 0 false conformant / divergent over
  the sweeps, as in v0.1.2.

### Found by the campaign
- The window-end age of the rate bracket was charged at the receipt of the sample showing the last target; on a
  20 Hz channel this added the publication interval to an on-time application and produced a false *violated*
  verdict in the pre-freeze check. The window end now charges the last target's age at its application. Found by
  scoring against the event log, before the reported campaign.
- The probe's Python send loop stalls (> 20 ms between sends) recur in 39 of 180 windows in this container; every
  one is *undetermined* by the client-stall rule and is reported as client-limited, not charged to the mock.

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
- Tests: 55 unit (no ROS) + 31 integration (ROS 1), all passing in the validation container at the release commit.

### Archive (`validation/v0.1.2/`)
- `mock/` (commit d322518, 54.1 min), `live-src-v1/`, `live-src-v2/` (commit 0f4549b, which differs from d322518
  in `validation/analyze_v012.py` only): the runs reported by manuscript RC4. `mock-run1-superseded/`,
  `live-src-v{1,2}-run1-superseded/` (commit f9e2b51): the first freeze, superseded because the rate estimator did
  not distinguish a stall of the probe's own send loop from a stale accepted-command channel (see the READMEs);
  kept unchanged, used for no claim. The v0.1.1 and v0.1.0 archives are untouched.

### Found by the campaign
- A ~47 ms stall of the probe's Python send loop recurs in the 120 Hz-loop / 1000 Hz-publish configuration and in
  one regression case; the rate verdict is `undetermined` there (client's longest send gap > required period).
- Every v0.1.1 fault-mode liveness case with a timeout below 1 s had no latency measurement (the policy fired during
  the silent settling before the latency probes); 0.1.2 recovers the implementation first.

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
