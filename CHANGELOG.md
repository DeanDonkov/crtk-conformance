# Changelog

## Validation of 0.1.8 (RC14) — 2026-09-22

No change to `src/` or `tests/` (identical to `a52e89e`).
- **Pre-registered campaigns** (`validation/v0.1.8/PREREGISTRATION.md`, commit `79b3f71`, before any run;
  `validation/v0.1.8/RESULTS.md`): F (reference node, correlation guard, 160 runs): 6/6 predictions matched; D (dVRK
  console, 27 runs): 2/2 matched; S (SRC, 18 anchor runs): S-1 deviates — the joint-space anchor detected the 0.1-m
  unit of SRC v1.0.0 in one launch (3 runs) and abstained in two (wrist-yaw chatter; residual gates failed, no false
  verdict); S-2 (SRC v2.0.0 control) and S-3 matched. One SRC v2.0.0 launch failed at start-up and was repeated
  (`--only-launch`, commit `da00219`), as pre-registered.
- **Offline studies** (`validation/v0.1.8/studies/`): correlation guard, operating characteristic (decision rate and
  error among determinate verdicts), joint-space anchor operating range, deadlines decidable by the sound fault bound.
- Exploratory, after campaign S: the single-joint estimator on the campaign-S poses (`validation/rederive_single_joint_S_v018.py`): on
  v1.0.0 launch 1 it also decides divergent in 2 of 3 runs, so the detection does not show that the joint-space fit was needed.
- **Known issue** (found by the final test run): `poe_fit_trial`'s `max_nfev=200` allows only about six LM iterations under
  SciPy 1.10.1 (the campaign image), which counts the Jacobian's evaluations in the budget; one unit test fails there.
  Refitting every campaign trial to convergence changes no verdict (`validation/poe_scipy_budget_v018.py`). To be fixed
  in a later version (bound iterations, not evaluations) with new validation runs.
- RC14 tables, figure, SRC trace analysis and reporting checks (`validation/tables_v018.py`, `fig_v018.py`,
  `src_traces_v018.py`, `verify_reporting_v016.py --tag rc14`).

## 0.1.8 — 2026-09-22

Two probe changes in response to an external review of the RC13 manuscript (points 1 and 3).  Both change how evidence
is collected, not how an interval is decided.

### Changed
- **Frame probe: correlation guard** (`spatial.correlation_plan`, `spatial.correlation_guard`; `--no-correlation-guard`
  restores 0.1.7).  Before the trials the probe records a resting window of paired residuals (at least 3 s, extended up
  to 30 s until it spans 20 integrated autocorrelation times), spaces the ten trials by max(5, ceil(2 tau_int)) samples,
  and withholds the verdict when the correlation time is not resolved, when the spaced trials would exceed 60 s, or when
  fewer than four effectively independent trials remain.  A residual without noise (the dVRK computes one channel from
  the other) keeps back-to-back trials.  The report adds `observations.correlation_plan` and
  `estimates.outcome_without_correlation_guard`.
- **Geometry anchor: joint-space method** (`dimensional.poe_fit_trial`, `dimensional.geometry_anchor_poe_decision`;
  `dimensional.anchor_method: poe`, the new default; `single_joint` and `--anchor-method single_joint` restore 0.1.7).
  Each trial steps every joint of the declared chain (`joint_types`, default `RRPRRR`) by +- a small step and fits the
  spatial twists of all joints to the measured joint changes and poses (product of exponentials at the measured
  reference configuration).  Coupled or incomplete joint motion is then part of the data.  Gates: largest fit residual
  <= 0.02 rad and <= 2 % of d_int, each wrist joint moved >= 0.05 rad, axes at the model angle +- 2 deg.
- `dimensional.se3_exp` (closed form).  crtk-mock: `noise_model: ar1` with `ar_phi` (AR(1) position noise per publish
  step on measured_cp and local/measured_cp).
- The campaign scripts of v0.1.3, v0.1.6 and v0.1.7 pin the 0.1.7 frame procedure (and, for v0.1.6, the single-joint
  anchor), so they reproduce their archives.

### Tests
- `tests/test_v018.py` (6, ROS-free) and one integration test of the guard under AR(1) noise.


## Validation additions after 0.1.7 (RC13) — 2026-09-22

No change to `src/` or `tests/` (identical to the 0.1.7 release commit `2fac51b`).
- **Confirmatory campaign of 0.1.7**, pre-registered in `validation/v0.1.7/PREREGISTRATION.md` before any run: K1 and
  controls (9 runs) and the loss experiment (64 runs) with fresh seeds (`validation/run_validation_v017.py`,
  `validation/analyze_v017.py`). 11 of 13 predictions matched. One hold run under 5 % loss was undetermined (its
  calibration lost 5 of 30 commands, too many for confirmation), so L-1 and L-2 deviate. All 32 fault intervals contain
  τ_w; 2 of the 32 conditional (0.1.6-rule) estimates exclude it. `validation/v0.1.7/RESULTS.md`.
- **Offline sensitivity studies** (`validation/boundary_sensitivity_v017.py`, `validation/anchor_sensitivity_v017.py`;
  `validation/v0.1.7/studies/`), and the RC13 table generator `validation/tables_v017.py`.

## 0.1.7 — 2026-09-21

Two decision-rule changes made after the pre-registered v0.1.6 campaigns, in response to an external review of the RC11
manuscript (points 1 and 2). They are **post hoc**, and the campaigns stay reported under their own rules. Both are
re-derived offline on the archive in `validation/v0.1.7/` (`validation/rederive_v017.py`); no run was repeated.

### Changed
- **Consistency gate.** When the scale probe's command/feedback consistency diagnostic is raised, the unit verdict is
  withheld (undetermined, with the ungated outcome kept in `unit_outcome_without_consistency_gate`). The report adds
  `verdict_kinds`:
  - `spatial_feedback_binding`: the frame probe's verdict;
  - `spatial_command_semantic`: that verdict read for `servo_cp`, undetermined when the flag is raised;
  - `shared_binding_assumption`.

  With the flag raised, the report's dimensional class is also undetermined. `--no-consistency-gate` restores 0.1.6.
  Archive: of 102 runs carrying the diagnostic, only the 3 K1 runs are flagged; their unit verdict changes from
  divergent to undetermined.
- **Liveness rule `0.1.7`** (new default; `0.1.6` and `0.1.5` stay selectable).
  - Rejections keep the 0.1.6 confirmation rule.
  - Every faulted trial bounds τ_w by the time its FAULT was observed. This is sound whether or not the post-gap
    command arrived, and wider by about the response wait.
  - The 0.1.6 interval is reported as `conditional_estimate_s`, conditional on arrival; it decides nothing.
  - For records without an observation time, the bound is gap + response wait + 0.2 s + G
    (`liveness.fault_observation_window`).

  Loss experiment: 64/64 intervals contain τ_w (0.1.6: 53/54, with 10 contradictory); median width without loss rises
  from 45 to 334 ms. Archived v0.1.3: 23/23 contain τ_w; median width/τ_w rises from 0.14 to 0.53.
- The stop decision for fault/drift expectations is `liveness.fault_horizon_decision`, moved out of the probe
  unchanged. The hold branch uses `liveness.trip_upper_bound` under rule 0.1.7.

### Tests
- `tests/test_v017.py`: the gate, the fault bound, verdict kinds, and replays of the K1 runs, the 64 fault-policy loss
  runs and the one 0.1.6 excluding interval.
- `test_liveness_interval_contains_timeout` now checks tightness on the conditional estimate.

## 0.1.6 — 2026-09-21

RC9 response to the external review of the RC8 review copy, with the RC10 reporting corrections (below). Every change is listed in `validation/v0.1.6/PREREGISTRATION.md`
(and its addenda A–D) with the order in which it was made relative to the pre-registered runs. The v0.1.3 archive and the
0.1.5 verification campaign are unchanged; all new measurements are in `validation/v0.1.6/`.

### New
- `dimensional.py` (ROS-free): the scale decision of eq. (6), moved unchanged out of `ScalingUnitsProbe.run()`
  (`scale_decision`; a test replays every archived v0.1.3 scale run), and the instrument-geometry unit anchor: screw
  axes of wrist steps, their common normal, and `geometry_anchor_decision` (Student-t interval of L / d_int widened by a
  declared relative uncertainty u_rel, gates on the step angle, the axial slide and the angle between the axes).
- `probes/geometry.py`: `GeometryAnchorProbe` (`--probes geometry`, `--geometry-trials`, `--geometry-settle-s`);
  `adapter.servo_jp()`; expectation mode `dimensional.mode: geometry_anchor` (L_m, u_rel, L_source, pitch_joint,
  yaw_joint, delta_q_rad, axes_angle_deg, reference_joints).
- Scale probe: `command_feedback_consistency` diagnostic (settled goal residual; flags a non-shared binding or a
  tracking deficit; not a verdict).
- Liveness: 30 calibration commands (was 12), `liveness.loss_upper_bound()` (one-sided Clopper–Pearson),
  `required_confirmations()` and the **confirmation rule** (rule "0.1.6", default): a non-response without a state change
  counts only after r = max(2, ceil(ln α / ln p_up)) repeats at the same gap, each after its own answered stream
  (α = 0.01, r ≤ 4); the 0.1.5 rule stays selectable (`--liveness-rule 0.1.5`, `--calibration-commands 12`).
- `--skip-rate-sweep`, `--enable-timeout-s`.
- Reference node: separate command binding (`cmd_bind_*`), mixture noise, Gilbert–Elliott loss; all default to the
  archived behaviour (fixture test over every preset).

### Changed
- Frame probe: samples with an unset (zero) header stamp are skipped and counted (the released dVRK publishes identity
  poses stamped 0 until homed).
- Report: probes of the same class are combined (any D → D; else any C → C; else U).
- `probes/common.ensure_enabled`: one state command at a time — `enable` only when not ENABLED, then `home` after an
  enable or when not homed, awaiting is_homed and not is_busy; nothing is sent to a ready arm. The released dVRK console
  (cisst-ros 4.0.0 ROS 1 bridge, subscriber queue size 1) keeps only the latest state command, so 0.1.5's back-to-back
  `enable` + `home` never re-enabled a disabled arm (addendum A; before any probe run).

### Fixed
- `RateSensitivityProbe.run()` raised `KeyError('per_rate')` with `--skip-rate-sweep` (addendum C).

### Analysis and reporting (RC10; external review of the RC9 copy; no run repeated, no verdict changed)
- `validation/analyze_v016.py`, live boundary coverage: `e.get("ci_low") or 1` replaced every legitimate 0.0 lower
  interval end of the scale probe by 1 m. Endpoints are now tested explicitly for missing or NaN values
  (`interval_covers`). Live scale coverage is 30/30, 28/30 and 30/30 at 0.95, 1.00 and 1.05 ε (0.93–1.00), not
  27/25/27 (0.83–0.90). `analysis/B_live_boundary.csv` regenerated (new columns `covered`, `interval_missing`).
- `validation/verify_reporting_v016.py` (`--tag`): recomputes that coverage from the raw records and cross-checks the
  analysis CSV; checks the live per-cell counts against the supplement text (previously only printed), the Monte Carlo
  coverage ranges, the RC10 wording corrections and the final test log.

### Tests
- 98 ROS-free unit tests (+1 skipped without ROS); 137 with ROS. Two timing tests fail intermittently on a 2-vCPU host
  under pytest, with the 0.1.5 code as well (`validation/v0.1.6/tests/README.md`).

## 0.1.5 — 2026-09-19

Response to the adversarial review of the RC7 (T-MRB) manuscript. Three changes to the liveness/stop sub-probe of
`probes/rate.py`; the decision logic that they touch now lives in the ROS-free module `liveness.py` so that it can be
replayed offline on archived trial records. The frame, scale, state and rate procedures are unchanged; the v0.1.0–0.1.3
archives are unchanged (`validation/reanalyze_liveness_v015.py` re-derives the archived liveness intervals under the
0.1.5 rules without touching them: 23 of 23 still contain the injected timeout; the archived delayed-application case
widens from [489.0, 824.0] to [176.4, 824.0] ms; the eleven drift intervals' lower ends move down by one feedback period).

### Liveness lower bound (review finding F1)
- The response latency $r_i$ of the last streamed command (the return to base after the half-step offset) bounded the
  one-way transport latency of that command only if the sample read as the response *was* the response. 0.1.3 took the
  first gap sample within the half-step band around the base pose; an implementation that applies commands later than
  one feedback period is still executing the earlier in-band stream when the gap starts, so that sample precedes the
  actual response (archived `L_delayed`: $r_i$ = 0.2–9.7 ms against ~310 ms responses; the archived interval contained
  the timeout only because the reference node times its policy from receipt and loopback transport is negligible).
- `liveness.last_command_latency()` accepts a sample as the response only after a sample *outside* the band — the
  departure to the half-step offset — was observed (the pose leaves the band only once the half-step is applied and
  returns only once the last command is). Without an observed departure the trial's lower bound uses the latency
  allowance $L$ instead, and the estimate says so (`last_stream_departures_observed`, `lower_bound_uses_L_for_all_trials`).
  `_gap_trial` therefore no longer clears the feedback buffer after the stream: the samples between the half-step send
  and the last send are needed by the guard. Only samples up to the post-gap send are considered: a later in-band
  sample may be the pose passing the base on its way to the post-gap goal (a 2.05-s "response" after a 2.0-s silence
  on live SRC v1.0.0 in the first verification run), and a response that arrives later is not attributable.

### Onset-based drift lower bound (finding F8)
- `liveness.timeout_interval_from_trials()`: the drift onset bound is $o_i - r_i - G - \delta_i/v_{\min} - P - L$; 0.1.3
  omitted the policy-evaluation granularity $G$ that the passing-trial bound already carried. Under the stated
  assumption $G = P$ the archived drift lower ends were one feedback period too high; all eleven still contained the
  injected timeout (margins 12.3–24.1 ms against a 10-ms correction).

### Post-gap non-responses versus baseline command loss (finding F10)
- A post-gap command that draws no response is evidence of a silence-triggered stop policy only if commands sent without
  a preceding silence are answered. `probe_liveness` now reads the calibration of `measure_resolution()` (12 commands
  sent without silence): when any of them drew no response, a `rejected` trial (a non-response without a visible
  state change) is *unattributable* — it may be the loss of the post-gap command itself — and contributes no bound
  (`liveness.attributable()`); a `faulted` trial remains attributable, because a FAULT read from the operating state
  cannot be produced by loss, but it then bounds the timeout by the time its state was observed
  (`state_observed_at_s`, recorded per trial), not by the gap: the lost post-gap command lets the policy fire during
  the response wait (seen in the v0.1.5 verification campaign, `V_drop_fault`: a 10.7-ms gap "faulted" against a
  0.25-s policy). A `drifted` trial is attributable (the drift is seen in the pose). When no attributable tripping
  trial remains the timeout estimate is `undetermined` with the reason recorded (`rejection_confounded_by_command_loss`,
  `baseline_command_loss`, `confound_note`), and `run()` leaves a hold/fault/drift expectation `undetermined` instead
  of `violated`. The archived regression case `R_drop_50` (no stop policy, 50 % random drops; 7 of 12 calibration
  commands unanswered) was reported as `rejected` at every gap with "tau_w <= 23 ms" and a violated hold expectation;
  under 0.1.5 it is undetermined. The cost of the rule is stated in the report: a genuine rejection policy on an
  interface that also loses commands is undetermined, not detected (`V_drop_reject`).

### Observability of a gap trial (found by the v0.1.5 verification campaign, not by the review)
- Two further pre-existing defects of the stop sub-probe surfaced when the new experiment V ran the revised probe against
  delayed application and random command loss; both are fixed in 0.1.5 and neither touches an archived interval
  (`tests/test_liveness_v015.py` checks that on the archive):
  - A `held` trial whose silence ended before the settled reference window closed (no feedback sample after it) had its
    drift never evaluated, yet counted as a passing trial. For a release-with-drift policy the post-gap command is acted
    on after the release, so the response alone does not show that the policy had not fired: in `V_delayed_drift_1`
    (300-ms application delay, hence a 0.93-s reference window) silences of 0.85–0.92 s were "held" and pushed the lower
    end to 547 ms above the 500-ms timeout. Each trial now records `drift_evaluated`; under a drift stop class only
    evaluated trials are passing trials (`liveness.drift_evaluated()`, archived records judged by gap versus window).
    A drift that began inside the reference window is censored: its onset is the first observable sample and bounds
    the timeout from above only (`reference_window_still`, the window's spread against max(4 σ̂, still tolerance)).
    The 300-ms-delay drift case is therefore `undetermined` (lower end at the floor) rather than wrong; the settle hint
    of three times the largest response latency is the limiting heuristic and is left as is.
  - Under command loss the recovery before a stream can fail silently (its enable or return-to-base command lost): the
    silence then starts in FAULT, or with the pose already at the post-gap goal, and the post-gap command's "response"
    is spurious (attained without motion; `V_drop_fault`: "held" at 0.76 and 1.13 s of silence against a 0.25-s fault
    policy, hence an inconsistent interval). A trial is now `not_observable` when the operating state at the start of
    the silence is FAULT/DISABLED (`state_at_gap_start`, read from the latest state message without waiting) or when
    the pose at the end of the silence is already within the attainment tolerance of the post-gap goal
    (`distance_to_goal_at_gap_end_m`); `not_observable_reason` says which. An unobservable bisection trial moves
    neither end of its bracket.

### Other
- Author metadata in `pyproject.toml` corrected to the author's name as published (Dean Donkov); `LICENSE` already carries the full Apache-2.0 text without a name field since ad2232d.
- Tests: `tests/test_liveness_v015.py` (13 tests: the departure guard on synthetic samples, the granularity term, the
  fallback to $L$, the attribution rule under command loss, the observability rules, offline replays of the archived
  `L_delayed`, drift and `R_drop_50` records). Unit tests: 82. Three
  integration tests (`test_rate_final_command_only_is_not_credited`, `test_rate_accepted_channel_satisfied_on_reference`,
  `test_fifo_queue_slower_than_the_client_is_violated_and_logged`) still asserted the 0.1.3 rate sub-verdicts and
  therefore failed against 0.1.4 in a ROS environment; they now assert `undetermined` from the tool and the archival
  0.1.3 rule on the same bracket; `test_rate_reduced_targets_keep_the_requested_rate` placed its resting noise exactly on
  the five-target boundary of the rate diagnostic and failed in one of three container runs, so its noise is 0.4 mm and
  it accepts 5–7 targets (the rate diagnostic itself is unchanged). Integration tests: 35 (run under the pure-Python ROS 1 stack of `docs/ros1-stack.md`
  and in the `focal-crtk:rc3` container).
- Validation: `validation/run_validation_v015.py` (the v0.1.3 campaign design with seeds offset by 30000, plus
  experiment V: delayed application, delayed drift, random loss with a hold / fault / rejection policy) and
  `validation/v0.1.5/` (plan, mock campaign, live SRC v1.0.0 / v2.0.0 runs, the offline reanalysis of the v0.1.3
  liveness archive, test logs). The v0.1.3 archive remains the measurement set reported in the manuscript.

## 0.1.4 — 2026-09-11

> **Commit hashes**: the history was re-authored to a single author before first publication, so every
> commit SHA cited below and in `validation/` predates that rewrite. `PROVENANCE-REWRITE.md` maps each of
> them to its current SHA; no file content changed (every tree object is identical).

Labelling only. **No measurement, estimator or decision input changes**: `estimate_applied_age()` is untouched,
and every reported quantity — the source-age bracket `[age_lower_s, age_upper_s]`, the acceptance counts, the
client's achieved rate and send gaps, the ZOH error bound — is computed exactly as in 0.1.3. The archived
v0.1.0–0.1.3 validation outputs are unchanged and are not regenerated.

### The rate class no longer carries a conformance verdict
- `rate_estimator.rate_subverdict()` returns `"undetermined"` unconditionally; `satisfied` / `violated` are no
  longer emitted for the rate part of a declared temporal expectation, in the JSON report or the text summary.
  Two properties of the channel, not of any particular run, make the decision unsound:
  1. `setpoint_cp` identifies the applied command by matching its **value** against the history of sent targets.
     It carries no explicit command identifier and no send-time stamp, so a repeated or superseded target cannot
     be attributed unambiguously.
  2. `age_lower` is measured at the probe's **receipt** of the sample and therefore exceeds the setpoint's
     publication-side age by the channel's transport delay, which no CRTK artifact bounds for an arbitrary
     implementation. `--latency-bound-s` subtracts a client-supplied bound but cannot establish one.
- `probes/rate.py` reports the bracket as a diagnostic note and records the withdrawal reason verbatim in
  `estimates.rate_note` (`rate_estimator.RATE_VERDICT_WITHDRAWN_NOTE`). The obsolete
  `estimates.rate_verdict_conditional_on_feedback_transport_delay` key is removed — there is no verdict for it
  to qualify.
- **Consequence for the aggregate temporal outcome**: `combine()` is unchanged, so a run that declares
  `rate: required` can no longer come out `conformant` — one declared class is now always undecided. A
  `violated` state-machine or stop-behaviour sub-verdict still makes the temporal outcome `divergent`, and both
  of those classes are still decided and reported per class in `estimates.sub_verdicts`.
- Deciding this class needs a specification change, not another estimator revision: an explicit command
  identifier or send-time stamp on the command channel (recommendation 3 in the manuscript).

### Reproducibility of the archived campaign
- The 0.1.3 rule is kept verbatim as `rate_estimator.rate_subverdict_v013_archival()`, used by
  `validation/analyze_v013.py` (which scores the archived v0.1.3 campaign, produced under that rule) and by the
  RC3/RC4 reviewer counterexample scripts, whose bodies are unchanged apart from the redirected import. Every
  soundness/completeness figure derived from the v0.1.3 archive re-derives exactly.
- `tests/test_rate_estimator.py` pins both: the 0.1.3 cases now assert against the archival rule, and
  `test_rate_class_carries_no_verdict_0_1_4` asserts that the tool itself reports `undetermined` whatever the
  bracket says, while the brackets themselves still order as before.

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
