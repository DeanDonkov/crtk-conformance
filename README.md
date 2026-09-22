# crtk-conformance

Black-box **semantic** conformance probes for CRTK/ROS surgical-robot interfaces, plus
**crtk-mock**, a configurable CRTK-compatible reference node with injectable semantic divergences.

Companion code for the manuscript *Structural Conformance Is Not Semantic Conformance: An Error Model
and Black-Box Test Suite for CRTK/ROS Interfaces in Surgical Robotics* (Donkov, 2026). **Version 0.1.3 —
the revision described by manuscript RC5** (see `CHANGELOG.md`; the measurement code of the reported archives is
the commit recorded in `validation/v0.1.3/*/meta*.json`). This branch (`rc5-temporal-semantics`) is the single
entry point for RC5; the RC4 state is branch `rc4-decision-semantics` (commit `59c4876`), the RC3 state
tag/branch `rc3-major-revision`, the RC2 state commit `c3ecdc1`.

Two implementations of the same CRTK topic names and message types can still interpret a
`PoseStamped` in different frames, in different units, or under different timing and state
preconditions. The probes estimate each of these **semantic bindings** through the public ROS
interface only and report a three-valued outcome per class.

## The semantic rule

> `conformant` means that the discovered binding satisfies an **explicitly declared client expectation**
> within the client's stated task-space tolerance, on the channel the probe observed and under the assumptions
> the report records. `divergent` means the declared expectation is violated beyond the tolerance, established
> by an interval that lies entirely beyond it (never by a bound that merely could exceed it). `undetermined` is
> returned whenever the interface does not expose what the decision needs, whenever data are missing or
> ambiguous, whenever the interval straddles the tolerance, whenever the expectation is claimed beyond what was
> tested — and whenever **no expectation was declared**: with no expectation there is no conformance verdict,
> only a report of what was observed.

The expectations are a small YAML file (`--expectations`):

```yaml
spatial:
  mode: expected_transform        # identity | expected_transform | discover_only (default)
  translation_m: [0.20, 0.0, 0.0] # the base_frame-like transform a measured_cp / local/measured_cp pair shows,
                                  # in the unit declared below (dimensional.expected_unit_m) — a precondition
  quaternion_xyzw: [-0.9659258, 0, 0, 0.2588190]
  orientation_tolerance_deg: 2.0  # optional (0.1.2): separate orientation verdict; otherwise positional only
dimensional:
  mode: si                        # si | discover_only (default); 'si' needs --anchor-topic
  expected_unit_m: 1.0            # metres per interface unit the client assumes
temporal:
  state_machine: required         # required | forbidden | any (default)
  stop_behaviour: hold            # hold | fault | drift | release | any (default); 'release' is undetermined from pose
  horizon_s: 2.0                  # the silence up to which the stop expectation is claimed (default: tested range); 0.1.3: enforced on the tau_w interval for fault/drift
  rate: required                  # required | any (default); 0.1.4: NEVER decided -- the source age of the applied setpoint is reported as a diagnostic and the sub-verdict is always `undetermined`
```

## Decision semantics (0.1.3)

| Class | Decision quantity | Interval | Verdict rests on | Assumptions recorded in the report |
|---|---|---|---|---|
| spatial | exact maximum positional error of the residual `T_hat · T_expected⁻¹` over the ball ‖p‖ ≤ r_ws (eq. 3′), positional only | Hotelling T² regions of the translation and rotation-vector residuals propagated through the Lipschitz bounds of e_max (≥ 95 %, conservative; undefined for n ≤ 3) | `measured_cp` vs `local/measured_cp` (passive) | translations read in `expected_unit_m`; the frame of `servo_cp` is *assumed* to be that of `measured_cp` (source-supported for dVRK/SRC, never tested by the probe) |
| dimensional | anchored scale ŝ vs the declared unit, eq. (6) at r_ws | Student-t on the per-trial ratios | anchor topic (external SI pose) | the anchor's accuracy |
| temporal / state | executed disabled / enabled | — | `operating_state`, `state_command`, response to `servo_cp` | — |
| temporal / stop | observational class (`held`, `drifted`, `rejected`, `faulted`, `not_observable`; `held_through_range` for the whole probe) and a timeout **interval** | intersection of per-trial bounds: passing gap − r_last (that trial's own last-command latency) − G − t_det; trip + L (fault/rejection) or the drift onset; L = `--latency-bound-s` or, *conditionally*, the run maximum of observed latencies | pose during the silence and the response afterwards | deterministic timeout evaluated at least once per feedback period G; a `hold` is claimed only up to `horizon_s`; a `fault`/`drift` expectation is satisfied only if the interval lies within `horizon_s` **and** 1/f_c + J_max lies below it; `release` is undetermined from the pose |
| temporal / rate | **source age of the applied setpoint** (t − send time of the target the channel shows applied), bracketed [age_lower, age_upper] between causally matched samples — **reported, never decided (0.1.4): the sub-verdict is always `undetermined`** | the bracket itself | **`setpoint_cp`** (CRTK: current setpoint to the low-level controller); feedback crossings are a diagnostic only | `setpoint_cp` piecewise constant on the commanded targets (checked); no explicit command identifier or send-time stamp on the channel, and age_lower is measured at receipt, above the publication-side age by an unbounded transport delay — the two reasons the class carries no verdict |
| spatial (scope) | — | the Hotelling region is exact for iid **multivariate-normal** trial errors only; rotation deviations treated as Euclidean-normal under a small-angle approximation; campaign coverage is empirical for its Gaussian configurations (the RC4 reviewer's mixture noise gives 43 % coverage: `tests/test_spatial.py`) | — | recorded in every frame report (`observations.assumptions`, `spatial_decision.assumptions`) |

## What is and is not implemented (0.1.3; 0.1.6 additions marked)

| Implemented | Not implemented |
|---|---|
| ROS 1 (`rospy`) transport; discovery through the ROS master API | ROS 2 (no `rclpy` backend) |
| `FrameSemanticsProbe` — binding of the unqualified `measured_cp` relative to `local/measured_cp`, decided on the exact maximum error eq. (3′) against a declared expected transform with a propagated interval; **passive** (never publishes `servo_cp`, so it never tests the command frame); 0.1.8: correlation guard (resting window, trial spacing by 2 τ_int, withheld when the correlation is unresolved or too long) | inferring the binding when `local/` is absent (undetermined by construction); TF beyond a one-hop `/tf` lookup; an active command-frame test |
| `ScalingUnitsProbe` — internal command/measurement ratio, and a unit estimate **only with an out-of-band anchor topic**; noise-adaptive step; goals streamed at the client rate; no-response accounting; 0.1.6: command/feedback consistency diagnostic; 0.1.7: a raised diagnostic withholds the unit verdict | detecting a uniform unit scale without an anchor (impossible; paper Sec. 5.2) |
| 0.1.6: `GeometryAnchorProbe` — unit anchor from instrument geometry: screw axes of wrist-pitch/yaw steps (`servo_jp`), their common normal against a declared link length L with relative uncertainty u_rel, eq. (6) decision; gates return undetermined. 0.1.8: joint-space method (default): every joint stepped, product-of-exponentials fit to the measured joints and poses, so reported coupled or incomplete motion is data; `--anchor-method single_joint` restores 0.1.6 | an anchor for instruments without a declared, perpendicular pitch–yaw pair |
| `RateSensitivityProbe` — operating-state precondition; liveness / stop-behaviour probe with measured timing resolution, observational stop classes, drift onset/speed estimation and a timeout **interval** with explicit allowances; source age of the applied setpoint reported by `setpoint_cp` (bracketed between samples) as a **diagnostic**, with the feedback-crossing statistic likewise | identifying the internal controller rate (not identifiable through the interface); identifying a physical release from the pose; measuring the platform's own jitter; **any rate verdict at all (0.1.4)** |
| JSON report validated against `schema/report.schema.json`; text summary; every outcome-affecting constant recorded | PDF reports |
| `crtk-mock` with presets emulating *documented* behaviours (built from cited configuration values) | any emulation of dVRK/AMBF/SRC *code* |
| Unit tests (no ROS; 98 in 0.1.6) incl. both reviewers' counterexamples (`tests/adversarial/`, `tests/test_liveness_verdict.py`, `tests/test_liveness_v015.py` and `tests/test_liveness_v016.py` offline replays) + integration tests (ROS 1; 137 tests in total with ROS) | hardware tests of any kind |

Three probe families, one per binding class; the temporal probe has three sub-probes. There are no others.

## Install

```
pip install -e .            # library + CLIs (numpy, scipy, jsonschema, PyYAML)
pip install -e .[dev]       # + pytest, matplotlib
```

ROS 1 must be available to the interpreter: `rospy`, `rosgraph`, `geometry_msgs`, `sensor_msgs`,
`tf2_msgs` and `crtk_msgs` (https://github.com/collaborative-robotics/crtk_msgs). A ROS Noetic
installation provides all of them. The validation and live runs reported in the paper were executed in a
container whose complete recipe (from-source ROS Noetic on Ubuntu 20.04, pinned commits, wheel hashes,
image ids) is in `validation/environment/`. `docs/ros1-stack.md` describes the pure-Python stack used for
the v0.1.0 archive.

## Run against a platform

```
crtk-mock --preset emul-dvrk-jhu-psm2 &                       # or your real arm
crtk-conformance run --namespace /PSM1 --tolerance-mm 1.0 --workspace-radius-m 0.10 \
    --speed-mm-s 50 --client-rate-hz 100 --jitter-max-ms 5 --trials 10 --temporal-trials 5 \
    --expectations expectations.yaml [--anchor-topic /tracker/tool_pose] [--latency-bound-s 0.005] --out report.json
```

## Task thresholds versus implementation constants

Every **decision threshold** is derived from the user's inputs (tolerance ε, workspace radius r_ws, speed v,
client rate and jitter bound) through the paper's error model: eq. (3′) (the exact maximum over the ball; the
bound eq. (3) is reported for comparison only) for the spatial residual, eq. (6) for the unit scale, eq. (7)
against the timeout interval and eq. (9) on the longest stale interval for the temporal class. Nothing about ε
is hard-coded.

The probes also contain **implementation constants** that affect whether an estimate can be formed (never
where the decision boundary lies). All are CLI options with these defaults and are written into the report's
`parameters`:

| Constant | Default | Option |
|---|---|---|
| scale-probe step (interface units) | 0.005, enlarged to ≥ 20 × resting noise | `--step-mm` |
| temporal-probe step | 0.002, enlarged to ≥ 20 × resting noise | `--temporal-step-mm` |
| still / no-response tolerance | 1e-5, enlarged to ≥ 3 × resting noise | `--still-tol-mm` |
| settle time per scale step | 1.0 s | `--settle-s` |
| stamp pairing window (frame probe) | 50 ms | `--pairing-window-ms` |
| post-gap response timeout | 5 × p95 measured response latency, ≥ 0.2 s | `--response-timeout-s` |
| rate matching tolerance δ | 5 × measured resting noise, ≥ 1 µm | `--rate-match-tolerance-mm` |
| rate window per nominal rate | 1 s | `--rate-window-s` |
| largest rate-sweep excursion | 0.05 interface units | `--rate-max-step-mm` |
| smallest requested gap / bisection steps | measured floor / 7 | `--gap-min-s`, `--bisection-steps` |
| liveness stream | 0.3 s at 100 Hz | constructor only |
| resting-noise window | 1 s, holding under a stream at the client rate | — |
| channel-bounded flag | observable ≥ 0.9 × min(client, publish) | `rate_estimator.BOUND_FRACTION` |
| unmatched-fraction limit | 0.5 | `rate_estimator.UNMATCHED_UNDETERMINED_FRACTION` |
| fewest separable rate targets | 5 | `probes/rate.MIN_RATE_TARGETS` |
| boundary guard band | 1e-9 relative | `probes/base.BOUNDARY_REL_GUARD` |
| trials | 10 (frame, scale), 5 (liveness), 1 window per rate | `--trials`, `--temporal-trials` |

`--trials` (default 10): the scale estimate is the mean of *n* trials with a Student-t 95 % interval; the spatial
interval is propagated from Hotelling T² regions (needs n ≥ 4); the liveness interval is built from the passing and
tripping gaps of `--temporal-trials` bisection runs (needs ≥ 3). `--still-tol-mm` also sets the drift-onset floor.
0.1.5: the lower end of the liveness interval uses the response latency of the last streamed command only when that
command's departure to the half-step offset was observed before its return to base (`liveness.last_command_latency`);
otherwise the latency allowance is used and the estimate says so. The onset-based drift bound carries the granularity
term. When calibration commands sent without any silence draw no response, a post-gap non-response without a visible
state change is not read as a stop policy (`rejection_confounded_by_command_loss`; a FAULT read from the operating
state still counts, bounded by the time it was observed) and the stop expectation stays `undetermined`. A gap trial is
`not_observable` when the silence started in FAULT/DISABLED or the pose was already at the post-gap goal, and a `held`
trial whose drift could not be evaluated is not a passing trial for a drift policy (`CHANGELOG.md`, 0.1.5).
0.1.6: 30 calibration commands, and a post-gap non-response without a state change counts only after it recurs at the
same gap (confirmation rule, `liveness.required_confirmations`; `--liveness-rule 0.1.5` restores the 0.1.5 rule).
State commands are sent one at a time (`ensure_enabled`): the released dVRK console keeps only the latest.
0.1.7 (default `--liveness-rule 0.1.7`): every faulted trial bounds the timeout by the time its FAULT was observed, which
is sound under command loss; the 0.1.6 interval is reported as `conditional_estimate_s` (conditional on post-gap
arrival) and decides nothing.

## Validate against the mock (reproduces the paper's Section 7)

```
python validation/run_validation_v013.py --out validation/v0.1.3/mock          # ~1 h, private ROS master on :11611
python validation/run_validation_v015.py --out validation/v0.1.5/mock          # the same design with the 0.1.5 probe (seeds +30000) plus experiment V, :11615
python validation/reanalyze_liveness_v015.py                                   # the archived v0.1.3 liveness intervals under the 0.1.5 rules (offline)
# RC9 / 0.1.6 (pre-registered: validation/v0.1.6/PREREGISTRATION.md; environment: validation/v0.1.6/environment/)
python validation/dvrk_sim_v016.py campaign --launches 3                       # released dVRK 2.4.0 console, kinematic simulation (inside the dVRK image)
python validation/src_live_v016.py run v1|v2                                   # SRC releases on AMBF with the geometry anchor
python validation/run_validation_v016.py --only K,B,L                          # reference node: command-side mismatch, boundary, command loss
python validation/boundary_montecarlo_v016.py                                  # offline Monte Carlo of the released decision rules near epsilon
python validation/reanalyze_liveness_v016.py; python validation/rescore_v016.py; python validation/analyze_v016.py all
python validation/verify_reporting_v016.py --paper <manuscript dir> --tag rc12  # paper numbers recomputed from the raw v0.1.6 archive
# 0.1.7 (post hoc rule changes; the v0.1.6 campaign scripts above pin the 0.1.6 rules)
python validation/rederive_v017.py                                             # offline re-derivation under the 0.1.7 rules -> validation/v0.1.7/
python validation/run_validation_v017.py --only K,L                            # pre-registered confirmatory campaign of 0.1.7 (validation/v0.1.7/PREREGISTRATION.md)
# 0.1.8 (pre-registered: validation/v0.1.8/PREREGISTRATION.md; results: validation/v0.1.8/RESULTS.md)
python validation/run_validation_v018.py F                                     # reference node: correlation guard under AR(1) noise
python validation/run_validation_v018.py D --launches 3                        # dVRK console: frame and joint-space anchor controls
python validation/run_validation_v018.py S v1|v2 --launches 3                  # SRC releases: joint-space anchor, resting traces
python validation/analyze_v018.py; python validation/src_traces_v018.py; python validation/rederive_single_joint_S_v018.py   # the last is exploratory
python validation/frame_guard_study_v018.py; python validation/operating_characteristic_v018.py
python validation/anchor_poe_study_v018.py; python validation/deadline_resolution_v018.py   # offline studies -> validation/v0.1.8/studies/
python validation/tables_v018.py <dir>; python validation/fig_v018.py --figdir <dir>
python validation/verify_reporting_v016.py --paper <manuscript dir> --tag rc14  # paper numbers recomputed from the raw archives
python validation/analyze_v013.py validation/v0.1.3/mock --live validation/v0.1.3/live-src-v1 validation/v0.1.3/live-src-v2
```

The analyzer needs no ROS (numpy, scipy, matplotlib). `validation/v0.1.3/mock/` holds the archived outputs of the
run reported in the paper (`meta.json`: commit, interpreter, package versions, container, seeds, commands; one
`*.events.jsonl` per temporal run — the mock's own record of every command received, applied, dropped, ignored,
superseded or discarded, from which `analyze_v013.py` derives the rate truth independently of the estimator);
`validation/v0.1.3/live-src-v1/` and `live-src-v2/` hold the runs against the released Surgical Robotics Challenge
v1.0.0 and v2.0.0 CRTK interfaces on AMBF (`validation/environment/run_live.sh`). `validation/v0.1.2/` (RC4),
`validation/v0.1.1/` (RC3) and `validation/archive-v0.1.0/` (RC1/RC2) are the historical archives, unchanged;
their analysis scripts (`analyze_v012.py`, `analyze_v011.py`, `analyze.py`) are kept as used.

## Mock presets

`crtk-mock --list-presets`. Each preset is an *emulation of documented behaviour* assembled from ledger-cited
source lines (`src/crtk_mock/presets.py`); running a probe against a preset measures the probe, not the platform.

## Equation numbering

Equation numbers in source comments follow the manuscript: (1) positional error, (2) two-sided bound, (3)
operator-norm upper bound, (4) axis decomposition, (5) incremental error, (6) unit-scale error, (7)
bounded-jitter rate, (8) Gaussian rate margin, (9) zero-order-hold lag, (10) composition. The v0.1.0 archive's
`decision_basis` strings use the draft numbering of that release ("eq. (4)" = (7), "eq. 6" = (9),
"eq. (M2.1)" = (6)).

## Layout

```
src/crtk_conformance/   adapter.py, expectations.py, rate_estimator.py, probes/, thresholds.py, stats.py, geometry.py, report.py, cli.py
src/crtk_conformance/   ... spatial.py (eq. 3′, Hotelling interval)
src/crtk_mock/          config.py (ROS-free MockConfig), node.py (mock, rospy), presets.py, cli.py
tests/                  pytest (unit + integration); tests/adversarial/ = the RC3 reviewer's counterexample script and results, as supplied
validation/             harness.py, run_validation_v012.py, analyze_v012.py, environment/, v0.1.2/, v0.1.1/, archive-v0.1.0/ (+ the older run/analyze scripts, kept as used)
```

## License

Apache-2.0. See `LICENSE`.
