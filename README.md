# crtk-conformance

Black-box **semantic** conformance probes for CRTK/ROS surgical-robot interfaces, plus
**crtk-mock**, a configurable CRTK-compatible reference node with injectable semantic divergences.

Companion code for the manuscript *Structural Conformance Is Not Semantic Conformance: An Error Model
and Black-Box Test Suite for CRTK/ROS Interfaces in Surgical Robotics* (Donkov, 2026). Version 0.1.1
(see `CHANGELOG.md`).

Two implementations of the same CRTK topic names and message types can still interpret a
`PoseStamped` in different frames, in different units, or under different timing and state
preconditions. The probes estimate each of these **semantic bindings** through the public ROS
interface only and report a three-valued outcome per class.

## The semantic rule

> `conformant` means that the discovered binding satisfies an **explicitly declared client expectation**
> within the client's stated task-space tolerance. `divergent` means the declared expectation is violated
> beyond the tolerance. `undetermined` is returned whenever the interface does not expose what the decision
> needs, whenever data are missing or ambiguous, whenever the confidence interval straddles the tolerance —
> and whenever **no expectation was declared**: with no expectation there is no conformance verdict, only
> a report of what was observed.

The expectations are a small YAML file (`--expectations`):

```yaml
spatial:
  mode: expected_transform        # identity | expected_transform | discover_only (default)
  translation_m: [0.20, 0.0, 0.0] # the base_frame-like transform a measured_cp / local/measured_cp pair shows
  quaternion_xyzw: [-0.9659258, 0, 0, 0.2588190]
dimensional:
  mode: si                        # si | discover_only (default); 'si' needs --anchor-topic
  expected_unit_m: 1.0            # metres per interface unit the client assumes
temporal:
  state_machine: required         # required | forbidden | any (default)
  stop_behaviour: hold            # hold | release | fault | any (default)
  rate: required                  # required | any (default)
```

## What is and is not implemented (0.1.1)

| Implemented | Not implemented |
|---|---|
| ROS 1 (`rospy`) transport; discovery through the ROS master API | ROS 2 (no `rclpy` backend) |
| `FrameSemanticsProbe` — binding of the unqualified `measured_cp` relative to `local/measured_cp`, decided against a declared expected transform; **passive** (never publishes `servo_cp`) | inferring the binding when `local/` is absent (undetermined by construction); TF beyond a one-hop `/tf` lookup |
| `ScalingUnitsProbe` — internal command/measurement ratio, and a unit estimate **only with an out-of-band anchor topic**; noise-adaptive step; no-response accounting | detecting a uniform unit scale without an anchor (impossible; paper Sec. 5.2) |
| `RateSensitivityProbe` — operating-state precondition; liveness / stop-behaviour probe with measured timing resolution and stop classes (hold / release / rejected / fault / not observable / no policy within range); noise-robust observable-rate estimator with achieved-client-rate and publish-rate measurement | identifying the internal controller rate (not identifiable through the interface); measuring the platform's own jitter |
| JSON report validated against `schema/report.schema.json`; text summary; every outcome-affecting constant recorded | PDF reports |
| `crtk-mock` with presets emulating *documented* behaviours (built from cited configuration values) | any emulation of dVRK/AMBF/SRC *code* |
| Unit tests (no ROS; 39) + integration tests (ROS 1; 22) | hardware tests of any kind |

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
    --expectations expectations.yaml [--anchor-topic /tracker/tool_pose] --out report.json
```

## Task thresholds versus implementation constants

Every **decision threshold** is derived from the user's inputs (tolerance ε, workspace radius r_ws, speed v,
client rate and jitter bound) through the paper's error model: eq. (3) for the spatial residual, eq. (6) for
the unit scale, eq. (7) and eq. (9) for the temporal class. Nothing about ε is hard-coded.

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

`--trials` (default 10): each estimate is the mean of *n* independent trials with a Student-t 95 %
confidence interval; at n = 10 the half-width is about 0.72 sample standard deviations.

## Validate against the mock (reproduces the paper's Section 7)

```
python validation/run_validation_v011.py --out validation/v0.1.1/mock          # ~1 h, private ROS master on :11611
python validation/analyze_v011.py validation/v0.1.1/mock --live validation/v0.1.1/live-src-v1 validation/v0.1.1/live-src-v2
```

`validation/v0.1.1/mock/` holds the archived outputs of the run reported in the paper (`meta.json`: commit,
interpreter, package versions, container, seeds, commands). `validation/v0.1.1/live-src-v1/` and
`live-src-v2/` hold the runs against the released Surgical Robotics Challenge v1.0.0 and v2.0.0 CRTK
interfaces on AMBF (`validation/environment/run_live.sh`). `validation/archive-v0.1.0/` is the historical
v0.1.0 archive, unchanged.

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
src/crtk_mock/          node.py (mock), presets.py, cli.py
tests/                  pytest (unit + integration)
validation/             harness.py, run_validation_v011.py, analyze_v011.py, environment/, v0.1.1/, archive-v0.1.0/ (+ run_validation.py, analyze.py for the old archive)
```

## License

Apache-2.0. See `LICENSE`.
