# crtk-conformance

Black-box **semantic** conformance probes for CRTK/ROS surgical-robot interfaces, plus
**crtk-mock**, a configurable CRTK-compatible reference node with injectable semantic divergences.

Companion code for the preprint *Structural Conformance Is Not Semantic Conformance: An Error Model
and Black-Box Test Suite for CRTK/ROS Interfaces in Surgical Robotics* (Donkov, 2026).

Two implementations of the same CRTK topic names and message types can still interpret a
`PoseStamped` in different frames, in different units, or under different timing and state
preconditions. The probes estimate each of these **semantic bindings** through the public ROS
interface only, report a three-valued outcome (`conformant` / `divergent` / `undetermined`) against
a task-space tolerance the user states, and never adapt commands behind the user's back.

## What is and is not implemented (v0.1.0)

| Implemented | Not implemented |
|---|---|
| ROS 1 (`rospy`) transport; discovery through the ROS master API | ROS 2 (planned; no `rclpy` backend exists in this release) |
| `FrameSemanticsProbe` — binding of the unqualified `measured_cp`/`servo_cp` relative to `local/` | any use of TF beyond a one-hop `/tf` lookup for information |
| `ScalingUnitsProbe` — internal command/measurement ratio, and a unit estimate **only when an out-of-band anchor topic is supplied** | detecting a uniform unit scale without an anchor (impossible; see paper Sec. 5.2) |
| `RateSensitivityProbe` — operating-state precondition, command-liveness timeout estimate, effective command rate | measuring jitter of the platform; anything about dynamics or contact |
| JSON report validated against `schema/report.schema.json`; text summary | PDF reports |
| `crtk-mock` with presets emulating documented behaviours | any emulation of dVRK/AMBF/SRC *code* — presets are built from cited configuration values only |
| Unit tests (no ROS needed) + integration tests (ROS 1 needed) | hardware tests of any kind |

Three probe families are implemented, one per binding class. The temporal probe has three
sub-probes (state precondition, liveness, effective rate). There are no other tests.

## Install

```
pip install -e .            # library + CLIs (numpy, scipy, jsonschema, PyYAML)
pip install -e .[dev]       # + pytest, matplotlib
```

ROS 1 must be available to the interpreter: `rospy`, `rosgraph`, `geometry_msgs`, `sensor_msgs`,
`tf2_msgs` and `crtk_msgs` (https://github.com/collaborative-robotics/crtk_msgs). A ROS Noetic
installation provides all of them. For a machine without ROS, `docs/ros1-stack.md` describes the
pure-Python stack (rospy, rosmaster and generated messages built from the ROS GitHub sources)
that the validation in the paper was run with.

## Run against a platform

```
crtk-mock --preset emul-dvrk-jhu-psm2 &                       # or your real arm
crtk-conformance run --namespace /PSM1 --tolerance-mm 1.0 --workspace-radius-m 0.10 \
    --speed-mm-s 50 --client-rate-hz 100 --jitter-max-ms 5 --trials 10 \
    [--anchor-topic /tracker/tool_pose] --out report.json
```

Every decision threshold is derived from the four user inputs (tolerance, workspace radius, speed,
client rate/jitter) through the paper's error model; nothing is hard-coded. The `--anchor-topic`
is a `geometry_msgs/PoseStamped` topic carrying an independent SI measurement of the same tool
(an external tracker). Without it the dimensional outcome is `undetermined` by construction.

`--trials` (default 10): each estimate is the mean of *n* independent trials with a Student-t
95 % confidence interval. At n = 10 the half-width is about 0.72 sample standard deviations,
which separates divergences an order of magnitude above the measurement noise while keeping a
full run under a minute; raise it for finer floors.

## Validate against the mock (reproduces the paper's Section 7)

```
python validation/run_validation.py --out validation/results     # ~25 min, private ROS master on :11611
python validation/analyze.py validation/results                   # tables + figures
```

`validation/results/` in this repository holds the archived outputs of the run reported in the
paper (`meta.json` records the commit, interpreter and platform).

## Mock presets

`crtk-mock --list-presets`. Each preset is an *emulation of documented behaviour* assembled from
cited source lines (see `src/crtk_mock/presets.py`); running a probe against a preset measures the
probe, not the platform.

## Layout

```
src/crtk_conformance/   adapter.py (ROS 1 discovery/IO), probes/, thresholds.py, stats.py, geometry.py (error model), report.py, cli.py
src/crtk_mock/          node.py (mock), presets.py, cli.py
tests/                  pytest (unit + integration)
validation/             harness.py, run_validation.py, analyze.py, results/
```

## License

Apache-2.0. See `LICENSE`.
