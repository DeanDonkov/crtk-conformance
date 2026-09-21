# Validation plan, crtk-conformance 0.1.5 (RC8)

Purpose: verify on fresh measurements the three liveness changes made after the RC7 adversarial review
(findings F1, F8, F10; see `CHANGELOG.md` 0.1.5), and confirm that nothing else changed.  The v0.1.3 mock and
live archives (`validation/v0.1.3/`) remain the measurement set reported in the manuscript; they are not
modified.  Their liveness intervals are re-derived offline under the 0.1.5 rules by
`validation/reanalyze_liveness_v015.py` (output in `reanalysis/`), in the same way the v0.1.4 relabel was
applied to the archived rate verdicts.

## Runs

| Part | Script | Location | What it establishes |
|---|---|---|---|
| tests | `run_tests_v015.sh` (unit: `pytest tests --ignore=tests/test_integration.py`; integration: `pytest tests/test_integration.py` against the pure-Python ROS 1 stack in `docs/ros1-stack.md`, and again inside `focal-crtk:rc3`) | `tests/` | 77 unit tests including `tests/test_liveness_v015.py` (departure guard, G in the onset bound, fallback to L, archived replays, R_drop_50 replay); 35 integration tests, three of which now assert the v0.1.4/0.1.5 rate verdict (undetermined) and the archival relabel |
| mock | `run_validation_v015.py` = `run_validation_v013.py` experiments F, C, S, T, L, P, R, X, M with seeds offset by 30000, plus experiment V | `mock/` | the 0.1.3 campaign design reproduced by the revised probe (agreement with the v0.1.3 archive is analysed by `analyze_v013.py` on the new directory); experiment V isolates the three changes |
| live | `run_live.sh v1` / `v2` (unchanged) | `live-src-v1/`, `live-src-v2/` | the revised probe against the two released SRC CRTK interfaces (AMBF `ambf-2.0` @16a81518, SRC v1.0.0 @158b554e, v2.0.0 @03befbf1), the same three expectation files as v0.1.3 |

## Experiment V (0.1.5 verification)

| Run | Injected | Expectation | 0.1.3 behaviour | 0.1.5 requirement |
|---|---|---|---|---|
| `V_delayed_{0,1,2}` | fault, tau_w = 0.5 s, every command applied 50 / 100 / 300 ms late | fault | the last streamed command's response latency r_i was read from the still-executing in-band stream (0.2–9.7 ms in `L_delayed`), so the passing lower bounds were too high by about the delay | r_i is measured only from an observed departure (`last_stream_departure_observed`); when none is seen, L is used; the interval must contain 0.5 s |
| `V_delayed_drift_{0,1}` | release with 20 mm/s drift, tau_w = 0.5 s, commands applied 100 / 300 ms late | drift | onset lower bound without the granularity term G | onset lower bound o − r_i − G − thr/v_min − P − L; the interval must contain 0.5 s |
| `V_drop_hold_{0,1}` | no stop policy, 20 % / 50 % of commands dropped at random | hold within 0.5 s | `R_drop_50`: 3 of 4 unanswered post-gap commands read as a rejection policy, hold expectation violated | the calibration probes show baseline command loss, every tripped trial is a non-response: `rejection_confounded_by_command_loss`, stop sub-verdict undetermined |
| `V_drop_fault` | fault, tau_w = 0.25 s, 30 % drops, state machine on | fault | — | the tripping class is `faulted` (operating state), which loss cannot produce: verdict formed, interval contains 0.25 s |
| `V_drop_reject` | the same policy without the state machine (tripping class `rejected`), 30 % drops | fault | — | indistinguishable from loss under the 0.1.5 rule: undetermined (the documented cost of the rule) |

## Environment

`focal-crtk:rc3` rebuilt from `validation/environment/` (debootstrap focal → toolchain → ROS Noetic from the
pinned sources → runtime wheels → AMBF/SRC); the image digest is recorded in every `meta.json` /
`meta_env.json`.  Two vCPUs; the mock campaign, the live runs and the tests are run sequentially.
