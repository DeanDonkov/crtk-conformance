# crtk-conformance 0.1.7

Two decision-rule changes on top of 0.1.6, made after the pre-registered v0.1.6 campaigns in response to an external
review of the RC11 manuscript. They make the tool follow its own rule that missing evidence yields **undetermined**:

1. **A raised command/feedback consistency diagnostic withholds the unit verdict.** A unit verdict assumes that
   commands and feedback share one binding. When the scale probe's diagnostic shows that they do not, 0.1.6 still
   reported a unit verdict, and in case K1 it reported a command-frame error as a unit error. 0.1.7 returns
   undetermined, and the report's new `verdict_kinds` separates:
   - the frame probe's feedback-binding verdict;
   - its command-semantic reading, which is withheld when the assumption is contradicted.
2. **Fault timing bounds are sound under command loss.** Each faulted trial now bounds the timeout by the time its
   FAULT was observed, not by the gap plus a latency allowance. The old bound excluded the true timeout in one run of
   the loss experiment, because a lost post-gap command let the fault fire during the response wait. The tighter
   0.1.6 interval is still reported, as `conditional_estimate_s`, and decides nothing. The cost is width: about the
   probe's response wait.

`--liveness-rule 0.1.6` and `--no-consistency-gate` reproduce 0.1.6, and the v0.1.6 campaign scripts pin them.

## Evidence

Everything was re-derived offline from the archive (`validation/v0.1.7/`, `validation/rederive_v017.py`); no run was
repeated. These results are **post hoc**.

| Set | 0.1.6 | 0.1.7 |
|---|---|---|
| Runs carrying the consistency diagnostic | 102 | only the 3 K1 runs flagged; their unit verdict changes from divergent to undetermined |
| Fault-policy loss runs: intervals containing τ_w | 53 of 54 formed, 10 contradictory | 64 of 64 |
| Median width without loss | 45 ms | 334 ms |
| Stop sub-verdicts | 54 satisfied, 10 undetermined | 64 satisfied, all correct |
| Archived v0.1.3 intervals containing τ_w | 23/23 | 23/23 |
| Median width/τ_w (archived) | 0.14 | 0.53 (the older records bound faults by the response wait) |

The archived fault-horizon claims are unchanged: within 0.1 s violated, 0.25 s undetermined, 1 s satisfied.

## Tests

- New: `tests/test_v017.py`.
- Full ROS suite at the release commit: see `pytest_final_<commit>.log` in the RC12 package.

Known intermittent timing tests are as in 0.1.6 (`validation/v0.1.6/tests/README.md`).

## Also

Everything in 0.1.6 (`docs/release-notes-0.1.6.md`): the instrument-geometry anchor, the liveness confirmation rule,
one state command at a time, the consistency diagnostic, and the v0.1.6 pre-registered evidence.
