# Test runs of the RC9 branch (inside `focal-rc9:live`, ROS Noetic)

| Log | Code | Result |
|---|---|---|
| `pytest_ffd9ade.log` | `ffd9ade` (0.1.6.dev0) | 133 passed |
| `pytest_addendumA.log` | the commit that adds pre-registration addendum A | 135 passed, 1 failed: `test_fifo_queue_slower_than_the_client_is_violated_and_logged` |

## The intermittent failures

Two timing tests fail intermittently in this 2-vCPU container:

- `test_rate_accepted_channel_satisfied_on_reference`
- `test_fifo_queue_slower_than_the_client_is_violated_and_logged`

Both depend on the probe's own 100 Hz send loop not stalling. Under pytest that loop was seen to stall for 50–60 ms (`client_max_send_gap_s`), about 0.5 s into the rate window. The same stall occurred with the 0.1.5 `ensure_enabled`, so this is not a regression of addendum A.

Repeat runs of the pair:

- Addendum-A code: failed, passed, passed.
- `test_rate_accepted_channel_satisfied_on_reference` alone, with the 0.1.5 `common.py`: failed, passed.

As a standalone process, not under pytest, the same probe call showed no stall in 5 of 5 runs. The campaigns run the CLI as a standalone process.
