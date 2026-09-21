# Archived v0.1.3 liveness campaign under the 0.1.5 rules (offline re-derivation; archive unchanged)

Runs: 34; intervals with status ok in 0.1.3: 23; containing the injected timeout: 0.1.3 23/23, 0.1.5 23/23; median width 0.1.3 28.5 ms, 0.1.5 34.6 ms; lower ends changed in 12 intervals; r_i replaced by L in 45 trials of 1 runs; trials with the pose already at the post-gap goal (not observable under 0.1.5): 0; held trials of drift-policy runs whose silence did not outlast the reference window (no passing bound under 0.1.5): 55, all at the resolution floor (largest such gap 11.2 ms, where the passing bound is negative).

| run | tau_w (s) | class | 0.1.3 interval (ms) | 0.1.5 interval (ms) | lower-end change (ms) | contains (0.1.3 / 0.1.5) | note |
|---|---|---|---|---|---|---|---|
| L_beyond_range_fault | 2.0 | held_through_range | none | None | — | — |  |
| L_beyond_range_fault_horizon | 2.0 | held_through_range | none | None | — | — |  |
| L_beyond_range_hold | 2.0 | held_through_range | none | None | — | — |  |
| L_delayed | 0.5 | faulted | [489.0, 824.0] | [176.4, 824.0] | -312.6 | True / True | r_i below the run's response floor (306.5 - 10.0 ms) in 45 trials: replaced by L = 313.0 ms |
| L_fault_000 | 0.012 | faulted | undetermined | None | — | — |  |
| L_fault_001 | 0.015 | faulted | undetermined | None | — | — |  |
| L_fault_002 | 0.02 | faulted | undetermined | None | — | — |  |
| L_fault_003 | 0.03 | faulted | undetermined | None | — | — |  |
| L_fault_004 | 0.05 | faulted | [29.8, 68.9] | [29.8, 68.9] | +0.0 | True / True |  |
| L_fault_005 | 0.1 | faulted | [80.7, 115.2] | [80.7, 115.2] | +0.0 | True / True |  |
| L_fault_006 | 0.25 | faulted | [231.3, 266.9] | [231.3, 266.9] | +0.0 | True / True |  |
| L_fault_007 | 0.5 | faulted | [481.5, 522.1] | [481.5, 522.1] | +0.0 | True / True |  |
| L_fault_008 | 1.0 | faulted | [988.0, 1022.9] | [988.0, 1022.9] | +0.0 | True / True |  |
| L_hold | 0.0 | held_through_range | none | None | — | — |  |
| L_hold_horizon_beyond | 0.0 | held_through_range | none | None | — | — |  |
| L_horizon_drift_0200 | 0.5 | drifted | [487.7, 502.2] | [477.7, 502.2] | -10.0 | True / True | onset lower bound now includes G |
| L_horizon_drift_0500 | 0.5 | drifted | [475.9, 501.3] | [465.9, 501.3] | -10.0 | True / True | onset lower bound now includes G |
| L_horizon_drift_1500 | 0.5 | drifted | [487.1, 501.6] | [477.1, 501.6] | -10.0 | True / True | onset lower bound now includes G |
| L_horizon_fault_0100 | 0.25 | faulted | [231.0, 266.7] | [231.0, 266.7] | +0.0 | True / True |  |
| L_horizon_fault_0250 | 0.25 | faulted | [232.5, 267.3] | [232.5, 267.3] | +0.0 | True / True |  |
| L_horizon_fault_1000 | 0.25 | faulted | [232.1, 266.8] | [232.1, 266.8] | +0.0 | True / True |  |
| L_latency_bound | 0.25 | faulted | [231.5, 260.1] | [231.5, 260.1] | +0.0 | True / True |  |
| L_rc4_reviewer_horizon | 1.0 | faulted | [988.2, 1022.9] | [988.2, 1022.9] | +0.0 | True / True |  |
| L_release_000_drift | 0.1 | drifted | [78.4, 101.3] | [68.4, 101.3] | -10.0 | True / True | onset lower bound now includes G |
| L_release_000_hold | 0.1 | drifted | [78.8, 101.6] | [68.8, 101.6] | -10.0 | True / True | onset lower bound now includes G |
| L_release_001_drift | 0.1 | drifted | [80.2, 102.9] | [70.2, 102.9] | -10.0 | True / True | onset lower bound now includes G |
| L_release_001_hold | 0.1 | drifted | [86.8, 104.1] | [76.8, 104.1] | -10.0 | True / True | onset lower bound now includes G |
| L_release_002_drift | 0.5 | drifted | [479.1, 501.3] | [469.1, 501.3] | -10.0 | True / True | onset lower bound now includes G |
| L_release_002_hold | 0.5 | drifted | [478.0, 501.4] | [468.0, 501.4] | -10.0 | True / True | onset lower bound now includes G |
| L_release_003_drift | 0.5 | drifted | [486.5, 507.3] | [476.1, 507.3] | -10.4 | True / True | onset lower bound now includes G |
| L_release_003_hold | 0.5 | drifted | [486.3, 507.7] | [476.3, 507.7] | -10.0 | True / True | onset lower bound now includes G |
| L_release_nodrift_hold | 0.5 | held_through_range | none | None | — | — |  |
| L_release_nodrift_release | 0.5 | held_through_range | none | None | — | — |  |
| L_slowfb | 0.25 | faulted | [152.3, 372.6] | [152.3, 372.6] | +0.0 | True / True |  |
| R_drop_50 | None | rejected | upper_bound | undetermined (confounded) | — | — | 0.1.3 stop sub-verdict violated; 0.1.5: undetermined -- 7 of 12 calibration commands unanswered, no stop policy injected |
| R_delay_50ms_jitter_20ms | None | held_through_range | None | None | — | — | unchanged |
| R_noisy | None | held_through_range | None | None | — | — | unchanged |
| R_nolocal | None | held_through_range | None | None | — | — | unchanged |
