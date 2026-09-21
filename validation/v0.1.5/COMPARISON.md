# v0.1.5 verification campaign versus the v0.1.3 archive (reference node; same design, seeds +30000)

v0.1.3: 0.1.3 at 1c06238, 2026-09-10, wall 64.2 min; v0.1.5: 0.1.5 at ad2232d (working tree with the 0.1.5 patch), 2026-09-19, wall 73.1 min; image sha256:a11b7b278af2b3b8feb3315f384c389844d86a2ae60efe184026b096235cb644.

## Outcomes of the 276 runs common to both campaigns (archive under the current rules)

Identical outcome in 273 of 276 runs.

| run | v0.1.3 outcome | v0.1.5 outcome |
|---|---|---|
| F_rev_par_2 | undetermined | conformant |
| S_si_020 | divergent | undetermined |
| R_noisy | undetermined/undetermined/undetermined | undetermined/conformant/undetermined |

## Frame sweep (F_id)

v0.1.3: 120/120 intervals contain the exact maximum error; translation error at 0.1 mm noise mean/max 0.039/0.087 mm.
v0.1.5: 120/120; mean/max 0.036/0.075 mm.

## Spatial coverage replication (C_cov)

| configuration | v0.1.3 contain | v0.1.3 C/D/U | v0.1.5 contain | v0.1.5 C/D/U |
|---|---|---|---|---|
| C_cov_boundary | 60/60 | 0/0/60 | 60/60 | 0/0/60 |
| C_cov_large | 60/60 | 0/60/0 | 60/60 | 0/60/0 |
| C_cov_rotation | 60/60 | 48/0/12 | 60/60 | 56/0/4 |
| C_cov_zero | 60/60 | 60/0/0 | 60/60 | 60/0/0 |

## Anchored scale sweep (S_si)

v0.1.3: 27 runs, C/D/U 6/19/2, max relative error of s_hat 1.09 %.
v0.1.5: 27 runs, C/D/U 6/18/3, max relative error 0.66 %.

## Liveness (L)

v0.1.3: 34 runs, 23 formed, 23/23 contain the injected timeout, 22 conditional, median width 28.5 ms.
v0.1.5: 34 runs, 24 formed, 24/24 contain, 23 conditional, median width 33.3 ms.

| run | tau_w (s) | v0.1.3 interval (ms) / stop / outcome | v0.1.5 interval (ms) / stop / outcome | v0.1.5 departures observed | v0.1.5 r_i (ms) |
|---|---|---|---|---|---|
| L_beyond_range_fault | 2.0 | none / violated / divergent | none / violated / divergent | 5/5 | 1.1–1.8 |
| L_beyond_range_fault_horizon | 2.0 | none / undetermined / undetermined | none / undetermined / undetermined | 5/5 | 1.4–2.2 |
| L_beyond_range_hold | 2.0 | none / satisfied / conformant | none / satisfied / conformant | 5/5 | 1.3–1.9 |
| L_delayed | 0.5 | [489.0, 824.0] / satisfied / conformant | [188.1, 810.8] / satisfied / conformant | 40/45 | 300.9–311.4 |
| L_fault_000 | 0.012 | undetermined / undetermined / undetermined | undetermined / undetermined / undetermined | 35/38 | 0.8–10.7 |
| L_fault_001 | 0.015 | undetermined / undetermined / undetermined | undetermined / undetermined / undetermined | 44/45 | 1.0–10.9 |
| L_fault_002 | 0.02 | undetermined / undetermined / undetermined | undetermined / undetermined / undetermined | 43/45 | 1.2–11.5 |
| L_fault_003 | 0.03 | undetermined / undetermined / undetermined | [11.3, 44.9] / undetermined / undetermined | 44/45 | 1.0–11.0 |
| L_fault_004 | 0.05 | [29.8, 68.9] / satisfied / conformant | [34.5, 71.6] / satisfied / conformant | 45/45 | 0.9–11.4 |
| L_fault_005 | 0.1 | [80.7, 115.2] / satisfied / conformant | [80.8, 114.9] / satisfied / conformant | 45/45 | 1.0–10.7 |
| L_fault_006 | 0.25 | [231.3, 266.9] / satisfied / conformant | [232.1, 265.9] / satisfied / conformant | 45/45 | 1.1–10.7 |
| L_fault_007 | 0.5 | [481.5, 522.1] / satisfied / conformant | [488.0, 522.6] / satisfied / conformant | 44/45 | 0.8–11.3 |
| L_fault_008 | 1.0 | [988.0, 1022.9] / satisfied / conformant | [987.7, 1011.3] / satisfied / conformant | 42/45 | 1.0–11.6 |
| L_hold | 0.0 | none / satisfied / conformant | none / satisfied / conformant | 5/5 | 1.4–11.5 |
| L_hold_horizon_beyond | 0.0 | none / undetermined / undetermined | none / undetermined / undetermined | 5/5 | 1.6–11.4 |
| L_horizon_drift_0200 | 0.5 | [487.7, 502.2] / violated / divergent | [478.4, 501.1] / violated / divergent | 35/35 | 0.9–10.7 |
| L_horizon_drift_0500 | 0.5 | [475.9, 501.3] / undetermined / undetermined | [468.3, 501.0] / undetermined / undetermined | 35/35 | 0.9–11.3 |
| L_horizon_drift_1500 | 0.5 | [487.1, 501.6] / satisfied / conformant | [468.1, 501.1] / satisfied / conformant | 35/35 | 1.0–11.5 |
| L_horizon_fault_0100 | 0.25 | [231.0, 266.7] / violated / divergent | [232.0, 266.5] / violated / divergent | 45/45 | 1.0–11.5 |
| L_horizon_fault_0250 | 0.25 | [232.5, 267.3] / undetermined / undetermined | [232.0, 266.3] / undetermined / undetermined | 45/45 | 1.2–11.3 |
| L_horizon_fault_1000 | 0.25 | [232.1, 266.8] / satisfied / conformant | [232.4, 265.8] / satisfied / conformant | 45/45 | 0.8–10.7 |
| L_latency_bound | 0.25 | [231.5, 260.1] / satisfied / conformant | [232.3, 260.0] / satisfied / conformant | 45/45 | 0.8–11.1 |
| L_rc4_reviewer_horizon | 1.0 | [988.2, 1022.9] / violated / divergent | [988.1, 1011.0] / violated / divergent | 45/45 | 0.9–11.1 |
| L_release_000_drift | 0.1 | [78.4, 101.3] / satisfied / conformant | [68.2, 101.0] / satisfied / conformant | 30/30 | 1.2–11.6 |
| L_release_000_hold | 0.1 | [78.8, 101.6] / violated / divergent | [69.7, 101.3] / violated / divergent | 30/30 | 1.2–11.4 |
| L_release_001_drift | 0.1 | [80.2, 102.9] / satisfied / conformant | [77.2, 110.8] / satisfied / conformant | 24/25 | 0.8–10.8 |
| L_release_001_hold | 0.1 | [86.8, 104.1] / violated / divergent | [77.1, 101.8] / violated / divergent | 25/25 | 1.0–10.6 |
| L_release_002_drift | 0.5 | [479.1, 501.3] / satisfied / conformant | [469.6, 500.8] / satisfied / conformant | 35/35 | 0.7–10.9 |
| L_release_002_hold | 0.5 | [478.0, 501.4] / violated / divergent | [468.2, 501.4] / violated / divergent | 35/35 | 1.2–11.5 |
| L_release_003_drift | 0.5 | [486.5, 507.3] / satisfied / conformant | [476.5, 510.8] / satisfied / conformant | 25/25 | 1.1–11.6 |
| L_release_003_hold | 0.5 | [486.3, 507.7] / violated / divergent | [477.1, 502.0] / violated / divergent | 25/25 | 0.9–2.1 |
| L_release_nodrift_hold | 0.5 | none / satisfied / conformant | none / satisfied / conformant | 5/5 | 0.9–1.7 |
| L_release_nodrift_release | 0.5 | none / undetermined / undetermined | none / undetermined / undetermined | 5/5 | 1.6–2.4 |
| L_slowfb | 0.25 | [152.3, 372.6] / satisfied / conformant | [132.3, 372.1] / satisfied / conformant | 0/30 | none (L used) |

## Experiment V (0.1.5 verification cases; no v0.1.3 counterpart)

| run | injected | expectation | outcome | stop sub-verdict | timeout estimate | departures | r_i (ms) | calibration lost | confounded | finding |
|---|---|---|---|---|---|---|---|---|---|---|
| V_delayed_0 | F1: r_i measured from an observed departure; the 0.1.3 rule read the in-band stream | fault | conformant | satisfied | [438.2, 572.7] ms contains 500 ms | 40/45 | 51.2–61.6 | 0 | False | faulted policy detected: tau_w in [0.438, 0.573] s (midpoint 0.505 s, n=5; L 62 ms conditional, G 10 ms, t_det 0 ms) |
| V_delayed_1 | F1: r_i measured from an observed departure; the 0.1.3 rule read the in-band stream | fault | conformant | satisfied | [388.2, 610.9] ms contains 500 ms | 40/45 | 100.9–111.5 | 0 | False | faulted policy detected: tau_w in [0.388, 0.611] s (midpoint 0.500 s, n=5; L 111 ms conditional, G 10 ms, t_det 0 ms) |
| V_delayed_2 | F1: r_i measured from an observed departure; the 0.1.3 rule read the in-band stream | fault | conformant | satisfied | [187.8, 810.8] ms contains 500 ms | 40/45 | 301.3–311.5 | 0 | False | faulted policy detected: tau_w in [0.188, 0.811] s (midpoint 0.499 s, n=5; L 311 ms conditional, G 10 ms, t_det 0 ms) |
| V_delayed_drift_0 | F8: onset lower bound with G, delayed application | drift | conformant | satisfied | [351.3, 501.0] ms contains 500 ms | 30/35 | 101.0–111.5 | 0 | False | drifted policy detected: tau_w in [0.351, 0.501] s (midpoint 0.426 s, n=5; L 112 ms conditional, G 10 ms, t_det 60 ms) |
| V_delayed_drift_1 | F8: onset lower bound with G, delayed application | drift | undetermined | undetermined | undetermined | 35/40 | 300.9–311.8 | 0 | False | drifted observed but tau_w undetermined (interval lower bound not above the resolution floor; n=5) |
| V_drop_fault | F10: a fault visible in the operating state is not confounded by command loss | fault | conformant | satisfied | [232.0, 393.3] ms contains 250 ms | 32/45 | 1.1–11.6 | 5 | False | faulted policy detected: tau_w in [0.232, 0.393] s (midpoint 0.313 s, n=5; L 12 ms conditional, G 10 ms, t_det 0 ms) |
| V_drop_hold_0 | F10: no stop policy; non-responses are command loss; must be undetermined, not violated | hold | conformant | satisfied | None | 4/5 | 1.8–2.1 | 0 | None | held within 0.001 interface units for silences up to 1.5 s and responded afterwards (n=5); a stop policy with a longer timeout is not excluded |
| V_drop_hold_1 | F10: no stop policy; non-responses are command loss; must be undetermined, not violated | hold | undetermined | undetermined | undetermined | 14/31 | 1.3–9.1 | 6 | True | post-gap non-responses observed but tau_w undetermined (6 of 12 calibration commands sent without any preceding silence drew no response: post-gap non-responses |
| V_drop_reject | F10: a rejection policy under command loss is indistinguishable from the loss: undetermined (documented cost) | fault | undetermined | undetermined | undetermined (upper bound 20.3 ms) | 0/10 | none (L used) | 11 | True | no response at every gap down to the resolution floor, but 11 of 12 calibration commands sent without any preceding silence drew no response: post-gap non-respo |

## Rate windows (T)

v0.1.3: 36 runs / 180 windows; v0.1.5: 36 runs / 180 windows. The rate sub-verdict is undetermined in every window since v0.1.4; containment of the source-age diagnostic against the event log is computed by analyze_v013.py on each archive (tables/T_rate_summary.json).

v0.1.3 analyzer summary: windows = 180, scored = 160, unscored = 20, sound_determinate = 106, sound_undetermined = 54, false_conformant = 0, false_divergent = 0, completeness = 0.8760330578512396, determinate_truth_windows = 121, determinate_verdicts_on_them = 106, undetermined_on_client_limited = 39, bracket_windows = 150, bracket_contains_truth = 98, excess_lower_ms_max = 8.6012000010669, excess_lower_ms_p95 = 0.8760444505696742, crossings_invariant_violations = 0, required_rate_hz = 50.0.
v0.1.5 analyzer summary: windows = 180, scored = 165, unscored = 15, sound_determinate = 114, sound_undetermined = 51, false_conformant = 0, false_divergent = 0, completeness = 0.8837209302325582, determinate_truth_windows = 129, determinate_verdicts_on_them = 114, undetermined_on_client_limited = 36, bracket_windows = 155, bracket_contains_truth = 117, excess_lower_ms_max = 0.8576860000175657, excess_lower_ms_p95 = 0.23325850124820097, crossings_invariant_violations = 0, required_rate_hz = 50.0.

