# validation/v0.1.7: offline re-derivation under the 0.1.7 decision rules (RC12)

**POST HOC.** The 0.1.7 rules were changed after the pre-registered v0.1.6 campaigns, in response to an external review of the RC11 manuscript. The campaigns remain reported under their own (0.1.6) rules. **No run was repeated or added.** Every input is an archived probe report in `validation/v0.1.6/` or `validation/v0.1.3/`.

`python3 validation/rederive_v017.py` writes:

| File | Contents |
|---|---|
| `consistency_gate_v017.csv` | Every archived run carrying the scale probe's command/feedback consistency diagnostic: K (9), B scale (90), dVRK-sim (3). For each: the flag, and the unit verdict under 0.1.6 and under 0.1.7. |
| `L_archived_v017.csv` | The 23 archived v0.1.3 liveness intervals under 0.1.6 and 0.1.7, and the fault-horizon claims. |
| `rederivation_v017.json` | Summary, plus the 64 fault-policy runs of the loss experiment under 0.1.7. |

## The two rule changes

1. **Consistency gate** (`dimensional.consistency_gate`; `report.verdict_kinds`). A raised command/feedback consistency diagnostic contradicts the shared-binding assumption that a unit verdict needs, so the unit verdict is withheld (undetermined). The report also withholds the command-semantic reading of the frame verdict. The frame verdict itself stays, as a feedback-binding verdict.
2. **Sound fault bound** (`liveness.trip_upper_bound`, rule `"0.1.7"`; `probes/rate.py`). Every faulted trial bounds τ_w by the time its FAULT was observed. That bound is sound whether or not the post-gap command arrived. The 0.1.6 bound (gap + L) is reported as an estimate conditional on arrival and decides nothing.
   - The v0.1.3 records carry no observation time. They are bounded by gap + W + 0.2 s + G (`liveness.fault_observation_window`), where W is the run's response wait, 0.2 s the state query and G one feedback period.

## Results

- **Consistency gate.** Of 102 runs carrying the diagnostic, only the 3 K1 runs are flagged. Their unit verdict changes from divergent to undetermined; nothing else changes.
- **Loss experiment, 64 fault-policy runs.**
  - 0.1.7 forms 64 intervals and all 64 contain τ_w. Under 0.1.6: 54 formed, 53 contain, and 10 were contradictory.
  - Median width without loss: 45 ms under 0.1.6, 334 ms under 0.1.7.
  - Stop sub-verdicts (declared fault, horizon 2 s): 54 satisfied under both rules; the 10 that were undetermined under 0.1.6 are satisfied under 0.1.7. All are correct.
  - A horizon of 0.245 s, which is false for τ_w = 0.25 s: 0.1.6 would have decided it satisfied in one run (`L16_016_fault250_iid5_03`); 0.1.7 decides it satisfied in none.
- **Archived v0.1.3 intervals.**
  - 23 formed and 23 contain τ_w under 0.1.7, as under 0.1.6.
  - Width/τ_w: median 0.14 under 0.1.6; under 0.1.7, median 0.53 (fault 2.13, drift 0.07, range 0.05–10.7). The widening comes from the response-wait bound on these older records.
  - The fault-horizon claims are unchanged: 0.1 s violated, 0.25 s undetermined, 1 s satisfied, and the RC4 reviewer's case violated.
