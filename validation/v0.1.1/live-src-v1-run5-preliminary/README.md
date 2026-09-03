# Preliminary live run 5 against SRC v1.0.0 (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance ad78e5c. The response-latency measurement now works (10 of 10 probes
attained, median 74 ms, p95 91 ms) and the state-precondition sub-probe observed `executed_without_state_machine
= True`, but the gap trials of the liveness sub-probe commanded their post-gap step along +z of the arm-base
frame, in which this arm barely moves at its home configuration (its insertion joint sits near the lower
limit of `enforce_limits`, j3 in [0, 2.40]); the distance to the goal decreased by only 2–6 % and every trial
was classified `rejected` in run B, while the identical trials in run A of the previous attempt had reduced it
by 30 %. Fixed in the next commit: the resolution measurement tries all six axis directions and the gap and
state trials use the best-tracked one. Run stopped after run B. Nothing in this directory was edited after
the run.
