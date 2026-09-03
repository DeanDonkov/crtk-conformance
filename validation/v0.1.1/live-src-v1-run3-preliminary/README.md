# Preliminary live run 3 against SRC v1.0.0 (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance 804cf38. With the temporal step scaled to the resting noise the state
precondition sub-probe now observed `executed_without_state_machine = True` and the scale probe gave an
internal ratio of 0.87 (95 % CI 0.73–1.01), but the liveness classifier's `responded` criterion ("moved by
more than the hold tolerance") was still unreliable on this gravity-loaded arm: in run A most post-gap trials
were classified `hold` (correct) and one `rejected`; in run B, where the arm's jitter had grown to
sigma_hat = 7.6e-3 units and the hold tolerance to 0.076 units, every trial was classified `rejected`
although the arm moved 0.045–0.073 units toward each goal. Fixed in the next commit: `responded` is now
"the settled distance to the goal decreased by more than max(3 sigma_hat, 0.2 x initial distance)". Run
stopped after run B. Nothing in this directory was edited after the run.
