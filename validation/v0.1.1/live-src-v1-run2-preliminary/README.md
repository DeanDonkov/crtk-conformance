# Preliminary live run 2 against SRC v1.0.0 (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance 01750a4. The scale probe's noise-adaptive step worked here (internal ratio
0.89, 95 % CI 0.78–1.00, n = 10, step 0.056 interface units), but the temporal probe still used a fixed
0.002-unit step, far inside this instance's resting jitter (sigma_hat 5.6e-3 units), so its post-gap
attainment test and stop-behaviour classification were noise (trials classified alternately `rejected`
and `hold`, a meaningless tau_w of 0.83 s with CI 0.24–1.43 s). Fixed in the next commit: the temporal step
is at least 20 x the resting noise, as in the scale probe. The run was stopped after run A.
Nothing in this directory was edited after the run.
