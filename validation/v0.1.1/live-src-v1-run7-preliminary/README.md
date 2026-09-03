# Preliminary live run 7 against SRC v1.0.0, from the working configuration (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance ab465e0. First run from the working configuration (insertion 1.0 interface
units). Runs B and C classified the stop behaviour as `no stop policy detected up to 2.0 s` (hold), but run A
classified it as `release` below resolution: its drift metric was the distance from the *last streamed
setpoint* to the farthest sample of the gap, so the arm's steady-state tracking error after the stream
(0.041 units against a hold tolerance of 0.037 units in that run) counted as motion during the gap. Fixed in
the next commit: drift is measured from where the pose settled at the start of the gap to the farthest
sample after 0.2 s. The scale-probe internal ratios (1.05, 1.22, 0.90 with 95 % CIs containing 1) and the
auxiliary observations, including the exact match between injected joint errors and reported-minus-simulated
joint offsets and the position norm 0.96706 units at the working configuration, are valid and consistent with
the superseding run. Nothing in this directory was edited after the run.
