# Preliminary live run 1 against SRC v1.0.0 (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance 806ccd9 (first 0.1.1 commit). This run is retained because it exposed two
weaknesses of the 0.1.1 probes on a real, noisy, gravity-loaded controller that the mock could not show:

1. The liveness classifier treated "the post-gap command was not attained within 0.25 x step" as
   `rejected`, although the arm *responded* (it moved toward the goal but settled ~0.1 interface units
   away because of the joint controllers' gravity sag on this release, cf. SRC v2.0.0 CHANGELOG "Set the
   model level gravity to zero in order to improve the control accuracy"). A tracking error is not a
   rejection.  Fixed in the next commit: a command is `rejected` only when no motion toward the goal is
   observed; otherwise it is `attained` or `not_attained`.
2. The scale probe used a fixed 0.005 interface-unit step and a stillness criterion, both of which are
   inside the resting jitter of this instance (sigma_hat ~ 4.5e-3 interface units), so the internal ratios
   scattered between 0.4 and 3.1.  Fixed in the next commit: the step is scaled to the resting noise and the
   displacement is measured as the difference of averaged windows before and after a fixed settle time.

Nothing in this directory was edited after the run.  The auxiliary observations (`live_aux.json`) are
valid and agree with the superseding run.
