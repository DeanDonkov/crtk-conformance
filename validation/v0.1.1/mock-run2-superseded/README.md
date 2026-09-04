# Second v0.1.1 mock campaign (kept as data; superseded by `../mock/`)

Probe code: crtk-conformance aff6da5. Complete campaign, 2713 s, taken after the resting-noise fix. Its liveness
results are correct (the 0.1 s release policy is detected: `L_release_000` τ̂_w = 0.210 s). It exposed a second
defect, in `PRESET_emul-ambf-object-watchdog_authored.json`: the anchored unit estimate was ŝ = 1.91 (95 % CI
1.10–2.73) and the dimensional class `divergent` for an emulated unit of exactly 1 — a false divergent. Cause: the
scale probe sent one `servo_cp` command per step and then stayed silent through its 0.6 s settle window; the
emulated 0.5 s release policy fired inside the window and the 20 mm/s drift was measured as displacement. Fix
(commit 6e93bd1): the scale probe streams its goal at the client rate during settling and measurement and holds
the start pose under a stream before each step, as a servo client would; a regression test
(`test_scale_probe_streams_goals_so_a_release_policy_does_not_corrupt_the_estimate`) was added. The campaign
and both live runs were repeated on the fixed commit. Nothing in this directory was edited after the run except
the regenerated `tables/` (analysis output) and this README.
