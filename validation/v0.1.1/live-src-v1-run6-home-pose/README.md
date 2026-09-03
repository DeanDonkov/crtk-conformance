# Live run 6 against SRC v1.0.0, probes started at the released scene's initial pose (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance ab465e0 (final 0.1.1 probes). This run is complete (runs A, B, C) and its
probe results are consistent (no stop policy up to 2 s, no state machine, commands executed without one,
internal ratio 0.86–0.93 with the noise-scaled step, rate undetermined at the feedback jitter of this
instance). It was superseded only so that both versions are probed from the same physical working
configuration (insertion at 40 % of the documented range) rather than from the scene's initial pose, whose
insertion of 0.0254 interface units (2.5 mm) puts the tip within millimetres of the RCM and, in v2.0.0,
makes Cartesian control singular. `live_aux.json` here includes the exact match between the injected joint
errors printed by the interface (`logs/launch_crtk_interface.log`) and the reported-minus-simulated joint
offsets (Reviewer #2 M2). Nothing in this directory was edited after the run.
