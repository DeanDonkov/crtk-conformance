# Live run 1 against SRC v2.0.0, probes started at the released scene's initial pose (kept as data; superseded by `live-src-v2/`)

Probe code: crtk-conformance ab465e0 (final 0.1.1 probes). The scene starts with the instrument at joint zero
(insertion 0.0043 m in `measured_js`), i.e. with the tool tip at the remote centre of motion, where Cartesian
inverse kinematics is singular. The Cartesian probes therefore saw erratic responses (internal ratios 0.24–3.4,
tracking ratios of opposite sign in some directions, a 4.8 mm wander during command gaps classified as
`release`) that reflect the singular configuration, not the interface semantics. The topic inventory,
frame_id strings, publish rate, state-topic absence and de-perturbed joint feedback in `live_aux.json` are
unaffected and agree with the superseding run. The reported run places the arm at 40 % of the documented
insertion range (servo_jp, in the version's own unit) before probing. Run stopped after run A. Nothing in
this directory was edited after the run.
