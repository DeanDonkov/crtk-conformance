# Preliminary live run 2 against SRC v2.0.0 (kept as data; superseded by `live-src-v2/`)

Probe code: crtk-conformance d8ef24cb. First v2.0.0 run from the working configuration (insertion 0.1 m).
The three probe reports are valid. The auxiliary script, however, subscribed to
`/ambf/env/psm1/baselink/State` for the simulator's own joint state, a topic that exists in the v1.0.0 scene
but not in the v2.0.0 scene (whose base body publishes as `/ambf/env/psm1/baselinksimple/State`, see
`topics.txt`), so `live_aux.json` lacks the `reported_minus_simulated_joint` comparison. `run_live.sh` was
given the per-version body name and the run repeated. Nothing in this directory was edited after the run.
