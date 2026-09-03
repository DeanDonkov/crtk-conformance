# Preliminary live run 4 against SRC v1.0.0 (kept as data; superseded by `live-src-v1/`)

Probe code: crtk-conformance 40b3d45. The stop-behaviour classification is now consistent (`no stop policy
detected up to 2.0 s`, every post-gap command responded, pose held), but the report shows
`latency_probes_responded = 0`: the response-latency measurement inside `measure_resolution` never published
the goal it waited for (a bug present since the first 0.1.1 commit, harmless on the mock only because the
timeout fallback of 0.4 s sufficed there), and its reference pose was the stale start pose rather than the
current one. Fixed in the next commit. Run stopped after run A. Nothing in this directory was edited after
the run.
