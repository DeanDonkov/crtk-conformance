# mock-run1-superseded — first v0.1.2 campaign (commit f9e2b51), superseded

Complete campaign (54.2 min, 2026-09-05T02:12Z) at commit f9e2b51, the first freeze of 0.1.2. Its analysis
(`validation/analyze_v012.py`) found one estimator gap in the rate sub-probe, visible in window T_rate_004 at
100 Hz: the accepted-command channel showed 96 of 100 commands with a 54 ms stale interval on a mock that accepts
every command, and the verdict was *violated*, although the archive cannot tell whether the four commands were
lost in the mock's single-slot subscriber queue or whether the probe's own send loop stalled for 54 ms — the
probe recorded only its mean send rate. The estimator was changed to record the client's longest send gap and to
return *undetermined* when that gap exceeds the required period (a client stall is not the implementation's
stale interval); the reason text of a channel that never reports a commanded target was also corrected (it said
"not piecewise constant"), and the liveness finding string now names interface units instead of mm. The code was
re-frozen (see the reported archive's `meta.json`) and the whole campaign rerun; this directory is kept unchanged
as the record of the first run and is used for no claim in the manuscript. All other results of this run
(spatial, coverage, scale, liveness, presets, model check) agree with the reported run.
