# Live run 9 against SRC v1.0.0 (superseded by `../live-src-v1/`; kept as data)

Taken on probe commit aff6da5 (after the resting-noise fix) with `run_live.sh v1`; complete and internally
consistent (`logs/run.log`, `live_aux.json`, three `report_*.json`, `meta_env.json`). Superseded because the
second mock campaign (`../mock-run2-superseded/`) exposed a further scale-probe defect (a single command followed by
silence during the settle window). The SRC interfaces hold position on silence, so the results are not expected to
differ from the reported run beyond run-to-run timing variation; the reported run was repeated on the fixed commit
so that every reported number comes from one probe version. Nothing in this directory was edited after the run
except this README.
