# validation/v0.1.5 — verification of the 0.1.5 liveness revision (RC8)

The measurement set reported in the manuscript is still `validation/v0.1.3` (unchanged). This directory holds what the
0.1.5 release adds: the offline re-derivation of the archived liveness intervals under the 0.1.5 rules, and a verification
campaign of the revised probe on fresh reference-node measurements and on the two released SRC interfaces. None of the runs
here replaces a reported number; they verify the revision (plan: `VALIDATION_PLAN_V015.md`).

## Contents

| Path | What | Produced by |
|---|---|---|
| `reanalysis/L_liveness_v015.{csv,md}` | the 34 archived liveness runs under the 0.1.5 rules (23/23 formed intervals still contain the injected timeout; median width 28.5 → 34.6 ms; 12 lower ends move; `R_drop_50` unattributable) | `reanalyze_liveness_v015.py` (archive read only) |
| `mock/` | the v0.1.3 campaign design re-run by the 0.1.5 probe (seeds +30000) plus experiment V; 291 run records, 96 event logs, `meta.json` (sha256 of every file), `run_validation_v015.log`, `tables/` and figures from `analyze_v013.py` | `run_validation_v015.py` inside `focal-crtk:rc3` |
| `COMPARISON.md` | run-by-run comparison with the v0.1.3 archive under the current rules, the liveness table, experiment V | `compare_v015.py` |
| `live-src-v1/`, `live-src-v2/` | SRC v1.0.0 (158b554e) and v2.0.0 (03befbf1) on AMBF `ambf-2.0` @16a81518, three expectation files each, `meta_env.json`, logs | `environment/run_live.sh` (unchanged) |
| `tests/` | `unit.log` (82 passed), `integration.log` (35 passed), `versions.txt`, the driver log; `superseded-run1/`, `superseded-run2/` (earlier test runs, see below) | `environment`-style container run |
| `mock-run1-superseded/` | the first campaign run, kept because experiment V exposed two pre-existing observability defects of the stop sub-probe (CHANGELOG 0.1.5); its non-V results are superseded by `mock/` | first run of `run_validation_v015.py` |
| `live-src-v1-run2-superseded/` | the SRC v1.0.0 run of the second attempt; exposed the latency-window restriction (a 2.05-s "response" read after a 2.0-s silence, CHANGELOG 0.1.5); superseded by `live-src-v1/` | second attempt |
| `live-src-v2-run2-failed-startup/`, `live-src-v2-run3a-failed-startup/` | SRC v2.0.0 attempts in which the CRTK interface script crashed at start-up because the AMBF objects were not ready after the script's 25-s wait (known start-up race of the SRC/AMBF chain; no measurement); `live-src-v2/` is the successful re-run of the same unchanged script | attempts of `run_live.sh v2` |

## Environment and provenance

`focal-crtk:rc3` was rebuilt from scratch from `validation/environment/` (debootstrap focal from archive.ubuntu.com → toolchain
→ ROS Noetic from the pinned sources → cp38 wheels → crtk_msgs @14b04fcf, AMBF ambf-2.0 @16a81518, SRC v1.0.0/v2.0.0); image
digest `sha256:a11b7b278af2b3b8feb3315f384c389844d86a2ae60efe184026b096235cb644` (in every `meta.json` / `meta_env.json`).
Python 3.8.10, numpy 1.24.4, scipy 1.10.1; two vCPUs; tests, campaign and live runs strictly sequential. The code under test is
`ad2232d` (`origin/main`) plus the 0.1.5 patch, i.e. the working tree of the 0.1.5 release before it was committed
(`repo_dirty: true` in `meta.json`); the source files of the release are byte-identical to that tree.

Timeline of the three campaign attempts (2026-09-19, UTC): run 1 (07:24–08:47, tests + campaign; the live runs were not
started) exposed the drift-evaluation and start-in-FAULT/pose-at-goal defects in experiment V; run 2 (08:52–10:21, tests +
campaign + live v1; v2 failed at start-up) exposed the latency-window restriction on live SRC v1.0.0; run 3 (10:30–11:59,
tests + campaign + live v1; live v2 at 12:04–12:08 after one further start-up failure) is the archive in `mock/`,
`live-src-v1/`, `live-src-v2/`, `tests/`. The integration-test log of run 2 shows the one failure of
`test_rate_reduced_targets_keep_the_requested_rate` that led to the test's noise level being moved off the five-target
boundary (CHANGELOG 0.1.5; the rate diagnostic is unchanged).

## Results in brief (details in `COMPARISON.md` and `mock/tables/SUMMARY.md`)

* Of the 276 decided runs common to the v0.1.3 archive and this campaign, 273 have the same outcome as the archive under the
  current rules (rate withdrawn since 0.1.4, `R_drop_50` unattributable since 0.1.5); the three that differ
  (`F_rev_par_2`, `S_si_020`, `R_noisy`) are boundary cases whose interval end lies near the 1-mm tolerance and whose verdict
  turns on the noise draw.
* Frame sweep 120/120 intervals contain the exact maximum error (mean/max translation error 0.036/0.075 mm at 0.1-mm noise;
  0.039/0.087 mm in the archive); coverage replication 60/60 in all four configurations; scale sweep C/D/U = 6/18/3 (the archive's
  false divergent at the closed boundary is undetermined with these seeds); liveness 24 formed intervals, 24/24 contain the
  injected timeout, median width 33.3 ms, departure of the last streamed command observed in 1021 of 1068 gap trials (never
  with 20-Hz feedback, where L is used and the interval still contains); source-age diagnostic 117/155 windows contain the
  event-log truth (98/150 in the archive; a diagnostic, never a verdict).
* Experiment V: delayed application of 50/100/300 ms gives r_i of 51–62/101–112/301–312 ms measured from observed departures
  and intervals containing the 500-ms timeout ([438.2, 572.7], [388.2, 610.9], [187.8, 810.8] ms); delayed drift at 100 ms
  gives [351.3, 501.0] ms; delayed drift at 300 ms is undetermined (the 0.93-s settle window exceeds the timeout, so no passing
  trial can be evaluated and the onset is censored); 50 % random loss with a hold expectation is undetermined
  (confounded), 20 % loss happened to lose no post-gap command and the hold was satisfied; a 0.25-s fault policy under 30 %
  loss is detected from the operating state, [232.0, 393.3] ms; the same policy without a state machine is undetermined
  (the documented cost of the attribution rule).
* Live SRC v1.0.0 and v2.0.0: U/U/U, U/U/U, U/U/D under the three expectation files, as in v0.1.3; every calibration command
  answered, every gap trial responded, 120-Hz feedback; the 0.1.5 rules change nothing there.
