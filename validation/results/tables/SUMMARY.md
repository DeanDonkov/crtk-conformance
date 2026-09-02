# Validation results — crtk-conformance 0.1.0 @ a5cb73e9c7c1, 2026-09-02T14:24:32Z, wall 20.1 min, Linux-6.18.44-fc-v22-x86_64-with-glibc2.39

## F — frame probe

| noise (mm) | runs | mean ‖t̂−t‖ (mm) | max | mean |θ̂−θ| (deg) | max | mean ‖t̂‖ at t=0 (mm) | FP@1mm | FN@1mm | undet.@1mm | n |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.00 | 40 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1 | 0 | 0 | 10 |
| 0.02 | 40 | 0.0067 | 0.0140 | 0.0000 | 0.0000 | 0.0086 | 0 | 0 | 1 | 10 |
| 0.10 | 40 | 0.0357 | 0.0665 | 0.0000 | 0.0000 | 0.0397 | 0 | 0 | 1 | 10 |

## S — scale probe

| s | noise (mm) | r_int (mean ± std) | ŝ (mean ± std) | |ŝ−s|/s | outcome@1mm | latency (ms) |
|---|---|---|---|---|---|---|
| 1.0 | 0.00 | 1.0000 ± 0.0000 | 1.0000 ± 0.0000 | 4.44e-16 | conformant | 8.6 |
| 1.001 | 0.00 | 1.0000 ± 0.0000 | 1.0010 ± 0.0000 | 4.44e-16 | conformant | 9.1 |
| 1.01 | 0.00 | 1.0000 ± 0.0000 | 1.0100 ± 0.0000 | 2.20e-16 | undetermined | 9.4 |
| 1.05 | 0.00 | 1.0000 ± 0.0000 | 1.0500 ± 0.0000 | 1.06e-15 | divergent | 8.7 |
| 1.1 | 0.00 | 1.0000 ± 0.0000 | 1.1000 ± 0.0000 | 4.04e-16 | divergent | 8.4 |
| 0.9 | 0.00 | 1.0000 ± 0.0000 | 0.9000 ± 0.0000 | 3.70e-16 | divergent | 9.7 |
| 0.5 | 0.00 | 1.0000 ± 0.0000 | 0.5000 ± 0.0000 | 4.44e-16 | divergent | 8.3 |
| 0.1 | 0.00 | 1.0000 ± 0.0000 | 0.1000 ± 0.0000 | 2.78e-15 | divergent | 8.6 |
| 10.0 | 0.00 | 1.0000 ± 0.0000 | 10.0000 ± 0.0000 | 0.00e+00 | divergent | 8.7 |
| 1.0 | 0.02 | 1.0008 ± 0.0039 | 1.0010 ± 0.0036 | 1.03e-03 | conformant | 4.7 |
| 1.001 | 0.02 | 0.9974 ± 0.0054 | 1.0017 ± 0.0034 | 6.84e-04 | conformant | 4.8 |
| 1.01 | 0.02 | 1.0010 ± 0.0057 | 1.0089 ± 0.0041 | 1.06e-03 | undetermined | 5.5 |
| 1.05 | 0.02 | 1.0020 ± 0.0024 | 1.0514 ± 0.0051 | 1.31e-03 | divergent | 5.9 |
| 1.1 | 0.02 | 1.0009 ± 0.0049 | 1.0985 ± 0.0036 | 1.37e-03 | divergent | 7.3 |
| 0.9 | 0.02 | 1.0004 ± 0.0069 | 0.8982 ± 0.0035 | 1.96e-03 | divergent | 4.2 |
| 0.5 | 0.02 | 1.0008 ± 0.0081 | 0.4997 ± 0.0040 | 5.38e-04 | divergent | 6.0 |
| 0.1 | 0.02 | 0.9750 ± 0.0327 | 0.1003 ± 0.0050 | 2.92e-03 | divergent | 7.0 |
| 10.0 | 0.02 | 0.9999 ± 0.0005 | 9.9997 ± 0.0036 | 2.56e-05 | divergent | 8.7 |
| 1.0 | 0.10 | 0.9975 ± 0.0211 | 1.0044 ± 0.0226 | 4.44e-03 | divergent | 5.8 |
| 1.001 | 0.10 | 0.9949 ± 0.0176 | 0.9891 ± 0.0133 | 1.19e-02 | undetermined | 4.8 |
| 1.01 | 0.10 | 1.0148 ± 0.0174 | 1.0201 ± 0.0239 | 1.00e-02 | divergent | 5.0 |
| 1.05 | 0.10 | 0.9973 ± 0.0287 | 1.0540 ± 0.0232 | 3.79e-03 | divergent | 5.4 |
| 1.1 | 0.10 | 1.0010 ± 0.0233 | 1.1116 ± 0.0183 | 1.05e-02 | divergent | 5.1 |
| 0.9 | 0.10 | 1.0019 ± 0.0135 | 0.8897 ± 0.0125 | 1.14e-02 | divergent | 5.6 |
| 0.5 | 0.10 | 0.9949 ± 0.0526 | 0.5025 ± 0.0210 | 5.06e-03 | divergent | 5.1 |
| 0.1 | 0.10 | 1.1231 ± 0.1972 | 0.1001 ± 0.0244 | 1.10e-03 | divergent | 4.4 |
| 10.0 | 0.10 | 1.0003 ± 0.0026 | 10.0119 ± 0.0150 | 1.19e-03 | divergent | 4.9 |

Internal ratio over all 27 runs (s from 0.1 to 10): mean 1.0039, min 0.9750, max 1.1231 — feedback invisibility.
No-anchor control (s = 0.1): outcome **undetermined**, r_int = 1.0000.

## T — temporal probe: liveness

| τ_w injected (s) | τ̂_w (s) | std | 95 % CI | n | finding |
|---|---|---|---|---|---|
| 0.0 | — | — | — | 5 | no liveness policy detected up to 1.5 s (n=5) |
| 0.05 | 0.050 | 0.0001 | 0.050..0.050 | 5 | liveness policy detected |
| 0.1 | 0.099 | 0.0017 | 0.097..0.101 | 5 | liveness policy detected |
| 0.25 | 0.249 | 0.0001 | 0.249..0.249 | 5 | liveness policy detected |
| 0.5 | 0.499 | 0.0026 | 0.495..0.502 | 5 | liveness policy detected |
| 1.0 | 1.000 | 0.0000 | 1.000..1.000 | 5 | liveness policy detected |

Released-drift emulation (τ_w = 0.5 s, release, 20 mm/s drift): no liveness policy detected up to 1.5 s (n=5); pose drifted during the gap (released semantics); drift during 1.5 s gap = 19.8 mm (n=5).

## T — effective command rate

| loop (Hz) | publish (Hz) | measured publish (Hz) | effective rate at client 50/100/200/500/1000 Hz |
|---|---|---|---|
| 120 | 120 | 120.0 | 48 / 100 / 121 / 120 / 121 |
| 120 | 1000 | 884.8 | 50 / 100 / 120 / 112 / 121 |
| 1500 | 100 | 100.0 | 48 / 95 / 100 / 100 / 99 |
| 1500 | 1000 | 943.9 | 50 / 100 / 191 / 493 / 701 |

## T — state precondition on presets

| preset | operating_state topic | executed when DISABLED | executed after enable+home | enable latency (ms) | executed without state machine |
|---|---|---|---|---|---|
| reference | True | False | True | 6.551346000378544 | — |
| emul-dvrk-jhu-psm2 | True | False | True | 17.193554999721528 | — |
| emul-src-v1 | False | — | — | — | True |

## R — robustness

| case | spatial | dimensional | temporal | false 'conformant'? |
|---|---|---|---|---|
| delay_50ms_jitter_20ms | conformant | conformant | divergent | no |
| divergent_but_noisy | divergent | divergent | conformant | no |
| drop_10 | conformant | undetermined | conformant | no |
| drop_50 | conformant | undetermined | divergent | no |
| missing_measured_cp | undetermined | undetermined | undetermined | no |
| no_local_topic | undetermined | conformant | conformant | no |

## P — presets (full run)

| preset | spatial | dimensional | temporal | key estimates |
|---|---|---|---|---|
| emul-ambf-object-watchdog | undetermined | conformant | conformant | ŝ=1.000; eff@100Hz=98 Hz |
| emul-dvrk-jhu-psm2 | divergent | conformant | conformant | ‖t̂‖=200.0 mm, θ̂=150.0°; ŝ=1.000; eff@100Hz=99 Hz |
| emul-src-v1 | undetermined | divergent | conformant | ŝ=0.100; eff@100Hz=100 Hz |
| emul-src-v2 | undetermined | conformant | conformant | ŝ=1.000; eff@100Hz=100 Hz |
| reference | conformant | conformant | conformant | ‖t̂‖=0.0 mm, θ̂=0.0°; ŝ=1.000; eff@100Hz=93 Hz |

## M — model check (mock execution vs. error model)

| case | ‖p_c‖ or step (mm) | executed error (mm) | model prediction (mm) | bound (mm) |
|---|---|---|---|---|
| absolute_perp | 0.0 | 200.00 | 200.00 | 200.00 |
| absolute_axis | 0.0 | 200.00 | 200.00 | 200.00 |
| absolute_perp | 20.0 | 203.70 | 203.70 | 238.64 |
| absolute_axis | 20.0 | 200.00 | 200.00 | 238.64 |
| absolute_perp | 50.0 | 222.10 | 222.10 | 296.59 |
| absolute_axis | 50.0 | 200.00 | 200.00 | 296.59 |
| absolute_perp | 100.0 | 278.07 | 278.07 | 393.19 |
| absolute_axis | 100.0 | 200.00 | 200.00 | 393.19 |
| incremental_perp | 1.0 | 1.93 | 1.93 | 1.93 |
| incremental_axis | 1.0 | 0.00 | 0.00 | 1.93 |
| incremental_perp | 5.0 | 9.66 | 9.66 | 9.66 |
| incremental_axis | 5.0 | 0.00 | 0.00 | 9.66 |
| incremental_perp | 20.0 | 38.64 | 38.64 | 38.64 |
| incremental_axis | 20.0 | 0.00 | 0.00 | 38.64 |
| scale_absolute | 0.0 | 0.00 | 0.00 | nan |
| scale_absolute | 20.0 | 18.00 | 18.00 | nan |
| scale_absolute | 50.0 | 45.00 | 45.00 | nan |
| scale_absolute | 100.0 | 90.00 | 90.00 | nan |
