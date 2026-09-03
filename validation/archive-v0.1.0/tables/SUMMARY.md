# Validation results — crtk-conformance 0.1.0 @ 5393272c3e17, 2026-09-02T19:05:40Z, wall 21.6 min, Linux-6.18.44-fc-v22-x86_64-with-glibc2.39

## F — frame probe

| noise (mm) | runs | mean ‖t̂−t‖ (mm) | max | mean |θ̂−θ| (deg) | max | mean ‖t̂‖ at t=0 (mm) | FP@1mm | FN@1mm | undet.@1mm | n |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.00 | 40 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1 | 0 | 0 | 10 |
| 0.02 | 40 | 0.0068 | 0.0145 | 0.0000 | 0.0000 | 0.0070 | 0 | 0 | 1 | 10 |
| 0.10 | 40 | 0.0326 | 0.0682 | 0.0000 | 0.0000 | 0.0368 | 0 | 0 | 1 | 10 |

## S — scale probe

| s | noise (mm) | r_int (mean ± std) | ŝ (mean ± std) | |ŝ−s|/s | outcome@1mm | latency (ms) |
|---|---|---|---|---|---|---|
| 1.0 | 0.00 | 1.0000 ± 0.0000 | 1.0000 ± 0.0000 | 4.44e-16 | conformant | 8.7 |
| 1.001 | 0.00 | 1.0000 ± 0.0000 | 1.0010 ± 0.0000 | 4.44e-16 | conformant | 8.6 |
| 1.01 | 0.00 | 1.0000 ± 0.0000 | 1.0100 ± 0.0000 | 2.20e-16 | undetermined | 8.6 |
| 1.05 | 0.00 | 1.0000 ± 0.0000 | 1.0500 ± 0.0000 | 1.06e-15 | divergent | 8.3 |
| 1.1 | 0.00 | 1.0000 ± 0.0000 | 1.1000 ± 0.0000 | 4.04e-16 | divergent | 8.8 |
| 0.9 | 0.00 | 1.0000 ± 0.0000 | 0.9000 ± 0.0000 | 3.70e-16 | divergent | 8.2 |
| 0.5 | 0.00 | 1.0000 ± 0.0000 | 0.5000 ± 0.0000 | 4.44e-16 | divergent | 8.6 |
| 0.1 | 0.00 | 1.0000 ± 0.0000 | 0.1000 ± 0.0000 | 2.78e-15 | divergent | 8.7 |
| 10.0 | 0.00 | 1.0000 ± 0.0000 | 10.0000 ± 0.0000 | 0.00e+00 | divergent | 8.2 |
| 1.0 | 0.02 | 1.0017 ± 0.0076 | 0.9998 ± 0.0037 | 1.69e-04 | conformant | 6.2 |
| 1.001 | 0.02 | 0.9984 ± 0.0048 | 1.0014 ± 0.0035 | 4.37e-04 | conformant | 5.0 |
| 1.01 | 0.02 | 1.0015 ± 0.0042 | 1.0099 ± 0.0040 | 6.37e-05 | undetermined | 6.6 |
| 1.05 | 0.02 | 1.0009 ± 0.0044 | 1.0516 ± 0.0046 | 1.49e-03 | divergent | 5.0 |
| 1.1 | 0.02 | 0.9981 ± 0.0043 | 1.1014 ± 0.0029 | 1.24e-03 | divergent | 5.4 |
| 0.9 | 0.02 | 0.9987 ± 0.0068 | 0.8980 ± 0.0042 | 2.26e-03 | divergent | 6.0 |
| 0.5 | 0.02 | 1.0006 ± 0.0075 | 0.5019 ± 0.0031 | 3.70e-03 | divergent | 5.8 |
| 0.1 | 0.02 | 1.0128 ± 0.0473 | 0.1008 ± 0.0034 | 8.18e-03 | divergent | 5.7 |
| 10.0 | 0.02 | 0.9999 ± 0.0005 | 10.0026 ± 0.0034 | 2.56e-04 | divergent | 8.7 |
| 1.0 | 0.10 | 0.9964 ± 0.0262 | 1.0036 ± 0.0200 | 3.57e-03 | undetermined | 5.4 |
| 1.001 | 0.10 | 1.0013 ± 0.0227 | 0.9948 ± 0.0123 | 6.21e-03 | undetermined | 6.1 |
| 1.01 | 0.10 | 1.0135 ± 0.0322 | 1.0120 ± 0.0175 | 1.96e-03 | undetermined | 5.2 |
| 1.05 | 0.10 | 0.9969 ± 0.0205 | 1.0491 ± 0.0183 | 8.12e-04 | divergent | 6.3 |
| 1.1 | 0.10 | 0.9978 ± 0.0187 | 1.0990 ± 0.0162 | 9.37e-04 | divergent | 5.1 |
| 0.9 | 0.10 | 1.0030 ± 0.0271 | 0.9038 ± 0.0167 | 4.27e-03 | divergent | 5.2 |
| 0.5 | 0.10 | 1.0209 ± 0.0317 | 0.5088 ± 0.0196 | 1.76e-02 | divergent | 6.7 |
| 0.1 | 0.10 | 1.0886 ± 0.1864 | 0.1025 ± 0.0183 | 2.50e-02 | divergent | 5.6 |
| 10.0 | 0.10 | 0.9999 ± 0.0020 | 9.9939 ± 0.0263 | 6.12e-04 | divergent | 6.0 |

Internal ratio over all 27 runs (s from 0.1 to 10): mean 1.0048, min 0.9964, max 1.0886 — feedback invisibility.
No-anchor control (s = 0.1): outcome **undetermined**, r_int = 1.0000.

## T — temporal probe: liveness

| τ_w injected (s) | τ̂_w (s) | std | 95 % CI | n | finding |
|---|---|---|---|---|---|
| 0.0 | — | — | — | 5 | no liveness policy detected up to 1.5 s (n=5) |
| 0.05 | 0.050 | 0.0000 | 0.050..0.050 | 5 | liveness policy detected |
| 0.1 | 0.101 | 0.0058 | 0.094..0.108 | 5 | liveness policy detected |
| 0.25 | 0.249 | 0.0001 | 0.249..0.249 | 5 | liveness policy detected |
| 0.5 | 0.502 | 0.0025 | 0.499..0.505 | 5 | liveness policy detected |
| 1.0 | 1.000 | 0.0000 | 1.000..1.000 | 5 | liveness policy detected |

Released-drift emulation (τ_w = 0.5 s, release, 20 mm/s drift): no liveness policy detected up to 1.5 s (n=5); pose drifted during the gap (released semantics); drift during 1.5 s gap = 20.0 mm (n=5).

## T — effective command rate

| loop (Hz) | publish (Hz) | measured publish (Hz) | effective rate at client 50/100/200/500/1000 Hz |
|---|---|---|---|
| 120 | 120 | 120.0 | 49 / 100 / 61 / 116 / 112 |
| 120 | 1000 | 970.9 | 50 / 96 / 120 / 120 / 120 |
| 1500 | 100 | 95.0 | 49 / 99 / 100 / 100 / 97 |
| 1500 | 1000 | 996.7 | 50 / 100 / 200 / 499 / 795 |

## T — state precondition on presets

| preset | operating_state topic | executed when DISABLED | executed after enable+home | enable latency (ms) | executed without state machine |
|---|---|---|---|---|---|
| reference | True | False | True | 7.767136999973445 | — |
| emul-dvrk-jhu-psm2 | True | False | True | 6.050939000033395 | — |
| emul-src-v1 | False | — | — | — | True |

## R — robustness

| case | spatial | dimensional | temporal | false 'conformant'? |
|---|---|---|---|---|
| delay_50ms_jitter_20ms | conformant | conformant | divergent | no |
| divergent_but_noisy | divergent | undetermined | conformant | no |
| drop_10 | conformant | undetermined | conformant | no |
| drop_50 | conformant | undetermined | divergent | no |
| missing_measured_cp | undetermined | undetermined | undetermined | no |
| no_local_topic | undetermined | conformant | conformant | no |

## P — presets (full run)

| preset | spatial | dimensional | temporal | key estimates |
|---|---|---|---|---|
| emul-ambf-object-watchdog | undetermined | conformant | conformant | ŝ=1.000; eff@100Hz=92 Hz |
| emul-dvrk-jhu-psm2 | divergent | conformant | conformant | ‖t̂‖=200.0 mm, θ̂=150.0°; ŝ=1.000; eff@100Hz=99 Hz |
| emul-src-v1 | undetermined | divergent | conformant | ŝ=0.100; eff@100Hz=100 Hz |
| emul-src-v2 | undetermined | conformant | conformant | ŝ=1.000; eff@100Hz=99 Hz |
| reference | conformant | conformant | conformant | ‖t̂‖=0.0 mm, θ̂=0.0°; ŝ=1.000; eff@100Hz=99 Hz |

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
