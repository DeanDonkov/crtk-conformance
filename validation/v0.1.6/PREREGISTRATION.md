# RC9 pre-registration: v0.1.6 campaigns

**Written:** 21 September 2026, before any `crtk-conformance` probe was run against the dVRK kinematic simulation or against the SRC releases with the geometry anchor.

**Earlier contact with the targets:**

- The only earlier contact with dVRK-sim was the feasibility smoke test of the RC9 brief. It ran no probe. Its outputs are exploratory and are archived separately, in the RC9 prompt's zip.
- The WP1 smoke test below (`dvrk_sim_smoke.py`) also runs no probe.
- Disclosure: both smoke scripts compute, outside the tool, two quantities that the probes also estimate. They are the relation between `measured_cp` and `local/measured_cp`, and the common normal of the wrist-pitch and wrist-yaw screw axes. In WP1 (run before this file was committed; outputs in `dvrk_sim/smoke/`) these were:
  - JHU: measured = base_frame · local to 1.6e-16 m / 5e-11 rad.
  - Without base frame: identity.
  - The common normal was 9.1000 mm on both configurations (λ̂ = 1.0000). No zero stamps were seen after homing.
  - No stamp of `measured_cp` equalled one of `local/measured_cp` exactly; 200 of 200 had a partner within 50 ms.
  - The predictions below for the frame and geometry probes on dVRK-sim therefore rest partly on that exploratory observation and are not blind. What they test is whether the tool's decision reproduces a known truth on third-party software.

**Code state:** the tool, harness, configurations and declarations are fixed at commit `ffd9ade` (branch `rc9`). This file is committed on top of that commit, and its commit hash is recorded in the campaign metadata.

**If something changes after a result is seen:** any change to the tool, a parameter, a declaration or the case list after an independent-target result has been seen is reported with the reason and both outcomes. Outcomes that differ from a prediction are reported as observed.

## Fixed parameters

| Item | Value |
|---|---|
| Task | ε = 1 mm, except where stated; r_ws = 0.10 m; v = 50 mm/s; client 100 Hz; J_max = 5 ms |
| Frame probe | 10 trials, 5 samples per trial, 50-ms pairing window. Unset (zero) stamps are skipped (0.1.6). |
| Geometry anchor | 5 trials; settle 1.0 s; window 0.3 s; Δq = 0.5 rad (wrist pitch, then wrist yaw) |
| Geometry anchor: L | L = 9.1 mm: sawIntuitiveResearchKit 2.4.0 (`bc33e79`) `share/tool/LARGE_NEEDLE_DRIVER_400006.json` l.29, `A = 0.0091` |
| Geometry anchor: u_rel | 1.5 %. Released models disagree by 1.1 %: SRC uses 9.0 mm. |
| Geometry anchor: gates | Angle gate 0.02 rad; slide gate 2 % of d_int; axes 90° ± 2° |
| Temporal probe | 5 trials, gap_max 2.0 s. Liveness rule 0.1.6 (confirmation), 30 calibration commands, α = 0.01, at most 4 repeats. |
| `--enable-timeout-s` | 15 on dVRK-sim; homing takes several seconds there, measured in WP1 |
| dVRK-sim reference joints | yaw 0.1, pitch 0.1, insertion 0.12 m, roll 0, wrist_pitch 0.1, wrist_yaw −0.2 |
| Launches | 3 independent launches per configuration: fresh ROS master and `dvrk_system`, bring-up enable/home, stamp and pose check |

**Probe order per launch:**

1. Client runs: C_dvrk, C_src, C_disc
2. Frame cases: F2–F5
3. Unit cases: U
4. T_fault025

## 1. dVRK 2.4.0 kinematic simulation (released software, unmodified)

**Truth sources:**

- **Frame.** The PSM2 `base_frame` of `dvrk_config_jhu` 2.4.0 `jhu-dVRK/system-MTMR-PSM2-Teleop.json` l.22–28, copied by script. The code behind it is `mtsIntuitiveResearchKitArm.cpp` l.961 (measured_cp = base_frame · local) and l.1587 (goals mapped through base_frame⁻¹).
- **Unit.** SI, per REP-103 and the dVRK documentation; the tool file's A is in metres.
- **State machine.** Present: `operating_state` and `state_command`. Goals are ignored unless the arm is ready, per `ArmIsReady` in the arm source.
- **Stop behaviour.** The last goal is held. No command-age timeout was identified in the arm layer, and kinematic simulation has no IO watchdog.

Declared E_max values are derived in `derived_truths.json`.

| Config | Case | Declaration | Spatial | Dimensional | Temporal (sub-verdicts → class) |
|---|---|---|---|---|---|
| jhu | C_dvrk | RC8 dVRK-authored file; dimensional = geometry anchor, SI, ε = 1 mm | **C** (E_max 0) | **U** (the interval reaches 1.5 mm > ε) | state `required` satisfied; stop `hold` 2 s satisfied; rate U → **U** |
| jhu | C_src | RC8 SRC-authored file (identity; state `forbidden`); geometry SI | **D** (278.1 mm) | **U** | state `forbidden` **violated** (goals ignored while DISABLED); stop satisfied; rate U → **D** |
| jhu | C_disc | none | U | U (scale probe, no anchor) | U |
| jhu | F2 identity | identity | **D** (278.1 mm) | – | – |
| jhu | F3 inverse | base_frame⁻¹ | **D** (412.3 mm) | – | – |
| jhu | F4 | residual 0.7 mm along a 0.3° axis | **C** (0.874 mm; loose bound 1.224 mm) | – | – |
| jhu | F5a/b/c/d | residual E_max 0.95/0.98/1.02/1.05 mm | **C/C/D/D** | – | – |
| jhu | U_si_5mm | geometry anchor, SI, ε = 5 mm | – | **C** | – |
| jhu | U_mm_1mm / U_mm_5mm | geometry anchor, client unit 1 mm | – | **D / D** | – |
| jhu | T_fault025 | stop `fault` within 0.25 s | – | – | stop **violated** (held) → **D** |
| nobase | C_src | as above | **C** | **U** | **D** (state `forbidden` violated) |
| nobase | C_dvrk | as above | **D** (278.1 mm) | **U** | **U** |
| nobase | F_exactJHU | JHU transform | **D** (278.1 mm) | – | – |

**Notes on these predictions:**

- The C_disc scale probe also reports the 0.1.6 command/feedback consistency diagnostic. The prediction is **no flag**: the binding is shared and the kinematic simulation tracks exactly.
- The simulation is noise-free, so F4/F5 test the decision rule on third-party data, not its statistical reliability. There is no "exactly at ε" case (see the RC9 brief, §4.4).

## 2. Released SRC v1.0.0 / v2.0.0 on AMBF `ambf-2.0` @ `16a81518` (WP3.5)

**Setup:** RC3/RC8 chain, launch arguments as RC8, PSM1 only, no source modified.

**Parameters:** as the RC8 live runs (`--settle-s 2.0 --still-tol-mm 0.05 --trials 10 --temporal-trials 5`), plus geometry anchor settle 2.0 s. AMBF is a dynamic simulation, so this settle time is longer than on dVRK-sim.

**Per release, in one stack launch:**

- SRC-authored + geometry, ε = 1 mm, all probes
- SRC-authored + geometry, ε = 5 mm, geometry only
- dVRK-authored + geometry, ε = 1 mm, all probes

If a geometry gate fails (for example, joint tracking under AMBF's controller does not reach Δq within 0.02 rad), the outcome is U and is reported as such.

**Unit truth:**

- v1.0.0 uses 0.1-m units: `CHANGELOG.md` l.29 "assets were scaled by 10"; `psmFK.py` l.70–72.
- v2.0.0 is SI.

**Geometry anchor:**

- Joints `4` / `5` (`toolrolllink-toolpitchlink`, `toolpitchlink-toolyawlink`).
- Reference joints [0, 0, q_ins, 0, 0.1, −0.2], with q_ins = 1.0 (v1, interface units) or 0.1 (v2), as in RC8.
- Same L, u_rel, Δq and gates as above.

| Release | Declaration | Spatial | Dimensional @ 1 mm | Dimensional @ 5 mm | Temporal |
|---|---|---|---|---|---|
| v1.0.0 | SRC-authored + geometry SI | U (no second pose) | **D** (λ̂ ≈ 0.101 m/unit) | **D** | as RC8: **U** |
| v1.0.0 | dVRK-authored + geometry SI | U | **D** | – | **D** (state machine absent) |
| v2.0.0 | SRC-authored + geometry SI | U | **U** (λ̂ ≈ 1.011; interval reaches 2.6 mm) | **C** | **U** |
| v2.0.0 | dVRK-authored + geometry SI | U | **U** | – | **D** |

## 3. Reference node (crtk-mock), 0.1.6 campaigns

### WP4: K1, correct feedback with a wrong command binding

- Feedback binding: the JHU transform, declared correctly.
- Command binding: JHU · E, with E = 20 mm and 10° about the axis (0.48, 0.60, 0.64).
- Scale probe with the SI-pose anchor.

**Predictions:**

- Frame **C**: false relative to command semantics, because [A] is violated.
- Scale internal ratio ≠ 1.
- Anchored scale **D**: the non-shared frame binding is misattributed to units.
- Consistency diagnostic: **flag**.

**Controls:** presets `reference` and `emul-dvrk-jhu-psm2` give **no flag**. Three runs each.

### WP5: decision reliability near the boundary

**Offline Monte Carlo** (`boundary_montecarlo_v016.py`; the released `spatial_decision` / `decide` / `scale_decision` unchanged):

- Grid: E_true/ε ∈ {0.90, 0.95, 0.98, 1.00, 1.02, 1.05, 1.10}.
- Truth is labelled exactly (fractions): E ≤ ε counts as conformant.
- Frame trial model: pure translation; σ_trial = σ·√(2/5) with σ ∈ {0.02, 0.1} mm; or the RC8 mixture per trial.
- Scale trial model: SI-pose anchor, pre-window error σ/√30 per axis in interface units, σ ∈ {0.02, 0.1} mm.
- n = 10 (frame) or 9 (scale); N = 2000 replicates per cell; seed 31000.

**Stated expectations:**

- Exactly at ε, only false D or U is possible.
- A calibrated two-sided 95 % interval gives about 2.5 % false D there.
- The Gaussian construction is conservative, so less is expected.
- The mixture may exceed it.

**Live confirmation** (`run_validation_v016.py`):

- Frame at ratios {0.95, 1.00, 1.05} × {Gaussian 0.02 mm, Gaussian 0.1 mm, mixture}, N = 30 each.
- Scale at ratios {0.95, 1.00, 1.05} × Gaussian 0.1 mm, N = 30.
- The rates must agree with the Monte Carlo within the Clopper–Pearson intervals. Agreement or disagreement is reported.

### WP6: liveness under command loss

**Policies:**

- (a) no stop policy: hold; truth "held"
- (b) fault policy with operating state, τ_w = 0.25 s

**Loss conditions:**

- none
- iid 2 % and 5 %
- Gilbert–Elliott, mean burst 5 commands (p_bg = 0.2), stationary loss 5 % (p_gb = 0.0105)

**Rules:** 0.1.5 (12 calibration commands, archived rule) and 0.1.6 (30, confirmation), run separately.

**Sample size:** N = 8 per cell, stated in the report.

**Per-run classification:**

- (a) held_through_range (correct) / false trip (a stop policy claimed) / not observable
- (b) interval formed and contains 0.25 s / formed and excludes it (a false verdict) / U (contradictory, unattributable, not formed)

**Predictions:**

- Under 0.1.5, lost post-gap commands produce false trips in (a) and excluding or inconsistent intervals in (b) at 2–5 % loss.
- Under 0.1.6, false trips and excluding intervals are rare (per gap at most α under iid loss), paid for with more trials.

**Archived re-derivation (offline):** the 23 archived intervals under the 0.1.6 rule. Archived non-responses carry no confirmation, so rejection-only evidence becomes unattributable. Every change is reported.
