# Cross-version check: tool position reported by SRC v1.0.0 and v2.0.0 at matched insertion

Both releases build `measured_cp` as the forward kinematics of `measured_jp` (v1.0.0 `psm_arm.py` l.203-206,
v2.0.0 l.274-277). The live runs commanded the PSM to q = (0, 0, q3, 0, 0, 0) with q3 = 1.0 interface units
(v1.0.0) and q3 = 0.1 (v2.0.0) — 40 % of each release's insertion range (v1 j3 limit 2.40, v2 qmax 0.24) — and
recorded ‖measured_cp position‖:

| release | commanded q3 | ‖measured_cp‖ (live, `live_aux.json` q_work) | own FK at the commanded q | own FK at the measured `measured_js` |
|---|---|---|---|---|
| v1.0.0 (158b554e) | 1.0 | 0.96641 | 0.967 | 0.9673 |
| v2.0.0 (03befbf1) | 0.1 | 0.08610 | 0.0861 | 0.0861 |

Ratio 0.967 / 0.0861 = 11.23, not 10. The constants explain it exactly:

```
v1.0.0 psmFK.py l.70-75:        L_rcc 4.389   L_tool 4.16   L_pitch2yaw 0.09   L_yaw2ctrlpnt 0.106   (tool2rcm 0.229)
v2.0.0 PSMKinematicSolver():    L_rcc 0.4389  L_tool 0.416  L_pitch2yaw 0.009  L_yaw2ctrlpnt 0.0     (tool2rcm 0.0229)
z(v1) = -(4.389  - 1.0 - 4.16  + 0.09  + 0.106) = -0.967
z(v2) = -(0.4389 - 0.1 - 0.416 + 0.009 + 0.0  ) = -0.0861
```

Every length constant of v1.0.0 is ten times its v2.0.0 counterpart except the control-point offset, which v2.0.0
sets to zero (`psmKinematics.py` l.103, "Fixed length from the tool yaw joint to the end effector tip") where
v1.0.0 uses 0.106 (ten times the dVRK large-needle-driver value 0.0106 m). With the control point matched the
ratio is exactly 10 (0.967 vs 0.0967). The v1.0.0 live value differs from its nominal FK (0.96641 vs 0.967, 0.06 %)
because the gravity-loaded v1.0.0 arm settled at measured_js = (0.0152, -0.0350, 1.0003, -0.0003, 0.0059, 0.0063)
rather than at the commanded configuration and the reported position is the mean of noisy samples (σ ≈ 5 mm units
on x); FK of the mean joint vector is 0.9673 (the reported run is `live-src-v1/`, probe commit e546af1; the
superseded run 8 gave 0.96728 / 0.96729).

Commands (inside `focal-crtk:rc3`, `source /rc3/live/env.sh`):

```
cd sources/src-v1/scripts && python3 -c "from surgical_robotics_challenge.kinematics.psmFK import compute_FK; import numpy as np; T=compute_FK([0,0,1.0,0,0,0,0],7); print(np.linalg.norm(T[0:3,3]))"
cd sources/src-v2/scripts && python3 -c "from surgical_robotics_challenge.kinematics.psmKinematics import PSMKinematicSolver; import numpy as np; k=PSMKinematicSolver(); print(k.L_rcc,k.L_tool,k.L_pitch2yaw,k.L_yaw2ctrlpnt,k.L_tool2rcm_offset); T=k.compute_FK([0,0,0.1,0,0,0,0],7); print(np.linalg.norm(T[0:3,3]))"
```

What this does and does not show: the reported positions are consistent with each release's declared constants
and the constants differ by a factor of ten (the D2 claim, ledger P01/P02/P62/P63, D19). It does not measure the
unit of either interface against an external reference: no independent SI anchor exists inside the simulator's
CRTK interface, so the dimensional class remains `undetermined` in every live report.
