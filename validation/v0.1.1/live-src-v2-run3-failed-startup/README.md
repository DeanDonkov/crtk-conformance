# Live run 3 against SRC v2.0.0: interface crashed at start-up (no probe data; kept for completeness)

The released `launch_crtk_interface.py` raised `TypeError: unsupported operand type(s) for /: 'NoneType'
and 'float'` in `psm_arm.py:280 -> units_conversion.get_joint_pos` right after start-up
(`logs/launch_crtk_interface.log`): the AMBF client's handle for `psm1/baselink` had not yet received a
state message when the interface first read the joint positions ("Requested Joint Idx of 6 outside valid
range [0 - -1]"). This is a start-up race in the released software under the fixed 25 s / 15 s start-up
delays of `run_live.sh`, not a probe result. The three `report_*.json` files therefore see no `measured_cp`
messages and report `missing topics`. Nothing was modified; the run was repeated as `../live-src-v2/`.
