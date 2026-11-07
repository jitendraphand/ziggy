<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Not yet designed or validated (explicit inventory)

| # | Item | Why it matters |
|---|---|---|
| 1 | Lower-body plates/arms/bridges now in CAD (v0.2, docs/LOWER_BODY.md). Still open: RobStride 04 adapter holes (no drawing), AK80 output dowels, DIN 471 grooves, drawings with GD&T, hip yaw stop and tilting-moment support | primary load path |
| 2 | Torso metal frame, battery tray (pelvis plate exists in v0.2) | structural and battery protection |
| 3 | Wrist actuator selection and mechanism | QA flags the placeholder near its limit |
| 4 | Finger servo selection, extensor return, hand sensing | grasping untested |
| 5 | Linkage ankle (v0.2) | reduces distal leg mass |
| 6 | Sync MJCF to v0.2 CAD (lower body 31.25 kg vs 22 kg sim; joint heights differ up to 238 mm) | sim fidelity (G-06, G-07) |
| 7 | Walking controller (MPC/WBC), stepping recovery, state estimation | **no walking; only a simulated standing ankle strategy (docs/SIMULATION.md)** |
| 8 | Whole-body IK including legs and squatting; reach to a 0.75 m table and the floor | QA shows these targets are unreachable standing |
| 9 | Firmware: CAN drivers, safety MCU heartbeat, category-1 stop | safety |
| 10 | Head sensors (stereo/RGB-D), mic, speaker | household perception |
| 11 | Physical print and fit trials, tube-fit coupons | all print claims are `designed` only |
| 12 | Electrical schematic and PCB (power distribution board) | today it is a block diagram |
| 13 | Household risk assessment (ISO 13482-style) | required before any home trial |
| 14 | Fail-safe brakes or passive support (robot collapses when de-energized, HC-16) | safety |
| 15 | Elastomer stop bumpers and software deceleration (HC-14) | stop impact |
| 16 | Leg-to-leg clearance in hip roll adduction (G-02) and knee pinch guard | collision, pinch |
| 17 | Arms, torso and hands at v0.2 depth | only the lower body was upgraded |
