<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Not yet designed or validated (explicit inventory)

| # | Item | Why it matters |
|---|---|---|
| 1 | CNC actuator clevis/flange plates (≈30), clamp collars | the primary load path; drawings and FEA needed |
| 2 | Pelvis and torso metal frame, battery tray | structural and battery protection |
| 3 | Wrist actuator selection and mechanism | QA flags the placeholder near its limit |
| 4 | Finger servo selection, extensor return, hand sensing | grasping untested |
| 5 | Linkage ankle (v0.2) | reduces distal leg mass |
| 6 | CAD mass and inertia rollup to replace MJCF estimates | sim fidelity |
| 7 | Walking controller (MPC/WBC), stepping recovery, state estimation | **no walking; only a simulated standing ankle strategy (docs/SIMULATION.md)** |
| 8 | Whole-body IK including legs and squatting; reach to a 0.75 m table and the floor | QA shows these targets are unreachable standing |
| 9 | Firmware: CAN drivers, safety MCU heartbeat, category-1 stop | safety |
| 10 | Head sensors (stereo/RGB-D), mic, speaker | household perception |
| 11 | Physical print and fit trials, tube-fit coupons | all print claims are `designed` only |
| 12 | Electrical schematic and PCB (power distribution board) | today it is a block diagram |
| 13 | Household risk assessment (ISO 13482-style) | required before any home trial |
