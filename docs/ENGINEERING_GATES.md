<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Engineering gates v0.2 (derived from evidence; regenerate with tools/engineering_gates.py)

| ID | Gate | Status | Evidence | Detail |
|---|---|---|---|---|
| G-01 | Static interference (all pairs, >1 mm^3) = 0 | **PASS** | docs/validation/lower_body_v0.2.json | "12561 pairs" |
| G-02 | Joint travel sweep (left leg, 10 deg steps + limits) collision-free | **FAIL** | joint_sweep_left | {"hip_yaw": 0, "hip_roll": 197, "hip_pitch": 0, "knee": 0, "ankle_pitch": 0, "ankle_roll": 0} |
| G-03 | Mechanical hard stop on every leg joint (engages at limit+4 deg, free inside range) | **FAIL** | joint_sweep_left.hard_stop | {"hip_yaw": false, "hip_roll": true, "hip_pitch": true, "knee": true, "ankle_pitch": true, "ankle_roll": true} |
| G-04 | Printed parts fit Neptune 4 Max envelope | **PASS** | printed_fit | 9 |
| G-05 | Hand calcs: all checks meet required SF | **FAIL** | docs/HAND_CALCS.md | ["HC-04", "HC-06", "HC-09", "HC-14", "HC-16"] |
| G-06 | Lower-body CAD mass <= sim v0.1 lower-body budget | **FAIL** | mass | "31.26 kg vs 22.0 kg (fasteners/wiring excluded)" |
| G-07 | CAD joint heights match sim within 10 mm | **FAIL** | cad_vs_sim | {"ankle_roll": 22.0, "ankle_pitch": 174.0, "knee": 70.0, "hip_pitch": -0.0, "hip_roll": 152.0, "yaw_face": 238.0} |
| G-08 | Stature within provisional target 1650-1700 mm | **FAIL** | media/v02_packaging.json | 1870.0 |
| G-09 | No open interface blockers (vendor drawings) | **FAIL** | hardware/lower_body/manifest.json | ["BLOCKER: RobStride 04 flange/mount drawing not obtained; adapter holes NOT drilled", "BLOCKER: hip yaw relies on actuator output bearing for leg moments; AK80-64 moment rating no |
| G-10 | Physical verification (print/fit/load/stop/E-stop tests) recorded | **FAIL** | none | "no hardware built" |
| G-11 | Self-collision / leg-to-leg sweep for all joints | **FAIL** | joint_sweep_left | "opposite leg only checked for hip yaw/roll" |

2 PASS / 9 FAIL. Manufacturing release requires all PASS; it is **blocked**.
