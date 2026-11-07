<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Requirements / verification matrix (v0.2)

Targets and achieved values are kept separate. "Achieved" means a CAD/analysis/simulation result **only**. Nothing has been physically measured.

| Req | Requirement (target) | Method | Achieved (v0.2) | Verdict | Evidence |
|---|---|---|---|---|---|
| R-01 | Stature 1650-1700 mm (provisional) | CAD packaging | 1870 mm | FAIL | media/v02_packaging.json |
| R-02 | Lower-body mass ≤ 22 kg (sim budget) | CAD mass rollup | 31.25 kg known, fasteners/wiring excluded | FAIL | lower_body_v0.2.json · mass |
| R-03 | No static interference | CAD boolean, all pairs >1 mm³ | 0 | PASS (analysis) | G-01 |
| R-04 | Full joint travel collision-free incl. other leg | CAD sweep, 10° steps + limits | hip roll adduction ≤-10° hits the right leg | FAIL | G-02 |
| R-05 | Hard stop on every joint | CAD sweep at limit±4° | 5/6 (hip yaw has none) | FAIL | G-03 |
| R-06 | Printed parts fit Neptune 4 Max (420×420×480) | bounding box | 9/9 fit | PASS (analysis) | G-04 |
| R-07 | Structural SF meets required on all hand calcs | hand calcs | 11/16 pass | FAIL | HAND_CALCS.md |
| R-08 | CAD joint heights match sim within 10 mm | comparison | up to 238 mm off | FAIL | G-07 |
| R-09 | All vendor interfaces sourced | drawings | RobStride 04 missing | FAIL | G-09 |
| R-10 | Safe when de-energized (no collapse) | hand calc HC-16 | SF 0.09 | FAIL | HC-16 |
| R-11 | Physical load, fit and stop tests | test rig | not done | FAIL (not run) | G-10 |
| R-12 | Simulated standing and push recovery (v0.1 model) | MuJoCo | as v0.1 (SIMULATION.md) | simulated only | media/metrics.json |
