<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Requirements traceability (v0.1)

| ID | Requirement | Design element | Verification | Status |
|---|---|---|---|---|
| R1 | Full-size humanoid (~1.6–1.75 m) | `params.DIM` | `test_model::test_mass_and_stature_budget` | simulated ✔ |
| R2 | Every printed part has separate STEP + STL | `cad/build.py`, manifest | `test_cad::test_step_reimports_valid`, `test_stl_watertight` | ✔ |
| R3 | Each part fits Neptune 4 Max with margin | `print_envelope_mm()` | `test_cad::test_fits_neptune4max_with_clearance` | ✔ (geometry) |
| R4 | Easy assembly | split shells, common M3/M5, clip-on covers | ASSEMBLY.md, physical trial | concept |
| R5 | Electronics components and wiring | BOM.md, WIRING.md | design review | architecture |
| R6 | Dexterous hands | 5-finger tendon hand, 6 act./hand | MJCF hand, CAD parts | sim + CAD; grasp untested |
| R7 | Legs with full 6-DoF | joint table | `test_model` | simulated |
| R8 | Body balance | support margin, PD stance | `test_balance` | quasi-static only |
| R9 | IK tool included and tested | MuJoCo + DLS IK | `test_ik` | ✔ |
| R10 | Recursive QA/QC | `tools/qa.py`, CI | CI run, validation report | ✔ |
| R11 | Household tasks | reach to counter/shelf/door/switch | `test_household_reach_standing` | partial (table and floor gap) |
| R12 | Actuator torque adequacy | ratings in params, `static_torque_cases` | `test_torque` | estimate; squat knee >rated |
| R13 | No unvalidated hardware claims | `docs/claims.json` | `test_release_gates` | ✔ |
| R14 | Safety near people, battery | SAFETY.md, E-stop chain | review + physical E-stop test | untested |
