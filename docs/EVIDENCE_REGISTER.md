<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Evidence register (v0.2)

| ID | Evidence | Kind | Generator | Artifact |
|---|---|---|---|---|
| E-01 | Static interference, 12k+ part pairs | CAD geometry | tools/lower_body_qa.py | docs/validation/lower_body_v0.2.json |
| E-02 | Joint sweep and hard-stop engagement, left leg vs whole assembly | CAD geometry | tools/lower_body_qa.py | same |
| E-03 | Printed-part fit, Neptune 4 Max | CAD geometry | tools/lower_body_qa.py | same |
| E-04 | CAD mass rollup (density × volume; purchased = datasheet) | CAD + datasheet | tools/lower_body_qa.py | same; manifest.json |
| E-05 | Hand calcs HC-01…16 | analytic | tools/hand_calcs.py | docs/HAND_CALCS.md, hand_calcs_v0.2.json |
| E-06 | Engineering gates, derived and recomputed in tests | derived | tools/engineering_gates.py | docs/ENGINEERING_GATES.* |
| E-07 | Previews (assembled, exploded, knee, packaging) | render | tools/lower_body_views.py | media/v02_*.png |
| E-08 | Simulation (unchanged v0.1 model) | MuJoCo | tools/sim.py, tools/qa.py | media/metrics.json |
| E-09 | Vendor interface data | datasheet/drawing | manual | docs/REFERENCES.md |
| — | Physical tests | **none** | — | — |

There is no FEA. Hand calcs are first-order with stated assumptions.
