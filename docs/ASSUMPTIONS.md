<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Provisional design assumptions (v0.1)

| Item | Assumption | Basis / to revisit |
|---|---|---|
| Stature | 1.65–1.70 m (model head top 1.70 m) | adult kitchen work, counter 0.9 m |
| Mass | 46 kg in the model budget (`MASS` in params.py) | must be re-derived from a CAD mass rollup |
| Payload | 2 kg per hand at forearm reach; 5 kg two-handed close to the body | household items |
| Speed | slow, quasi-static manipulation; walking is future work | safety |
| Cost | rough €/$ 25–40k in actuators and electronics; not quoted | no purchases made |
| Structure | 6061-T6 tube Ø40×2 + CNC 6061 plates; steel shoulder bolts at joints | **needs FEA and testing** |
| Printer | Neptune 4 Max 420×420×480 mm; usable envelope 400×400×470 mm (10 mm margin) | official spec |
| Wrist actuators | 3 N·m rated / 6 N·m peak **placeholder**, not a selected part | select and verify |
| Finger servo | 0.5 N·m rated / 1 N·m peak **placeholder** | select and verify |
| Inertias | diagonal approximations in the MJCF | replace with CAD-derived values |
