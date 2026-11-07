<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Hand calculations v0.2 (lower body)

**These are hand calculations, not FEA and not certification.** They use simple beam, shear and bearing formulas,
assumed factors and assumed material values (marked ASSUMPTION). Physical tests are required (see VERIFICATION.md).

## Inputs
| Symbol | Value | Unit | Source |
|---|---|---|---|
| m_robot | 46.0 | kg | sim v0.1 budget (CAD rollup of lower body alone is higher - see lower_body_v0.2.json) |
| k_dyn | 2.5 | - | ASSUMPTION: walking/step dynamic factor on vertical load |
| k_imp | 5.0 | - | ASSUMPTION: drop/impact factor (falls onto one leg) - not validated |
| T_peak | 120.0 | N*m | AK80-64 & RS04 peak torque (manufacturer) |
| C0_6004 | 5000.0 | N | SKF 6004 static load rating (skf.com) |
| C0_ak80 | 2520.0 | N | AK80-64 basic static load rating (cubemars.com) |
| Sy_6061 | 240.0 | MPa | 6061-T6 minimum yield, typical spec value (ASSUMPTION: verify mill cert) |
| Sbr_6061 | 386.0 | MPa | 6061-T6 bearing yield, handbook-typical (ASSUMPTION) |
| Sy_shaft | 650.0 | MPa | 42CrMo4 QT yield, conservative assumed (verify cert) |
| tau_pin | 600.0 | MPa | hardened dowel shear strength ASSUMPTION |
| Fp_M3 | 3000.0 | N | M3 12.9 assembly preload ASSUMPTION (~60% of typical VDI 2230 table values) |
| Fp_M4 | 5000.0 | N | M4 12.9 assembly preload ASSUMPTION |
| Fp_M6 | 12000.0 | N | M6 12.9 assembly preload ASSUMPTION |
| mu | 0.15 | - | aluminium/steel dry friction ASSUMPTION |
| As_M4 | 8.78 | mm^2 | ISO 898 tensile stress area M4 |
| Rm_129 | 1220.0 | MPa | ISO 898-1 class 12.9 min tensile |

## Checks
| ID | Check | Demand | Capacity | Unit | SF | SF req | Status | Note |
|---|---|---|---|---|---|---|---|---|
| HC-01 | Knee 6004 static (impact, one leg) | 1334.16 | 5000.0 | N | 3.75 | 1.0 | **PASS** | lever rule yA=68 yB=47 mm; s0=C0/P0 |
| HC-02 | AK80-64 output bearing static, same split (hip/ankle modules) | 922.14 | 2520.0 | N | 2.73 | 1.0 | **PASS** | moment (tilting) capacity of actuator bearing NOT published -> blocker for hip yaw |
| HC-03 | Ø20 stub shaft bending at arm face (impact) | 11.89 | 650.0 | MPa | 54.66 | 2.0 | **PASS** | a=7.0 mm |
| HC-04 | AK80 output 6xM4 friction torque vs peak | 120.0 | 63.0 | N*m | 0.53 | 1.5 | **FAIL** | friction-only joint SLIPS at peak -> needs dowels (location not on drawing) or shear-loaded fit bolts |
| HC-05 | AK80 output M4 bolt shear if slipped (per bolt) | 1428.57 | 6426.96 | N | 4.5 | 2.0 | **PASS** | bearing-type joint after slip; fatigue not assessed |
| HC-06 | AK80 stator 8xM3 friction torque vs peak | 120.0 | 153.0 | N*m | 1.27 | 1.5 | **FAIL** | low margin: add dowel or higher preload |
| HC-07 | 40x3 tube bending (actuator-limited moment) | 70.63 | 240.0 | MPa | 3.4 | 2.0 | **PASS** | Z=3003 mm^3; ignores cross-pin hole (Kt~2.5 -> see HC-08) |
| HC-08 | Tube bending with cross-pin hole Kt=2.5 (fatigue-ish) | 176.58 | 240.0 | MPa | 1.36 | 1.0 | **PASS** | local yielding risk; move pin to low-stress zone or bond+clamp |
| HC-09 | Split clamp torsion (2xM6) vs 120 N*m | 120.0 | 113.1 | N*m | 0.94 | 1.0 | **FAIL** | friction only; cross pin provides positive lock (HC-10) |
| HC-10 | Ø6 cross pin double shear vs yaw peak | 3000.0 | 16964.6 | N | 5.65 | 2.0 | **PASS** |  |
| HC-11 | Tube wall bearing at Ø6 pin hole | 166.67 | 386.0 | MPa | 2.32 | 1.5 | **PASS** |  |
| HC-12 | Stop pin shear at peak torque | 3636.36 | 16964.6 | N | 4.67 | 2.0 | **PASS** |  |
| HC-13 | Plate B bearing stress at stop slot | 50.51 | 386.0 | MPa | 7.64 | 2.0 | **PASS** |  |
| HC-14 | Hard-stop impact energy at no-load speed (no bumper) | 4.63 | None | J | None | None | **FAIL** | I=0.15 kg m^2 ASSUMED, 75 rpm; metal-on-metal stop needs elastomer bumper + software decel: NOT designed |
| HC-15 | Foot plate 10 mm bending, toe load (dyn) | 36.92 | 240.0 | MPa | 6.5 | 2.0 | **PASS** | ignores lightening cut-outs (Kt) -> verify |
| HC-16 | Power-off: AK80 backdrive torque 4.7 N*m vs squat knee/hip demand ~50 N*m | 50.0 | 4.7 | N*m | 0.09 | 1.0 | **FAIL** | robot collapses when de-energised: brakes or passive support required; gantry mandatory (SAFETY.md) |

Summary: 11 PASS, 5 FAIL. FAIL items are open design actions in docs/DESIGN_REVIEW.md.
