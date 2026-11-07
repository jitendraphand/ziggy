<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Lower body v0.2: integrated load-bearing legs and pelvis

Status: **preliminary design; manufacturing release blocked.** 2 of 11 engineering gates pass ([ENGINEERING_GATES.md](ENGINEERING_GATES.md)).
Source: `cad/lower_body.py` (CERN-OHL-S-2.0). Evidence: `tools/lower_body_qa.py` → `docs/validation/lower_body_v0.2.json`.

## Architecture
Each leg has 6 joints, built from one repeated **dual-support yoke module** (local axis +Y):
plate A (actuator stator, 6061-T6, 8 mm) | actuator | output arm A' | 64 mm gap | arm B' | 1 mm | plate B (12 mm, SKF 6004 bearing).
The output side has two supports: the actuator's own output bearing, plus a 6004 on a Ø20 stub in plate B. Leg moments
therefore don't pass only through the actuator bearing (hip yaw is the exception, see blocker B-2).

| Joint | Actuator | Range (deg) | Hard stop | Level (mm above ground) |
|---|---|---|---|---|
| hip yaw | AK80-64 | -30/+30 | **none (G-03 FAIL)** | yaw face 1108 |
| hip roll | AK80-64 | -25/+45 | Ø6 pin in arc slot, +2° | 1022 |
| hip pitch | RobStride 04 | -110/+30 | pin/slot | 870 |
| knee | RobStride 04 | 0/130 | pin/slot | 540 |
| ankle pitch | AK80-64 | ±40 | pin/slot | 244 |
| ankle roll | AK80-64 | ±20 | pin/slot | 92 |

Bridges between modules: 40×3 6061-T6 tube with split clamp (2×M6) **plus** a Ø6 cross pin for positive torque lock,
or a flat 4×M6 / 44 mm square interface. Foot: 10 mm aluminium plate with printed TPU sole.

## Interfaces (sourced vs blocked)
* **AK80-64**: Ø98 body, 61.9 mm length, rear mount 8×M3 on Ø85 PCD, output 6×M4 on Ø28 PCD, Ø21 pilot. Taken from the CubeMars 2D drawing ([REFERENCES](REFERENCES.md)).
  Flags: the phase reference is ambiguous; the 2×Ø3 dowels are **not drilled** because the drawing doesn't locate them.
* **RobStride 04**: **BLOCKER B-1.** No mount drawing was obtained, so the adapter plates are modeled **undrilled** and carry `critical = DRAWING REQUIRED`. No hole pattern has been invented.
* **SKF 6004-2RSH**: 20×42×12, C 9.95 kN, C0 5.0 kN. Housing Ø42 H7, retained by DIN 471 ring (groove not yet modeled).

## Manufacturing
Every part is listed in `hardware/lower_body/manifest.json` (id, make/buy, material, process, critical dimensions, status, mass)
with separate STEP files (`hardware/lower_body/step`) and STL files (`.../stl`). Machined-part STLs are for visualization only and say so.
Right-side parts are mirrors of the left (`mirror_of`). BoM: `hardware/lower_body/bom.csv` (fastener counts are **estimates**).
Process assumptions: CNC 6061-T6 plates ±0.05 on bores and pilots, ±0.1 general; tube cut to length ±0.2, cross-pin hole reamed Ø6 H7.
Printed covers: PETG, 0.2 mm layers, 4 walls; they carry no load and fit the Neptune 4 Max (G-04).

## Assembly and service sequence (per module)
1. Press the 6004 into plate B and fit the circlip. 2. Bolt the actuator to plate A (8×M3, threadlocker). 3. Bolt arm A' to the output (6×M4).
4. Join arm A' and arm B' with the bridge; insert the stop pin. 5. Slide the stub through the bearing, close plate B to plate A with spacers.
6. Route the cable through the rear-cap notch and zip-tie it to the strain-relief bridge. 7. Fit the printed rear cap (3×M3) and the bearing cover.
**Service:** each module comes off as a unit by removing the 2 clamp bolts and cross pin (tube) or the 4×M6 (flat), then unplugging its connector.
No other module has to be taken apart to remove one.

## Cable routing and strain relief
Each actuator's power and CAN cables leave through the top notch of its rear cap and are zip-tied at the bridge. They then run along
the tube with a service loop at each joint sized for full travel. Loop length and bend radius are **not yet verified** (open item).

## Pinch hazards (design assessment, not tested)
| Location | Hazard | Mitigation in CAD | Status |
|---|---|---|---|
| Knee flexion 0→130° | thigh/shin closing gap (scissor) | none beyond covers | **open**: needs a fixed guard or a ≥25 mm min gap (ISO 13854) |
| Yoke arm vs plate (all joints) | 1 mm rotating gap | bearing cover shrouds plate B | partly mitigated |
| Stop pin and arc slot | finger in slot | slot inside plate B, behind cover | mitigated (cover) |
| Ankle / foot vs floor | crush under foot | n/a | inherent; operational controls |
| Between legs (hip roll adduction) | legs collide ≤-10° | none | **open**: G-02 FAIL |

## Power-off collapse
HC-16: AK80-64 backdrive torque is about 4.7 N·m, against the roughly 50 N·m needed to hold a squat at the knee or hip (SF 0.09). **When de-energized, the robot collapses.**
Required before any trial: a gantry/harness, plus fail-safe brakes or a passive support. None is designed yet (NOT_YET_DESIGNED).
