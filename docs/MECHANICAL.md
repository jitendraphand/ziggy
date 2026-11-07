<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Whole-body mechanical architecture v0.1

**29 body DoF**, plus 2 × (15 finger joints driven by 6 actuators):
* Leg ×2 (6): hip yaw → hip roll → hip pitch → knee → ankle pitch → ankle roll
* Waist yaw (1), neck yaw and pitch (2)
* Arm ×2 (7): shoulder pitch → roll → yaw → elbow → wrist yaw → pitch → roll
* Hand ×2: 5 fingers × 3 joints (MCP, PIP, DIP). In the sim the tendon coupling is PIP = MCP and DIP = 0.8·PIP, with 1 flexor per finger plus thumb opposition.

## Load path. Metal carries the load; prints do not.
`actuator output flange → CNC 6061 clevis plate (M5 12.9) → Ø40×2 6061 tube (clamped + cross-bolted) → next clevis → next actuator housing`.
Printed parts are classified in `hardware/manifest.json`:
* `cover`: shells over the tubes, actuators, torso, pelvis and head. They protect people from pinch points and keep cables tidy. **No structural load.**
* `guide`: cable clips and the E-stop mount.
* `functional`: hand phalanges, palm, tendon spools, TPU sole pad. Light-duty prototypes: test them before you trust them.
* `coupon`: tolerance calibration prints.

## Joint fastening concept
* Actuator to plate: the manufacturer's bolt pattern, class 12.9 into metal threads, medium threadlocker, torque-striped.
* Plate to tube: split-clamp collar plus one cross-bolt (prevents rotation). Verify that tube bearing stress is acceptable (**FEA pending**).
* Covers: two half-shells joined at the flanges with M3 screws into heat-set inserts. They clip around the tube with 0.4 mm nominal clearance (tune with `coupon_tube_fit`).
* Finger joints: Ø3 dowel pins in Ø3.1 printed bores (tune with `coupon_holes`), retained by the clevis walls.

## Hand
Tendon-driven, underactuated. Servos in the forearm drive `tendon_spool`s. Each flexor runs through the 1.6 mm channels (PTFE-lined), and the extensor return uses elastic or a second channel. The design aims at the household grasps envelope (cylindrical, pinch, hook). **Grasp force has not been tested.**

## Known mechanical weaknesses
* The ankle actuators are distal and heavy. A linkage-driven ankle (actuators at the knee) is planned for v0.2.
* The diagonal inertias are rough.
* The torso and pelvis frame, and all CNC plate drawings, are **not yet designed** (see NOT_YET_DESIGNED.md).
