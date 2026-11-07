<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Bill of materials v0.1 (preliminary; nothing purchased)

Ratings are copied from the manufacturer pages listed in [REFERENCES.md](REFERENCES.md) (checked 2026-11-07).
**Verify current datasheets before buying.** Prices change, so none are listed.

## Actuators
| Qty | Joint(s) | Primary choice | Rated / peak N·m | Mass | Alternative | Notes |
|---|---|---|---|---|---|---|
| 17 | hip yaw, roll, pitch; ankle pitch, roll; waist; shoulder pitch, roll; elbow | CubeMars AK80-64 KV80 | 48 / 120 | 0.85 kg | RobStride 04 (40/120) | single **motor-side** 14-bit encoder, so add an output-side absolute encoder |
| 2 | knee | RobStride 04 | 40 / 120 | 1.42 kg | AK80-64 | dual encoders; 15–60 VDC |
| 4 | shoulder yaw, neck yaw, neck pitch | CubeMars AK70-10 | 8.3 / 24.8 | 0.62 kg | smaller AK/RobStride class | |
| 6 | wrist yaw, pitch, roll | **TBD** (placeholder 3/6) | assumption | – | small QDD or servo + belt | QA flags wrist pitch at about 1.0× the placeholder rating with a 2 kg reach |
| 12 | finger flex ×5 + thumb opposition, per hand | **TBD** smart serial bus servo (placeholder 0.5/1) | assumption | – | Dynamixel XL/XC class, Feetech STS class | drives `tendon_spool` |

Simulation static-torque findings (`docs/validation/v0.1.0.md`):
* Standing: every joint is below 30% of its rated torque.
* Deep squat (knee at 90°): knee at about 1.26× its continuous rating. That is allowed only briefly (below the 120 N·m peak), so limit the duty cycle or add a reduction stage.
* 2 kg at forward reach: elbow was over the AK70-10 rating, so v0.1 upgraded it to the AK80-64 (finding F-003).

## Structure (metal, load-bearing; not printed)
| Qty | Item | Spec |
|---|---|---|
| 4 | thigh/shin tubes | 6061-T6 Ø40×2 mm, ~330–340 mm long |
| 4 | upper arm/forearm tubes | 6061-T6 Ø40×2 mm (Ø30 option) |
| ~30 | actuator flange plates/clevises | CNC 6061-T6, 6–8 mm (**drawings not yet designed**) |
| 1 | pelvis and torso frame | 20×20 aluminium extrusion + 6061 plates |
| – | fasteners | ISO 4762 M3/M4/M5 class 12.9 into metal; threadlocker; M3 brass heat-set inserts in prints only for covers |
| 30 | finger joint pins | Ø3×(16–20) mm hardened dowel pins, ISO 8734 |
| – | tendons | 0.8 mm UHMWPE braided line, PTFE tube liners |

## Electronics
| Qty | Item | Example / requirement |
|---|---|---|
| 1 | battery | 12S Li-ion (44.4 V nominal, 50.4 V max), ≥20 Ah, **with BMS** (OV/UV/OC/OT); keep below the actuators' max voltage |
| 1 | bench supply (commissioning) | 48 V DC, adjustable current limit, ≥30 A |
| 1 | main fuse | DC-rated class-T or ANL-type, sized to the harness (start at 60 A) |
| 1 | main contactor + precharge | DC contactor ≥100 A coil 12 V; precharge resistor and relay for actuator bus capacitance |
| 1 | safety relay | dual-channel E-stop safety relay (category 0 stop) |
| 2 | E-stops | 22 mm latching mushroom (on robot, `estop_mount`) + wireless E-stop with heartbeat |
| 5 | branch fuses | one per limb plus torso and neck, DC-rated blade fuses sized to the actuator count |
| 2 | DC-DC | 48→12 V (hand servos), 48→5 V (compute) isolated, ≥5 A |
| 5 | CAN interfaces | USB-CAN or native CAN-FD, 1 Mbit/s, one bus per limb plus torso and neck |
| 20 | output encoders | 14-bit magnetic absolute (AS5047P/AS5048 class) on AK80-64 outputs |
| 1 | IMU | 6/9-axis, ≥400 Hz, at the pelvis |
| 1 | compute | Jetson Orin / x86 NUC class (perception) + real-time MCU (safety heartbeat) |
| 2 | foot load | four load cells per foot (future; the MuJoCo model uses touch sensors) |
