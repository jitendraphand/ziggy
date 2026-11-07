<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Engineering status: v0.2 (2026-11-07)

* **Scope added:** integrated pelvis plus 2×6-DoF legs with dual-support yoke modules, real bearing supports, pin/slot hard stops, removable modules, cable strain relief and printed covers.
* **Counts:** see `docs/validation/lower_body_v0.2.json` (instances, STEP/STL files) and `hardware/lower_body/bom.csv`.
* **Gates:** 2 PASS / 9 FAIL ([ENGINEERING_GATES.md](ENGINEERING_GATES.md)). **Manufacturing release blocked.**
* **Not changed:** the MuJoCo model and controllers (v0.1). The CAD-vs-sim mismatch is tracked as G-07.
* **Next:** obtain the RobStride 04 drawing; brakes or passive support; add dowels at the actuator output; elastomer stop bumpers; mass reduction; re-proportion the legs to the stature target; sync the sim to the CAD; build and test one knee module on a rig.
