<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Safety. Read before building or powering anything.

ZIGGY is an **unvalidated research prototype**. A ~46 kg robot with 120 N·m joints can crush, pinch,
strike or fall onto a person. Lithium batteries can catch fire.

## Hard rules
1. **No unattended or household use** until claims `STRUCT`, `WALK` and `SAFE_HOME` in `docs/claims.json` are
   `validated-physical` with evidence. Today they are `untested`.
2. **Commissioning ladder** (do not skip steps): single actuator on a bench → one limb clamped to a fixture →
   full robot **suspended from an overhead gantry/hoist with a fall-arrest harness**, feet off the ground → feet
   loaded with the harness slack-limited → supervised stance. Keep the robot **tethered** to a current-limited
   bench supply (start at ≤5 A) for every step before battery operation.
3. **E-stop**: a hardwired, category-0 removal of actuator power through a safety relay and contactor (see
   docs/WIRING.md), plus a wireless E-stop with a heartbeat (a lost link means stop). The software stop is
   additional, never a replacement.
4. **Exclusion zone**: ≥2 m clear radius during powered tests. One operator holds the E-stop. No children or pets.
5. **Torque/velocity limits** in firmware set to the minimum the test needs. Start position-control gains low.
6. **Battery**: use a pack with a BMS (cell over- and under-voltage, over-current, temperature cutoff). Fuse at the
   pack, charge only supervised in a fire-safe container, store at storage charge, and never puncture or crush it.
   Never charge it inside the robot during development.
7. **Printed parts are not load-rated.** Primary load paths are metal. Do not replace metal parts with prints.
8. Follow your local electrical and machinery safety rules (for example ISO 10218 / ISO 13482 principles for
   personal-care robots). This repository is not a certification and gives no warranty (see the licenses).
