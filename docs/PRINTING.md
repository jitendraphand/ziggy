<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Printing guide (ELEGOO Neptune 4 Max)

* Envelope: official 420×420×480 mm. ZIGGY parts must fit **400×400×470 mm** (10 mm margin), enforced by `tests/test_cad.py`.
* Each STL is exported **already in print orientation**, centred and on the bed (z_min = 0). Orientation notes are in `hardware/manifest.json`.
* Print one part per plate. Small parts may be batched, but each is a separate file.
* Start profile (tune for your filament): 0.4 mm nozzle, 0.2 mm layers, 4 walls, 25% gyroid.
  * PETG: 240 °C nozzle, 80 °C bed.
  * TPU 95A (sole pad): 25 mm/s or less.
  * Finger parts: 100% infill, printed on their side so the pin bores are vertical and round.
* **Print the coupons first**: `coupon_holes` (Ø2.9–4.2), `coupon_tube_fit` (radial 0.2–0.6 mm on Ø40 tube). Update `PIN` and `CLR` in `cad/parts.py`, regenerate, and rerun `tools/qa.py`.
* Torso shells (440 mm tall, printed standing on edge): use a brim and enclose the printer if the room is drafty.

| Part | Qty | Material | Load class |
|---|---|---|---|
| see `hardware/manifest.json` (generated: bbox, volume, sha256) | | | |
