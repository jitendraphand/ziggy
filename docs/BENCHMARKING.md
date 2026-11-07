<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Benchmarking plan (measurement plan, not a comparison)

ZIGGY v0.2 has **not** been compared with any robot, because there is no hardware to measure. The table lists public reference figures
for a commercial full-size humanoid, so future measurements have a defined yardstick. They are reference values, not targets ZIGGY meets.

| Metric | Reference: Unitree H1 (public spec) | ZIGGY v0.2 design value | How it will be measured |
|---|---|---|---|
| Height | ~1800 mm | 1870 mm (CAD) | tape, standing |
| Mass | ~47 kg | lower body 31.25 kg known; full body not rolled up | scale |
| Thigh / shank length | 400 / 400 mm | 330 / 296 mm between joint axes (CAD) | CAD + caliper |
| Knee peak torque | ~360 N·m | 120 N·m (RobStride 04 datasheet peak) | dyno / static load cell |
| Hip peak torque | ~220 N·m | 120 N·m | as above |
| Walking speed | public videos/specs | no walking | treadmill / motion capture |
| Push recovery | not published comparably | sim only | calibrated impulse pendulum |

Sources: unitree.com/h1 and support.unitree.com (checked 2026-11-07). Figures are vendor-reported, not independently verified.
Protocol, to run after G-10: identical payload, floor, impulse device, and 10 trials per metric with mean ± s.d.
