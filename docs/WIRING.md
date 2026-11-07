<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Electrical architecture v0.1

```
 [12S Li-ion pack + BMS]──[pack fuse]──┐                 ┌──[48V bench PSU, current-limited]  (tethered commissioning)
                                       └──(selector / ideal-diode OR; never both connected uncontrolled)
                                                    │
                               [precharge R + relay]║[MAIN CONTACTOR K1] ◄── coil via SAFETY RELAY
                                                    │                           ▲   ▲
       ┌──────────┬──────────┬──────────┬──────────┼──────────┐                 │   │
   [F1 L-leg] [F2 R-leg] [F3 L-arm] [F4 R-arm] [F5 torso+neck] [F6 DC-DC]   E-stop  wireless
       │          │          │          │          │           │  │         (robot) E-stop (heartbeat)
     6 act.     6 act.     4 act.+3wr  4 act.+3wr waist,neck  12V hands 5V compute
       │          │          │          │          │
     CAN0       CAN1       CAN2       CAN3       CAN4    (1 Mbit/s, 120 Ω at both ends, twisted pair + GND)
       └──────────┴──────────┴──────────┴──────────┴── compute (RT loop 500 Hz) ── IMU, foot sensors
```
* **Stop path**: E-stop or wireless heartbeat loss → safety relay opens → K1 drops → all actuator power is removed (category 0).
  Logic and compute stay powered for logging. Expect the robot to go limp: **the gantry/harness is required**.
  Category-1 controlled stop (damping mode, then K1) is future work and needs validated firmware.
* **Precharge**: the actuator drives carry bulk capacitance, so precharge to ≥90% before K1 closes. This avoids contact welding and inrush.
* **Fusing**: protect each branch at the wire's ampacity. Use XT30 to each actuator (manufacturer connector) and 14 AWG branch / 10 AWG trunk silicone wire (verify with the current calculation).
* **Encoders**: AK80-64 has a single motor-side encoder (64:1), so the output angle is ambiguous at power-up. Add an output absolute encoder on SPI to the limb node MCU, or run a homing routine on a fixture. RobStride 04 has dual encoders.
* **Cable paths**: harnesses run along the tube inside the 6 mm annulus under the limb covers, secured by `cable_clip_tube40`. Leave a service loop of ≥1.5× the joint range across each joint, and keep CAN pairs away from phase leads.
* **Grounding**: single-point star ground at the power distribution board. CAN GND is referenced there.
