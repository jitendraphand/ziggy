# SPDX-License-Identifier: Apache-2.0
"""Hand calculations for the v0.2 lower-body joints. NOT FEA, NOT certification.

Every input is listed with its unit and source (datasheet / standard-typical / ASSUMPTION).
Outputs: docs/HAND_CALCS.md and docs/validation/hand_calcs_v0.2.json.
Gate: a check passes only if SF >= required; failing checks stay FAIL until redesign.
"""
import json, math, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)
from cad import lower_body as LB

g = 9.81
IN = {  # value, unit, source
    "m_robot": (46.0, "kg", "sim v0.1 budget (CAD rollup of lower body alone is higher - see lower_body_v0.2.json)"),
    "k_dyn": (2.5, "-", "ASSUMPTION: walking/step dynamic factor on vertical load"),
    "k_imp": (5.0, "-", "ASSUMPTION: drop/impact factor (falls onto one leg) - not validated"),
    "T_peak": (120.0, "N*m", "AK80-64 & RS04 peak torque (manufacturer)"),
    "C0_6004": (5000.0, "N", "SKF 6004 static load rating (skf.com)"),
    "C0_ak80": (2520.0, "N", "AK80-64 basic static load rating (cubemars.com)"),
    "Sy_6061": (240.0, "MPa", "6061-T6 minimum yield, typical spec value (ASSUMPTION: verify mill cert)"),
    "Sbr_6061": (386.0, "MPa", "6061-T6 bearing yield, handbook-typical (ASSUMPTION)"),
    "Sy_shaft": (650.0, "MPa", "42CrMo4 QT yield, conservative assumed (verify cert)"),
    "tau_pin": (600.0, "MPa", "hardened dowel shear strength ASSUMPTION"),
    "Fp_M3": (3000.0, "N", "M3 12.9 assembly preload ASSUMPTION (~60% of typical VDI 2230 table values)"),
    "Fp_M4": (5000.0, "N", "M4 12.9 assembly preload ASSUMPTION"),
    "Fp_M6": (12000.0, "N", "M6 12.9 assembly preload ASSUMPTION"),
    "mu": (0.15, "-", "aluminium/steel dry friction ASSUMPTION"),
    "As_M4": (8.78, "mm^2", "ISO 898 tensile stress area M4"),
    "Rm_129": (1220.0, "MPa", "ISO 898-1 class 12.9 min tensile"),
}
v = {k: x[0] for k, x in IN.items()}
rows = []
def chk(cid, desc, demand, capacity, unit, sf_req, note=""):
    sf = capacity / demand if demand else float("inf")
    rows.append({"id": cid, "check": desc, "demand": round(demand, 2), "capacity": round(capacity, 2), "unit": unit,
                 "SF": round(sf, 2), "SF_required": sf_req, "status": "PASS" if sf >= sf_req else "FAIL", "note": note})

W = v["m_robot"] * g
# 1 knee 6004 bearing static: vertical load through knee, split by lever rule between actuator bearing (A) and 6004 (B)
yA = LB.G / 2 + LB.T_ARM + 28.0       # actuator-internal bearing ~ mid RS04 (ASSUMPTION: location unknown)
yB = LB.G / 2 + LB.T_ARM + LB.AX_GAP + LB.T_PB / 2
F_leg_imp = W * v["k_imp"]
F_B = F_leg_imp * yA / (yA + yB); F_A = F_leg_imp - F_B
chk("HC-01", "Knee 6004 static (impact, one leg)", F_B, v["C0_6004"], "N", 1.0, f"lever rule yA={yA:.0f} yB={yB:.0f} mm; s0=C0/P0")
chk("HC-02", "AK80-64 output bearing static, same split (hip/ankle modules)", F_A, v["C0_ak80"], "N", 1.0,
    "moment (tilting) capacity of actuator bearing NOT published -> blocker for hip yaw")
# 3 shaft bending: cantilever from arm B' face to bearing centre
a = LB.AX_GAP + LB.T_PB / 2
M = F_B * a; d = 20.0
sig = 32 * M / (math.pi * d ** 3)
chk("HC-03", "Ø20 stub shaft bending at arm face (impact)", sig, v["Sy_shaft"], "MPa", 2.0, f"a={a:.1f} mm")
# 4 actuator output joint: 6xM4 on Ø28 PCD, friction only
r_out = 14.0 / 1000
T_fric = 6 * v["Fp_M4"] * v["mu"] * r_out
chk("HC-04", "AK80 output 6xM4 friction torque vs peak", v["T_peak"], T_fric, "N*m", 1.5,
    "friction-only joint SLIPS at peak -> needs dowels (location not on drawing) or shear-loaded fit bolts")
F_sh = v["T_peak"] / r_out / 6
tau_cap = 0.6 * v["Rm_129"] * v["As_M4"]
chk("HC-05", "AK80 output M4 bolt shear if slipped (per bolt)", F_sh, tau_cap, "N", 2.0, "bearing-type joint after slip; fatigue not assessed")
# 6 stator 8xM3 on Ø85
T_st = 8 * v["Fp_M3"] * v["mu"] * 0.0425
chk("HC-06", "AK80 stator 8xM3 friction torque vs peak", v["T_peak"], T_st, "N*m", 1.5, "low margin: add dowel or higher preload")
# 7 tube bending: moment limited by adjacent actuators (vector sum of two peaks, +25% overshoot)
M_t = math.hypot(v["T_peak"], v["T_peak"]) * 1.25 * 1000
Do, Di = LB.TUBE_OD, LB.TUBE_OD - 2 * LB.TUBE_T
Z = math.pi * (Do ** 4 - Di ** 4) / (32 * Do)
chk("HC-07", "40x3 tube bending (actuator-limited moment)", M_t / Z, v["Sy_6061"], "MPa", 2.0, f"Z={Z:.0f} mm^3; ignores cross-pin hole (Kt~2.5 -> see HC-08)")
chk("HC-08", "Tube bending with cross-pin hole Kt=2.5 (fatigue-ish)", 2.5 * M_t / Z, v["Sy_6061"], "MPa", 1.0, "local yielding risk; move pin to low-stress zone or bond+clamp")
# 9 clamp torsion vs hip-yaw peak (2xM6 split clamp) + cross pin
T_cl = v["mu"] * 2 * v["Fp_M6"] * (math.pi / 2) * (Do / 2) / 1000
chk("HC-09", "Split clamp torsion (2xM6) vs 120 N*m", v["T_peak"], T_cl, "N*m", 1.0, "friction only; cross pin provides positive lock (HC-10)")
F_pin = v["T_peak"] / (Do / 2 / 1000) / 2
chk("HC-10", "Ø6 cross pin double shear vs yaw peak", F_pin, v["tau_pin"] * math.pi * 9, "N", 2.0)
chk("HC-11", "Tube wall bearing at Ø6 pin hole", F_pin / (6 * LB.TUBE_T), v["Sbr_6061"], "MPa", 1.5)
# 12 hard stop static (peak torque into pin)
F_stop = v["T_peak"] / (LB.STOP_R / 1000)
chk("HC-12", "Stop pin shear at peak torque", F_stop, v["tau_pin"] * math.pi * 9, "N", 2.0)
chk("HC-13", "Plate B bearing stress at stop slot", F_stop / (LB.STOP_PIN * LB.T_PB), v["Sbr_6061"], "MPa", 2.0)
# 14 stop impact energy (no bumper)
I_shin, w = 0.15, 75 * 2 * math.pi / 60
E = 0.5 * I_shin * w ** 2
rows.append({"id": "HC-14", "check": "Hard-stop impact energy at no-load speed (no bumper)", "demand": round(E, 2), "capacity": None,
             "unit": "J", "SF": None, "SF_required": None, "status": "FAIL",
             "note": "I=0.15 kg m^2 ASSUMED, 75 rpm; metal-on-metal stop needs elastomer bumper + software decel: NOT designed"})
# 15 foot plate bending (toe load)
M_f = 0.5 * W * v["k_dyn"] * 0.12 * 1000
Zf = 110 * 10 ** 2 / 6
chk("HC-15", "Foot plate 10 mm bending, toe load (dyn)", M_f / Zf, v["Sy_6061"], "MPa", 2.0, "ignores lightening cut-outs (Kt) -> verify")
# 16 power-off collapse
rows.append({"id": "HC-16", "check": "Power-off: AK80 backdrive torque 4.7 N*m vs squat knee/hip demand ~50 N*m", "demand": 50.0, "capacity": 4.7,
             "unit": "N*m", "SF": round(4.7 / 50, 2), "SF_required": 1.0, "status": "FAIL",
             "note": "robot collapses when de-energised: brakes or passive support required; gantry mandatory (SAFETY.md)"})

os.makedirs("docs/validation", exist_ok=True)
json.dump({"inputs": IN, "checks": rows}, open("docs/validation/hand_calcs_v0.2.json", "w"), indent=1)
L = ["<!-- SPDX-License-Identifier: CC-BY-4.0 -->", "# Hand calculations v0.2 (lower body)", "",
     "**These are hand calculations, not FEA and not certification.** They use simple beam, shear and bearing formulas,",
     "assumed factors and assumed material values (marked ASSUMPTION). Physical tests are required (see VERIFICATION.md).", "",
     "## Inputs", "| Symbol | Value | Unit | Source |", "|---|---|---|---|"]
L += [f"| {k} | {x[0]} | {x[1]} | {x[2]} |" for k, x in IN.items()]
L += ["", "## Checks", "| ID | Check | Demand | Capacity | Unit | SF | SF req | Status | Note |", "|---|---|---|---|---|---|---|---|---|"]
L += [f"| {r['id']} | {r['check']} | {r['demand']} | {r['capacity']} | {r['unit']} | {r['SF']} | {r['SF_required']} | **{r['status']}** | {r['note']} |" for r in rows]
L += ["", f"Summary: {sum(r['status']=='PASS' for r in rows)} PASS, {sum(r['status']=='FAIL' for r in rows)} FAIL. FAIL items are open design actions in docs/DESIGN_REVIEW.md."]
open("docs/HAND_CALCS.md", "w").write("\n".join(L) + "\n")
for r in rows: print(r["id"], r["status"], r["SF"], r["check"])
