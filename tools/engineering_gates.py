# SPDX-License-Identifier: Apache-2.0
"""Compute v0.2 engineering gates from evidence files; write docs/ENGINEERING_GATES.{json,md}
and hardware/lower_body/bom.csv. A gate's status is DERIVED from evidence and cannot be set by hand
(tests/test_lower_body.py recomputes and compares)."""
import csv, json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)

def compute():
    ev = json.load(open("docs/validation/lower_body_v0.2.json"))
    hc = json.load(open("docs/validation/hand_calcs_v0.2.json"))["checks"]
    pk = json.load(open("media/v02_packaging.json"))
    man = json.load(open("hardware/lower_body/manifest.json"))["parts"]
    sw = ev["joint_sweep_left"]
    G = []
    def gate(gid, desc, ok, evidence, detail):
        G.append({"id": gid, "gate": desc, "status": "PASS" if ok else "FAIL", "evidence": evidence, "detail": detail})
    gate("G-01", "Static interference (all pairs, >1 mm^3) = 0", len(ev["interference"]["interferences"]) == 0,
         "docs/validation/lower_body_v0.2.json", f"{ev['interference']['pairs_checked']} pairs")
    gate("G-02", "Joint travel sweep (left leg, 10 deg steps + limits) collision-free", all(not r["collisions"] for r in sw.values()),
         "joint_sweep_left", {j: len(r["collisions"]) for j, r in sw.items()})
    stops = {j: (r["hard_stop"] or {}).get("engages_beyond_range", False) and (r["hard_stop"] or {}).get("free_within_range", False) for j, r in sw.items()}
    gate("G-03", "Mechanical hard stop on every leg joint (engages at limit+4 deg, free inside range)", all(stops.values()),
         "joint_sweep_left.hard_stop", stops)
    gate("G-04", "Printed parts fit Neptune 4 Max envelope", all(ev["printed_fit"]["fits"].values()), "printed_fit", len(ev["printed_fit"]["fits"]))
    gate("G-05", "Hand calcs: all checks meet required SF", all(r["status"] == "PASS" for r in hc), "docs/HAND_CALCS.md",
         [r["id"] for r in hc if r["status"] != "PASS"])
    gate("G-06", "Lower-body CAD mass <= sim v0.1 lower-body budget", ev["mass"]["total_known_kg"] <= ev["mass"]["sim_v0.1_lower_body_budget_kg"],
         "mass", f"{ev['mass']['total_known_kg']} kg vs {ev['mass']['sim_v0.1_lower_body_budget_kg']} kg (fasteners/wiring excluded)")
    gate("G-07", "CAD joint heights match sim within 10 mm", all(abs(v["delta_mm"]) <= 10 for v in ev["cad_vs_sim"].values()), "cad_vs_sim",
         {k: v["delta_mm"] for k, v in ev["cad_vs_sim"].items()})
    gate("G-08", "Stature within provisional target 1650-1700 mm", 1650 <= pk["stature_estimate_mm"] <= 1700, "media/v02_packaging.json",
         pk["stature_estimate_mm"])
    blockers = sorted({f for p in man for f in p["flags"] if f.startswith("BLOCKER")})
    gate("G-09", "No open interface blockers (vendor drawings)", not blockers, "hardware/lower_body/manifest.json", blockers)
    gate("G-10", "Physical verification (print/fit/load/stop/E-stop tests) recorded", False, "none", "no hardware built")
    gate("G-11", "Self-collision / leg-to-leg sweep for all joints", False, "joint_sweep_left", "opposite leg only checked for hip yaw/roll")
    return G

FASTENERS = {  # per AK80 module / per RS04 module / per interface (counts from CAD hole features)
    "ISO 4762 M3 12.9 (actuator rear, length TBD)": ("AK", 8), "ISO 4762 M4 12.9 (actuator output, length TBD)": ("AK", 6),
    "ISO 4762 M4 (bearing cap)": ("ALL", 3), "ISO 4762 M5 12.9 (bridge-plate/arm)": ("ALL", 8), "ISO 4762 M3 (printed rear cap)": ("AK", 3),
}

def bom():
    man = json.load(open("hardware/lower_body/manifest.json"))["parts"]
    rows = [{"id": p["id"], "make_buy": p["make_buy"], "kind": p["kind"], "material": p["material"], "process": p["process"],
             "qty": 1, "mass_kg": p["mass_kg"], "critical": p["critical"], "status": p["status"],
             "source_file": p["step"] or f"mirror of {p['mirror_of']}"} for p in man]
    mods = {p["module"] for p in man if p["id"].endswith("-PLA-L")}
    n_ak = sum(1 for m in mods if m != "KN") * 2 + 2   # 4 AK modules x2 legs + 2 hip yaw
    n_all = len(mods) * 2
    for name, (kind, per) in FASTENERS.items():
        n = per * (n_ak if kind == "AK" else n_all)
        rows.append({"id": name, "make_buy": "buy", "kind": "fastener", "material": "steel 12.9", "process": "purchased", "qty": n,
                     "mass_kg": None, "critical": "length/torque per assembly drawing (NOT yet issued)", "status": "estimated count", "source_file": "hole features"})
    flat_if = 4 * 2 + 2   # flat interfaces per leg x2 + yaw out
    rows.append({"id": "ISO 4762 M6 12.9 (module interfaces, clamps)", "make_buy": "buy", "kind": "fastener", "material": "steel 12.9", "process": "purchased",
                 "qty": flat_if * 4 + 6 * 2, "mass_kg": None, "critical": "torque TBD", "status": "estimated count", "source_file": "hole features"})
    with open("hardware/lower_body/bom.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    return len(rows)

def main():
    G = compute()
    json.dump(G, open("docs/ENGINEERING_GATES.json", "w"), indent=1)
    L = ["<!-- SPDX-License-Identifier: CC-BY-4.0 -->", "# Engineering gates v0.2 (derived from evidence; regenerate with tools/engineering_gates.py)", "",
         "| ID | Gate | Status | Evidence | Detail |", "|---|---|---|---|---|"]
    L += [f"| {g['id']} | {g['gate']} | **{g['status']}** | {g['evidence']} | {json.dumps(g['detail'], ensure_ascii=False)[:180]} |" for g in G]
    L += ["", f"{sum(g['status']=='PASS' for g in G)} PASS / {sum(g['status']=='FAIL' for g in G)} FAIL. Manufacturing release requires all PASS; it is **blocked**."]
    open("docs/ENGINEERING_GATES.md", "w").write("\n".join(L) + "\n")
    n = bom()
    for g in G: print(g["id"], g["status"], g["gate"])
    print("bom rows", n)

if __name__ == "__main__":
    main()
