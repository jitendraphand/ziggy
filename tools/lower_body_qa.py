# SPDX-License-Identifier: Apache-2.0
"""v0.2 lower-body: export parts, write manufacturing + assembly manifests, and run
geometric QA (interference, joint-travel sweep, hard-stop engagement, print envelope,
CAD mass rollup, CAD-vs-sim consistency). Evidence -> docs/validation/lower_body_v0.2.json

    python tools/lower_body_qa.py            # full (≈1-3 min)
"""
import hashlib, json, math, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)
import numpy as np, cadquery as cq
from cad.lower_body import build, world, _place, T_of, STOP_MARGIN, JOINTS
from ziggy.params import print_envelope_mm, DIM, MASS

OUT = "hardware/lower_body"
TOUCH_MM3 = 1.0      # common volume <= 1 mm^3 counts as contact (coincident faces / tessellation noise), not interference

def common_vol(a, b):
    ba, bb = a.BoundingBox(), b.BoundingBox()
    if (ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin):
        return 0.0
    try:
        return float(a.intersect(b).Volume())
    except Exception:
        return float("nan")

def export(parts):
    os.makedirs(f"{OUT}/step", exist_ok=True); os.makedirs(f"{OUT}/stl", exist_ok=True)
    man = []
    for p in parts:
        if p.pid.endswith("-R"):
            src = p.pid[:-2] + "-L"
            man.append(_row(p, None, None, mirror_of=src)); continue
        st = f"{OUT}/step/{p.pid}.step"; cq.exporters.export(cq.Workplane().add(p.shape), st)
        sl = None
        if p.kind != "envelope-purchased":
            sl = f"{OUT}/stl/{p.pid}.stl"; cq.exporters.export(cq.Workplane().add(p.shape), sl, tolerance=0.1, angularTolerance=0.3)
        man.append(_row(p, st, sl))
    return man

def _row(p, st, sl, mirror_of=None):
    bb = p.shape.BoundingBox()
    return {"id": p.pid, "module": p.module, "kind": p.kind, "make_buy": p.make_buy, "material": p.material, "process": p.process,
            "qty": 1, "critical": p.crit, "status": p.status, "flags": p.flags, "printed": p.printed,
            "print_orientation": p.orient or None, "bbox_mm": [round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
            "volume_cm3": round(p.shape.Volume() / 1000, 2), "mass_kg": round(p.mass(), 4),
            "mass_source": "purchased datasheet" if p.mass_override is not None else f"CAD volume x density({p.material})",
            "step": st, "stl": sl, "mirror_of": mirror_of,
            "stl_note": ("METAL PART - STL for visualisation/CAM only, NOT for printing" if (sl and not p.printed) else None)}

def assembly(parts):
    return {"units": "mm", "frame": "world: origin floor mid-plane between feet, +x forward, +y left, +z up",
            "transform": "4x4 part-local -> world (right-side parts are mirrored geometry; their transform is the mirrored left one)",
            "instances": [{"id": p.pid, "T": [[round(float(v), 6) for v in r] for r in p.T], "module": p.module, "stage": p.stage,
                           "kind": p.kind} for p in parts]}

def interference(W, parts):
    hits, touches, n = [], 0, 0
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            v = common_vol(W[i], W[j]); n += 1
            if v != v: hits.append((parts[i].pid, parts[j].pid, "boolean-failed"))
            elif v > TOUCH_MM3: hits.append((parts[i].pid, parts[j].pid, round(v, 2)))
            elif v > 0: touches += 1
    return {"pairs_checked": n, "interferences": hits, "contacts_le_1mm3": touches}

def rot_about(origin, axis, deg):
    a = np.array(axis, float); a /= np.linalg.norm(a); t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K
    o = np.array(origin, float); return T_of(R, o - R @ o)

def sweep(parts, joints, side="l", step=10.0):
    """For each joint: rotate all downstream parts (stage >= k) about the joint axis, check vs upstream."""
    L = [p for p in parts if p.pid.endswith("-" + side.upper())] + [p for p in parts if p.pid == "PELVIS-PLATE"]
    res = {}
    for k, j in enumerate(JOINTS, start=1):
        J = joints[side][j]; lo, hi = J["range_deg"]
        down = [p for p in L if p.stage >= k]; up = [p for p in L if p.stage < k]
        if j in ("hip_yaw", "hip_roll"):   # also check against the (fixed, zero-pose) opposite leg
            up = up + [p for p in parts if p.pid.endswith("-" + ("R" if side == "l" else "L"))]
        Wup = [world(p) for p in up]
        angles = sorted(set([lo, hi, 0.0] + list(np.arange(math.ceil(lo / step) * step, hi, step))))
        worst = []
        for a in angles:
            M = rot_about(J["origin"], J["axis"], a)
            for p in down:
                wd = _place(p.shape, M @ p.T)
                for q, wu in zip(up, Wup):
                    v = common_vol(wd, wu)
                    if v > TOUCH_MM3 or v != v: worst.append({"angle": float(a), "moving": p.pid, "fixed": q.pid, "vol_mm3": round(v, 1)})
        stop = None
        pin = next((p for p in down if p.pid.startswith(f"{_mod(j)}-PIN")), None)
        plb = next((p for p in up if p.pid.startswith(f"{_mod(j)}-PLB")), None)
        if pin and plb:
            stop = {}
            for lab, a in (("lo-4", lo - STOP_MARGIN - 2), ("lo", lo), ("hi", hi), ("hi+4", hi + STOP_MARGIN + 2)):
                M = rot_about(J["origin"], J["axis"], a)
                stop[lab] = round(common_vol(_place(pin.shape, M @ pin.T), world(plb)), 2)
            stop["engages_beyond_range"] = stop["lo-4"] > TOUCH_MM3 and stop["hi+4"] > TOUCH_MM3
            stop["free_within_range"] = stop["lo"] <= TOUCH_MM3 and stop["hi"] <= TOUCH_MM3
        res[j] = {"range_deg": [lo, hi], "samples": len(angles), "collisions": worst, "hard_stop": stop}
    return res

def _mod(j):
    return {"hip_roll": "HR", "hip_pitch": "HP", "knee": "KN", "ankle_pitch": "AP", "ankle_roll": "AR", "hip_yaw": "HY"}[j]

def main(quick=False):
    t0 = time.time()
    parts, joints, levels = build()
    man = export(parts)
    json.dump({"generator": "cad/lower_body.py", "units": "mm", "parts": man}, open(f"{OUT}/manifest.json", "w"), indent=1)
    json.dump(assembly(parts), open(f"{OUT}/assembly.json", "w"), indent=1)
    W = [world(p) for p in parts]
    inter = interference(W, parts)
    sw = sweep(parts, joints, "l")
    env = print_envelope_mm()
    prints = {p.pid: [round(v, 1) for v in (p.shape.BoundingBox().xlen, p.shape.BoundingBox().ylen, p.shape.BoundingBox().zlen)]
              for p in parts if p.printed and p.pid.endswith("-L")}
    fits = {k: all(sorted(v)[i] <= sorted(env)[i] for i in range(3)) for k, v in prints.items()}
    mass = {"structure_kg": sum(p.mass() for p in parts if p.kind == "structure"),
            "purchased_known_kg": sum(p.mass() for p in parts if p.kind == "envelope-purchased"),
            "printed_kg": sum(p.mass() for p in parts if p.printed),
            "fasteners_wiring_kg": None, "note": "fasteners, wiring, encoders, elastomer bumpers NOT included (unknown)"}
    mass["total_known_kg"] = mass["structure_kg"] + mass["purchased_known_kg"] + mass["printed_kg"]
    sim_lb = MASS["pelvis"] + 2 * (MASS["hip_yaw"] + MASS["hip_roll"] + MASS["thigh"] + MASS["shin"] + 0.3 + MASS["foot"])
    mass["sim_v0.1_lower_body_budget_kg"] = sim_lb
    sim_z = {"ankle_roll": DIM["ankle_height"] * 1000, "ankle_pitch": DIM["ankle_height"] * 1000,
             "knee": (DIM["ankle_height"] + DIM["shin"]) * 1000, "hip_pitch": (DIM["ankle_height"] + DIM["shin"] + DIM["thigh"]) * 1000,
             "hip_roll": 870.0, "yaw_face": 870.0}
    cons = {k: {"cad_mm": round(levels[k], 1), "sim_mm": round(sim_z[k], 1), "delta_mm": round(levels[k] - sim_z[k], 1)} for k in sim_z}
    ev = {"version": "0.2.0", "date": time.strftime("%Y-%m-%d"), "runtime_s": None,
          "counts": {"instances": len(parts), "unique_left_or_center": sum(1 for p in parts if not p.pid.endswith("-R")),
                     "by_kind": {k: sum(1 for p in parts if p.kind == k) for k in ("structure", "envelope-purchased", "printed")}},
          "levels_mm": {k: round(v, 1) for k, v in levels.items()}, "interference": inter, "joint_sweep_left": sw,
          "printed_fit": {"envelope_mm": env, "parts": prints, "fits": fits},
          "mass": {k: (round(v, 2) if isinstance(v, float) else v) for k, v in mass.items()},
          "cad_vs_sim": cons}
    ev["runtime_s"] = round(time.time() - t0, 1)
    os.makedirs("docs/validation", exist_ok=True)
    json.dump(ev, open("docs/validation/lower_body_v0.2.json", "w"), indent=1)
    print(json.dumps({k: ev[k] for k in ("counts", "levels_mm", "mass", "runtime_s")}, indent=1))
    print("interferences:", inter["interferences"][:20], "contacts:", inter["contacts_le_1mm3"])
    for j, r in sw.items(): print(j, r["range_deg"], "collisions:", len(r["collisions"]), r["collisions"][:4], "stop:", r["hard_stop"])

if __name__ == "__main__":
    main()
