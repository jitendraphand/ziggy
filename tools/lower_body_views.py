# SPDX-License-Identifier: Apache-2.0
"""Render v0.2 CAD views from the actual lower-body solids (cad/lower_body.py).

media/v02_lower_body_assembled.png, v02_lower_body_exploded.png, v02_knee_module.png,
media/v02_full_body_packaging.png.  Run: python tools/lower_body_views.py
"""
import json, os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch
from cad.lower_body import build, world, _place, T_of

COL = {"structure": (0.62, 0.68, 0.76), "envelope-purchased": (0.85, 0.2, 0.2), "printed": (0.98, 0.6, 0.15), "upper": (0.35, 0.55, 0.9)}
ALPHA = {"structure": 1.0, "envelope-purchased": 0.45, "printed": 0.85, "upper": 0.35}
LIGHT = np.array([0.3, -0.6, 0.75]) / np.linalg.norm([0.3, -0.6, 0.75])

def tris(shape, tol=0.6):
    v, t = shape.tessellate(tol, 0.4)
    V = np.array([(p.x, p.y, p.z) for p in v]); return V[np.array(t)] if len(t) else np.zeros((0, 3, 3))

def add(ax, T, kind):
    if not len(T): return
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    sh = 0.5 + 0.5 * np.abs(n @ LIGHT); c = np.outer(sh, COL[kind])
    ax.add_collection3d(Poly3DCollection(T, facecolors=np.c_[c, np.full(len(c), ALPHA[kind])], edgecolors="none"))

def setup(ax, lo, hi, view):
    c = (lo + hi) / 2; h = (hi - lo) / 2 * 1.03; h = np.maximum(h, h.max() * 0.45)
    ax.set_xlim(c[0] - h[0], c[0] + h[0]); ax.set_ylim(c[1] - h[1], c[1] + h[1]); ax.set_zlim(c[2] - h[2], c[2] + h[2])
    ax.set_box_aspect(tuple(h), zoom=1.15); ax.set_proj_type("ortho"); ax.view_init(*view); ax.set_axis_off()

LEG = [Patch(color=COL["structure"], label="metal structure - actual CAD (6061-T6 / steel), cad/lower_body.py"),
       Patch(color=COL["envelope-purchased"], alpha=0.5, label="purchased ENVELOPE (actuator/bearing datasheet size - not vendor CAD)"),
       Patch(color=COL["printed"], label="printed cover - actual CAD (PETG)")]

def figure(path, groups, title, views=((5, 0, "front (+x)"), (18, -45, "3/4 left-front")), labels=(), extra_leg=()):
    allT = np.vstack([t.reshape(-1, 3) for t, _ in groups if len(t)])
    lo, hi = allT.min(0), allT.max(0)
    fig = plt.figure(figsize=(8 * len(views), 10), dpi=110)
    for i, (el, az, name) in enumerate(views):
        ax = fig.add_subplot(1, len(views), i + 1, projection="3d")
        for t, k in groups: add(ax, t, k)
        setup(ax, lo, hi, (el, az)); ax.set_title(name, fontsize=12)
        if i == len(views) - 1:
            for p, s in labels: ax.text(*p, s, fontsize=9, color="k", bbox=dict(fc="white", ec="0.6", alpha=0.85, pad=1.5))
    fig.legend(handles=LEG + list(extra_leg), loc="lower center", ncol=2, fontsize=10, frameon=True)
    fig.suptitle(title, fontsize=13); fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0.08, wspace=0)
    fig.savefig(path); plt.close(fig); print("wrote", path)

EXP_Y = {"PLA": 90, "ACT": 45, "RCAP": 150, "ARA": 18, "ARB": -18, "PIN": -18, "SFT": -40, "SPC": -40, "BRG": -70, "PLB": -70, "CAP": -100,
         "CLP": -110, "BCOV": -140, "PBR": 0, "DBR": 0}

def exploded_T(p, s=1.0):
    tag = p.pid.split("-")[1] if p.pid.count("-") >= 2 else ""
    dz = p.stage * 110.0 * s
    dy_local = EXP_Y.get(tag, 0.0) * s
    zbr = {"PBR": 45, "DBR": -45}.get(tag, 0) * s
    off = p.T[:3, :3] @ np.array([0, dy_local, zbr])
    sign = -1 if p.pid.endswith("-R") else 1
    off = off + np.array([0, 0, -dz + 6 * 110 * s])  # spread stages vertically (foot stays lowest)
    T = p.T.copy(); T[:3, 3] = T[:3, 3] + off; return T

def main():
    parts, joints, lv = build()
    W = {p.pid: world(p) for p in parts}
    G = [(tris(W[p.pid]), p.kind) for p in parts]
    zk, zh, za = lv["knee"], lv["hip_pitch"], lv["ankle_pitch"]
    lab = [((0, 330, lv["hip_roll"]), "hip roll module (AK80-64 env.)"), ((0, 330, zh), "hip pitch module"),
           ((0, 330, zk), "knee module (RS04 env., 6004 bearing)"), ((0, 330, za), "ankle pitch module"),
           ((0, 330, lv["ankle_roll"]), "ankle roll module + foot plate"), ((0, 330, lv["pelvis_plate_bottom"] + 20), "pelvis plate + hip yaw (AK80-64)"),
           ((0, 330, (zk + zh) / 2), "thigh tube 40x3 (clamp + Ø6 cross pin)")]
    figure("media/v02_lower_body_assembled.png", G, "ZIGGY v0.2 lower body - assembled (actual CAD; preliminary, unverified)", labels=lab)
    GE = [(tris(_place(p.shape, exploded_T(p))), p.kind) for p in parts if not p.pid.endswith("-R")]
    figure("media/v02_lower_body_exploded.png", GE, "v0.2 left leg - exploded by module (stages spread vertically, parts along joint axis)",
           views=((8, 0, "front (+x)"), (18, -45, "3/4")))
    # knee closeup: knee module + tube stubs
    kn = [p for p in parts if p.module == "KN" and p.pid.endswith("-L")]
    GK = [(tris(world(p)), p.kind) for p in kn]
    GKe = [(tris(_place(p.shape, exploded_T(p, 1.0) - np.r_[np.zeros((3, 4)), [[0, 0, 0, 0]]] * 0)), p.kind) for p in kn]
    # exploded closeup: only local-y explode (no stage spread)
    def kexp(p):
        T = exploded_T(p); T[2, 3] = p.T[2, 3] + (T[2, 3] - p.T[2, 3] - (6 - p.stage) * 110.0); return T
    GKe = [(tris(_place(p.shape, kexp(p))), p.kind) for p in kn]
    y0 = 110
    ze = zk - 420
    kl = [((0, y0 + 240, ze + 150), "plate A\n(stator)"), ((0, y0 + 180, ze - 160), "RS04 envelope\nmount BLOCKED"),
          ((0, y0 + 160, ze + 170), "arm A'\n(output)"), ((0, y0 + 20, ze + 175), "proximal bridge\n(clamp)"), ((0, y0 + 20, ze - 175), "distal bridge"),
          ((0, y0 + 45, ze - 120), "arm B' + pin\n+ shaft"), ((0, y0 - 25, ze + 120), "plate B (Ø42 H7,\nstop slot) + 6004"),
          ((0, y0 - 110, ze - 120), "cap, circlip,\nprinted cover")]
    figure("media/v02_knee_module.png", GK + [(t + np.array([0, 0, -420]), k) for t, k in GKe],
           "Knee module: assembled (top) and exploded along the axis (bottom)", views=((15, -60, "3/4"), (0, 0, "front (+x)")), labels=kl)
    # full-body packaging: actual lower body + v0.1 upper printed parts placed above pelvis (conceptual, blue translucent)
    import trimesh
    man = {p["name"]: p for p in json.load(open("hardware/manifest.json"))["parts"]}
    top = lv["pelvis_plate_bottom"] + 8
    up = []
    def put(name, R, t):
        m = trimesh.load(man[name]["stl"]); M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; m.apply_transform(M); up.append((m.triangles, "upper"))
    Ry = np.column_stack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]]).astype(float)  # part x->+y, y->-x, z->+z
    Ryb = np.column_stack([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).astype(float)
    put("torso_shell_front", Ry, (55, 0, top + 230)); put("torso_shell_back", Ryb, (-55, 0, top + 230))
    HZ = top + 450 + 30 + 110   # neck gap 30 mm + head radius 110 (v0.1 shell)
    put("head_shell_half", np.eye(3), (0, 0, HZ)); put("head_shell_half", np.diag([1, -1, -1.0]), (0, 0, HZ))
    for sy in (1, -1):
        for name, zc, L in (("upper_arm_cover_half", top + 330, 220), ("forearm_cover_half", top + 80, 200)):
            Rz = np.column_stack([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]).astype(float)
            put(name, Rz, (0, sy * 210, zc - L / 2))
    lab2 = [((0, 250, top + 230), "torso/arms/head: v0.1 printed\nshells, CONCEPTUAL placement"), ((0, 330, zh), "v0.2 lower body (actual CAD)"),
            ((0, 250, top + 700), f"top of head ≈ {HZ + 110:.0f} mm\n(provisional target 1650-1700: FAIL)")]
    figure("media/v02_full_body_packaging.png", G + up, "ZIGGY v0.2 whole-body packaging study (upper body NOT redesigned)", labels=lab2,
           extra_leg=[Patch(color=COL["upper"], alpha=0.4, label="upper body v0.1 printed shells - conceptual placement only")])
    json.dump({"stature_estimate_mm": round(HZ + 110, 0), "basis": "v0.2 lower-body CAD + v0.1 torso (440) + 30 neck + 220 head"}, open("media/v02_packaging.json", "w"))

if __name__ == "__main__":
    main()
