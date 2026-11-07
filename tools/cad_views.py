# SPDX-License-Identifier: Apache-2.0
"""Assembled + exploded CAD views from the ACTUAL exported STL parts.

Writes hardware/assembly.json (instance transforms) and media/cad_*.png.
Placement is SCHEMATIC / ESTIMATED: it shows where each printed part goes, NOT a validated
mount or fit. Grey = conceptual metal structure (not designed yet). Red = actuator
size envelopes from datasheet dimensions (NOT manufacturer CAD). Run: python tools/cad_views.py
"""
import json, os, sys
import numpy as np, trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)
from ziggy.params import DIM

AX = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}
def R(px, pz):
    """Rotation given world images of part X and part Z (right-handed, part Y = Z x X)."""
    x, z = np.array(AX[px], float), np.array(AX[pz], float); y = np.cross(z, x)
    M = np.column_stack([x, y, z]); assert abs(np.linalg.det(M) - 1) < 1e-9; return M

mm = lambda v: v * 1000.0
HIP = mm(DIM["pelvis_to_hip"] * 0 + DIM["thigh"] + DIM["shin"] + DIM["ankle_height"])  # 870
KNEE = HIP - mm(DIM["thigh"]); ANK = KNEE - mm(DIM["shin"])
PELV = HIP + mm(DIM["pelvis_to_hip"]); T0 = PELV + mm(DIM["waist"]); T1 = T0 + mm(DIM["torso"])
SH = T1 - 50; ELB = SH - mm(DIM["upper_arm"]); WR = ELB - mm(DIM["forearm"])

def instances():
    I = []
    def add(part, pos, px, pz, explode, group, note="schematic placement"):
        I.append({"part": part, "pos_mm": [float(v) for v in pos], "x_to": px, "z_to": pz,
                  "explode_mm": [float(v) for v in explode], "group": group, "placement": note})
    for s, sy in (("l", 1), ("r", -1)):
        y = 100 * sy
        for part, zc, L in (("thigh_cover_half", (HIP + KNEE) / 2, 340), ("shin_cover_half", (KNEE + ANK) / 2, 330)):
            add(part, (0, y, zc - L / 2), "+z", "+x", (160, 60 * sy, 0), f"{s}_leg")   # front half
            add(part, (0, y, zc + L / 2), "-z", "-x", (-160, 60 * sy, 0), f"{s}_leg")   # back half
        add("foot_sole_pad", (30, y, 0), "+x", "+z", (0, 0, -80), f"{s}_foot")
        add("foot_cover", (30, y, 8), "+x", "+z", (0, 0, 120), f"{s}_foot")
        for zc in (HIP + 10, KNEE + 0.0):
            pass
        for part_pos, ax in (((0, y, HIP + 60), "+z"), ((-70, y, HIP), "+x"), ((0, y + 55 * sy, HIP - 10), "+y" if sy > 0 else "-y"),
                             ((0, y + 55 * sy, ANK + 40), "+y" if sy > 0 else "-y")):
            add("actuator_cover_ak80", part_pos, "+x" if ax != "+x" else "+y", ax, np.array(AX[ax]) * 90, f"{s}_leg",
                "schematic; envelope-level only")
        for k in range(3):
            add("cable_clip_tube40", (0, y, KNEE + 80 + 110 * k), "+x", "+z", (0, 120 * sy, 0), f"{s}_leg")
        ya = 200 * sy
        for part, zc, L in (("upper_arm_cover_half", (SH + ELB) / 2, 220), ("forearm_cover_half", (ELB + WR) / 2, 200)):
            add(part, (0, ya, zc - L / 2), "+z", "+x", (140, 80 * sy, 0), f"{s}_arm")
            add(part, (0, ya, zc + L / 2), "-z", "-x", (-140, 80 * sy, 0), f"{s}_arm")
        add("actuator_cover_ak80", (0, ya + 60 * sy, SH), "+x", "+y" if sy > 0 else "-y", (0, 120 * sy, 0), f"{s}_arm", "schematic; envelope-level only")
        add("actuator_cover_ak80", (0, ya + 50 * sy, ELB), "+x", "+y" if sy > 0 else "-y", (0, 120 * sy, 0), f"{s}_arm", "schematic; envelope-level only")
        hand(add, s, sy, (0, ya, WR - 40))
    add("pelvis_cover", (0, 0, PELV - 60), "+y", "+z", (0, 0, -60), "torso")
    add("torso_shell_front", (55, 0, (T0 + T1) / 2), "+y", "+z", (220, 0, 0), "torso")   # open face toward -x (interior)
    add("torso_shell_back", (-55, 0, (T0 + T1) / 2), "-y", "+z", (-220, 0, 0), "torso")
    add("estop_mount", (-112, 0, T1 - 60), "+y", "-x", (-300, 0, 0), "torso")
    hz = T1 + 50 + 90
    add("head_shell_half", (0, 0, hz), "+x", "+z", (0, 0, 140), "head")
    add("head_shell_half", (0, 0, hz), "+x", "-z", (0, 0, -60), "head")
    return I

def hand(add, s, sy, wrist, detail=False):
    wx, wy, wz = wrist; pz = wz - 47.5
    # palm: part X -> world y (finger spread), part Y -> world -z (finger direction), part Z -> -x
    add("palm_base", (0, wy, pz), "+y", "-x", (60, 0, 0), f"{s}_hand")
    bottom = pz - 45
    for i, fy in enumerate((-31.5, -10.5, 10.5, 31.5)):
        z = bottom
        for k, (part, L) in enumerate((("phalanx_proximal", 45), ("phalanx_middle", 30), ("phalanx_distal", 21.5))):
            add(part, (0, wy + fy * sy, z - L / 2), "-z", "+y", (0, 0, -25 * (k + 1)), f"{s}_hand", "schematic; pin fit untested")
            z -= L
    add("phalanx_proximal", (30, wy + 50 * sy, pz - 10), "-z", "+y", (40, 30 * sy, -10), f"{s}_hand", "schematic thumb")
    add("phalanx_distal", (30, wy + 50 * sy, pz - 43), "-z", "+y", (40, 30 * sy, -40), f"{s}_hand", "schematic thumb")
    for k in range(6):
        add("tendon_spool", (40, wy + (k % 2 - 0.5) * 30, ELB - 40 - 30 * (k // 2)), "+y", "+x", (120, 0, 0), f"{s}_hand", "schematic; servo not selected")

def metal_and_envelopes():
    """Conceptual (NOT designed) metal tubes + actuator envelopes from datasheet sizes."""
    G, E = [], []
    def tube(p0, p1, r=20):
        p0, p1 = np.array(p0, float), np.array(p1, float); L = np.linalg.norm(p1 - p0)
        c = trimesh.creation.cylinder(r, L, sections=24)
        c.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], (p1 - p0) / L) if L else np.eye(4))
        c.apply_translation((p0 + p1) / 2); G.append(c)
    def env(center, axis, kind):
        if kind == "RS04": c = trimesh.creation.box((120, 120, 56))
        elif kind == "AK70-10": c = trimesh.creation.cylinder(44.5, 50, sections=24)
        else: c = trimesh.creation.cylinder(49, 62, sections=24)
        c.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], AX[axis])); c.apply_translation(center); E.append((c, kind))
    for sy in (1, -1):
        y = 100 * sy; ya = 200 * sy; ax = "+y" if sy > 0 else "-y"
        tube((0, y, HIP - 30), (0, y, KNEE + 60)); tube((0, y, KNEE - 60), (0, y, ANK + 50))
        tube((0, ya, SH - 40), (0, ya, ELB + 40), 20); tube((0, ya, ELB - 40), (0, ya, WR + 20), 20)
        env((0, y, HIP + 60), "+z", "AK80-64"); env((-70, y, HIP), "+x", "AK80-64"); env((0, y + 55 * sy, HIP - 10), ax, "AK80-64")
        env((0, y + 45 * sy, KNEE), ax, "RS04"); env((0, y + 55 * sy, ANK + 40), ax, "AK80-64"); env((60, y, ANK + 40), "+x", "AK80-64")
        env((0, ya + 60 * sy, SH), ax, "AK80-64"); env((0, ya, SH - 70), "+x", "AK80-64"); env((0, ya, SH - 130), "+z", "AK70-10")
        env((0, ya + 50 * sy, ELB), ax, "AK80-64")
    b = trimesh.creation.box((180, 260, T1 - T0 - 40)); b.apply_translation((0, 0, (T0 + T1) / 2)); G.append(b)  # torso frame volume
    p = trimesh.creation.box((140, 260, 20)); p.apply_translation((0, 0, PELV)); G.append(p)
    env((0, 0, T0 - 10), "+z", "AK80-64"); env((0, 0, T1 + 20), "+z", "AK70-10")
    return G, E

COL = {"cover": (0.25, 0.55, 0.9), "functional": (1.0, 0.6, 0.1), "guide": (0.2, 0.75, 0.4), "coupon": (0.5, 0.5, 0.5)}

def placed(inst, man, explode=0.0):
    m = trimesh.load(man[inst["part"]]["stl"]); T = np.eye(4); T[:3, :3] = R(inst["x_to"], inst["z_to"])
    T[:3, 3] = np.array(inst["pos_mm"]) + explode * np.array(inst["explode_mm"]); m.apply_transform(T); return m

def draw(ax, mesh, color, alpha=1.0, lw=0):
    pc = Poly3DCollection(mesh.triangles, facecolor=color, edgecolor=(0, 0, 0, 0.08) if lw else "none", linewidth=lw, alpha=alpha)
    # simple Lambert shading
    n = mesh.face_normals; L = np.array([0.4, -0.5, 0.75]); L /= np.linalg.norm(L)
    sh = 0.45 + 0.55 * np.clip(n @ L, 0, 1); pc.set_facecolor(np.c_[np.outer(sh, color), np.full(len(sh), alpha)]); ax.add_collection3d(pc)

def render(path, insts, man, explode, title, groups=None, metal=True, view=(15, -60), lims=None, labels=(), zoom=1.0):
    fig = plt.figure(figsize=(8, 11), dpi=120); ax = fig.add_subplot(111, projection="3d")
    pts = []
    for i in insts:
        if groups and i["group"] not in groups: continue
        m = placed(i, man, explode); draw(ax, m, COL[man[i["part"]]["load_class"]]); pts.append(m.bounds)
    if metal:
        G, E = metal_and_envelopes()
        for g in G: draw(ax, g, (0.6, 0.6, 0.62), 0.35)
        for e, k in E: draw(ax, e, (0.85, 0.15, 0.15), 0.30)
    B = np.vstack(pts); lo, hi = (B.min(0), B.max(0)) if lims is None else lims
    c = (lo + hi) / 2; h = (hi - lo) / 2 * 1.08; h = np.maximum(h, h.max() * 0.25)
    ax.set_xlim(c[0] - h[0], c[0] + h[0]); ax.set_ylim(c[1] - h[1], c[1] + h[1]); ax.set_zlim(c[2] - h[2], c[2] + h[2])
    ax.set_box_aspect(tuple(h)); ax.set_proj_type("ortho")
    ax.view_init(*view); ax.set_axis_off()
    for (p, t) in labels: ax.text(*p, t, fontsize=8, color="k")
    leg = [Patch(color=COL["cover"], label="printed cover (STL from cad/parts.py)"),
           Patch(color=COL["functional"], label="printed functional (hand, spool, sole)"),
           Patch(color=COL["guide"], label="printed guide/clip/mount")]
    if metal:
        leg += [Patch(color=(0.6, 0.6, 0.62), alpha=0.5, label="CONCEPTUAL metal tube/frame - NOT designed"),
                Patch(color=(0.85, 0.15, 0.15), alpha=0.5, label="actuator ENVELOPE (datasheet size, not mfr CAD)")]
    fig.legend(handles=leg, loc="lower center", fontsize=8, frameon=True, ncol=2)
    fig.suptitle(title + "\nSCHEMATIC / ESTIMATED placement - not a validated mount or fit", fontsize=10)
    ax.set_box_aspect(tuple(h), zoom=zoom)
    fig.savefig(path, bbox_inches="tight"); plt.close(fig); _crop(path); print("wrote", path)

def _crop(path, pad=20):
    from PIL import Image, ImageOps
    im = Image.open(path).convert("RGB"); bb = ImageOps.invert(im).getbbox()
    if bb: im.crop((max(bb[0]-pad, 0), max(bb[1]-pad, 0), min(bb[2]+pad, im.width), min(bb[3]+pad, im.height))).save(path, optimize=True)

def main():
    man = {p["name"]: p for p in json.load(open("hardware/manifest.json"))["parts"]}
    I = instances()
    json.dump({"units": "mm", "frame": "world: origin on floor between feet, +x forward, +z up",
               "rotation": "x_to / z_to = world direction of part X / Z axis; part Y = Z x X",
               "status": "SCHEMATIC / ESTIMATED placement. Not a validated assembly; metal structure not designed.",
               "instances": I}, open("hardware/assembly.json", "w"), indent=1)
    cnt = {}
    for i in I: cnt[i["part"]] = cnt.get(i["part"], 0) + 1
    print("instances:", len(I), "of", len(cnt), "part types; manifest types:", len(man))
    lab = [((0, 330, 670), "thigh covers"), ((0, 330, 270), "shin covers"), ((0, 0, 1700), "head shells"),
           ((0, 420, 1000), "arm covers"), ((0, 330, 20), "foot cover + TPU sole"), ((150, 0, 1250), "torso shells")]
    render("media/cad_assembled.png", I, man, 0.0, "ZIGGY v0.1 - full body, assembled (printed parts)", labels=lab)
    render("media/cad_exploded.png", I, man, 1.0, "ZIGGY v0.1 - full body, exploded", labels=lab)
    hand_groups = {"l_hand"}
    hl = [((0, 215, WR - 90), "palm_base"), ((0, 215, WR - 180), "phalanges x3 (Ø3 pins)"), ((60, 215, ELB - 60), "tendon spools (servo TBD)"),
          ((40, 240, WR - 120), "thumb (schematic)")]
    render("media/cad_hand_assembled.png", I, man, 0.0, "Left hand + forearm spools - assembled", groups=hand_groups, metal=False, view=(12, -35), labels=hl, zoom=1.2)
    hle = [((0, 215, WR - 120), "palm_base"), ((0, 215, WR - 230), "proximal / middle / distal"), ((200, 215, ELB - 60), "tendon spools"),
           ((80, 240, WR - 150), "thumb (schematic)")]
    render("media/cad_hand_exploded.png", I, man, 1.5, "Left hand + forearm spools - exploded", groups=hand_groups, metal=False, view=(12, -35), labels=hle, zoom=1.2)

if __name__ == "__main__":
    main()
