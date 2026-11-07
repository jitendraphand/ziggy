# SPDX-License-Identifier: CERN-OHL-S-2.0
"""ZIGGY v0.2 lower-body structural assembly (parametric, CadQuery/OpenCascade).

Architecture: each actuated joint is a removable "dual-support yoke module" whose local
frame has the joint axis on +Y (outboard), proximal link on +Z, distal link on -Z:

    y:  [plate B + 6004 bearing][arm B'] ... gap G (link/bridge) ... [arm A'][actuator][plate A]
         ^ shaft side support                                         ^ actuator output bearing

* Proximal yoke  = plate A (actuator stator) + plate B (bearing housing) + proximal bridge.
* Distal yoke    = arm A' (bolted to actuator output) + arm B' (stub shaft in 6004) + distal bridge.
* Bridges are either tube clamps (split clamp + cross pin on a 40x3 6061-T6 tube) or flat
  interface blocks with a 4xM6 square pattern so modules unbolt from each other.
* Mechanical hard stop: Ø6 hardened pin in arm B' running in an arc slot in plate B
  (slot = joint range + STOP_MARGIN). Elastomer bumper: NOT designed (see HAND_CALCS).

All purchased parts (actuators, bearings) are dimensioned ENVELOPES, not vendor CAD.
AK80-64 interface dims are read from the CubeMars 2D drawing (docs/REFERENCES.md); items we
could not read unambiguously carry flags. RobStride 04 interface: NOT AVAILABLE -> blocker.
"""
import math
import numpy as np
import cadquery as cq

# ------------------------------------------------------------------ parameters (mm)
AK80 = {  # CubeMars AK80-64 2D drawing (ak80-64-robotic-actuator-2d-drawing.pdf)
    "kind": "AK80-64", "body_d": 98.0, "body_l": 42.7, "boss_d": 71.0, "boss_l": 11.2,
    "rear_d": 80.0, "rear_l": 8.0, "rear_pcd": 85.0, "rear_n": 8, "rear_phase_deg": 15.0,
    "out_pcd": 28.0, "out_n": 6, "pilot_d": 21.0, "pilot_depth": 4.5, "mass_kg": 0.85,
    "flags": ["rear 8xM3 angular phase reference ambiguous on drawing (15 deg assumed from vertical)",
              "2x dowel Ø3 location not dimensioned on drawing -> not drilled",
              "rear M3 PCD Ø85 vs Ø80 step leaves ~0.7 mm web in adapter: verify with vendor STEP"],
}
RS04 = {"kind": "RS04", "box": 120.0, "len": 56.0, "mass_kg": 1.42,
        "flags": ["BLOCKER: RobStride 04 flange/mount drawing not obtained; adapter holes NOT drilled"]}
B6004 = {"d": 20.0, "D": 42.0, "B": 12.0, "mass_kg": 0.0656, "C_kN": 9.95, "C0_kN": 5.0}  # SKF 6004 product page

TUBE_OD, TUBE_T = 40.0, 3.0
G = 64.0          # inner gap between distal arms (= distal bridge width)
T_ARM, T_PA, T_PB = 8.0, 8.0, 12.0
AX_GAP = 1.0      # axial running gap arm B' <-> plate B (filled by spacer at the shaft)
R_PLATE, R_ARM = 56.0, 40.0
H_TUBE, H_FLAT = 32.0, 20.0
WX = 60.0         # bridge x width
STOP_R, STOP_PIN, STOP_MARGIN = 33.0, 6.0, 2.0
PIN_CLR = 0.5
HIP_Y = 110.0     # leg line from mid-plane
DENS = {"6061-T6": 2.70, "42CrMo4": 7.85, "steel": 7.85, "PETG": 1.27, "TPU-95A": 1.21}

# ------------------------------------------------------------------ primitives
def cyl_y(r, y0, y1, x=0.0, z=0.0):
    return cq.Solid.makeCylinder(r, y1 - y0, cq.Vector(x, y0, z), cq.Vector(0, 1, 0))

def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0), cq.Vector(0, 0, 1))

def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return cq.Solid.makeCylinder(r, x1 - x0, cq.Vector(x0, y, z), cq.Vector(1, 0, 0))

def box(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0))

def U(*s):
    r = s[0]
    for o in s[1:]: r = r.fuse(o)
    return r.clean()

def C(a, *s):
    for o in s: a = a.cut(o)
    return a.clean()

def prism_xz(pts, y0, y1):
    """Polygon given in (x, z) extruded over y in [y0, y1]."""
    w = cq.Workplane("XY").polyline(pts).close().extrude(y1 - y0).val()
    w = w.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)   # (x,y,z)->(x,-z,y): extrusion -> -y
    return w.translate(cq.Vector(0, y1, 0))

def pin_point(theta_deg, r=STOP_R):
    t = math.radians(theta_deg)
    return (-r * math.sin(t), -r * math.cos(t))  # rotation about +Y of (0,-r)

def arc_slot(lo, hi, y0, y1, r=STOP_R, w=STOP_PIN / 2 + PIN_CLR):
    a0, a1 = lo - STOP_MARGIN, hi + STOP_MARGIN
    n = max(8, int((a1 - a0) / 2))
    outer = [pin_point(a0 + (a1 - a0) * i / n, r + w) for i in range(n + 1)]
    inner = [pin_point(a1 - (a1 - a0) * i / n, r - w) for i in range(n + 1)]
    s = prism_xz(outer + inner, y0, y1)
    for a in (a0, a1):
        x, z = pin_point(a); s = s.fuse(cyl_y(w, y0, y1, x, z))
    return s.clean()

# ------------------------------------------------------------------ parts container
class Part:
    def __init__(self, pid, shape, *, material, process, make_buy, kind, module, stage, crit="", status="preliminary",
                 flags=(), mass_kg=None, printed=False, orient=""):
        self.pid, self.shape, self.material, self.process = pid, shape, material, process
        self.make_buy, self.kind, self.module, self.stage = make_buy, kind, module, stage
        self.crit, self.status, self.flags, self.mass_override = crit, status, list(flags), mass_kg
        self.printed, self.orient = printed, orient
        self.T = np.eye(4)

    def mass(self):
        if self.mass_override is not None: return self.mass_override
        return self.shape.Volume() / 1000.0 * DENS[self.material] / 1000.0

def _place(shape, T):
    m = cq.Matrix([list(map(float, T[i])) for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]) if False else None
    loc = cq.Location(cq.Vector(*T[:3, 3]))
    # build rotation via gp_Trsf for exact orthonormal matrices
    from OCP.gp import gp_Trsf
    tr = gp_Trsf()
    R = T[:3, :3]
    tr.SetValues(R[0, 0], R[0, 1], R[0, 2], T[0, 3], R[1, 0], R[1, 1], R[1, 2], T[1, 3], R[2, 0], R[2, 1], R[2, 2], T[2, 3])
    return shape.moved(cq.Location(tr))

# ------------------------------------------------------------------ module builder
def actuator_envelope(act, y_face):
    """Envelope starting at output face plane y_face, extending outboard (+y)."""
    if act["kind"] == "AK80-64":
        y1 = y_face + act["boss_l"]; y2 = y1 + act["body_l"]; y3 = y2 + act["rear_l"]
        e = U(cyl_y(act["boss_d"] / 2, y_face, y1), cyl_y(act["body_d"] / 2, y1, y2), cyl_y(act["rear_d"] / 2, y2, y3))
        e = C(e, cyl_y(act["pilot_d"] / 2, y_face, y_face + act["pilot_depth"]))
        return e, y2, y3            # body rear face, step end
    b = act["box"] / 2
    e = box(-b, b, y_face, y_face + act["len"], -b, b)
    return e, y_face + act["len"], y_face + act["len"]

def bridge(kind, side, y0, y1, z0, z1, tube):
    """Bridge block spanning y0..y1, z0..z1 (side='prox' -> tube exits +z, 'dist' -> -z)."""
    b = box(-WX / 2, WX / 2, y0, y1, z0, z1)
    ym = (y0 + y1) / 2  # = 0 by construction
    holes = []
    for x in (-20.0, 20.0):           # M5 tapped (Ø4.2) from both y faces, 12 deep, at mid height
        zc = (z0 + z1) / 2 if kind == "flat" else (z0 + 8 if side == "prox" else z1 - 8)
        holes += [cyl_y(2.1, y0, y0 + 12, x, zc), cyl_y(2.1, y1 - 12, y1, x, zc)]
    if kind == "tube":
        holes.append(cyl_z(TUBE_OD / 2, z0, z1))                       # tube socket (through)
        holes.append(box(TUBE_OD / 2 - 2, WX / 2 + 1, -0.75, 0.75, z0, z1))  # clamp slit
        for zc in (z0 + (z1 - z0) * 0.3, z0 + (z1 - z0) * 0.7):
            holes.append(cyl_y(3.2, max(y0, -28.0), min(y1, 28.0), 25.0, zc))   # 2x M6 clamp (Ø6.4), local boss
        holes.append(cyl_y(3.0, -TUBE_OD / 2 - 8, TUBE_OD / 2 + 8, 0.0, (z0 + z1) / 2))  # Ø6 cross pin
    else:
        zf = z1 if side == "dist" else z0  # unused; pattern is through
        for x in (-22.0, 22.0):
            for y in (-22.0, 22.0):
                holes.append(cyl_z(3.3, z0, z1, x, y))                 # 4x M6 clearance Ø6.6
        holes.append(cyl_z(9.0, z0, z1))                               # Ø18 harness pass-through
    if y1 - y0 > 100:   # lightening pockets either side of the central boss; also wrench access to clamp bolts
        for (a0, a1) in ((y0 + 14, -28.0), (28.0, y1 - 14)):  # noqa
            if a1 - a0 > 6:
                holes.append(box(-WX / 2 + 5, WX / 2 - 5, a0, a1, z0, z1 - 5) if side == "prox" else
                             box(-WX / 2 + 5, WX / 2 - 5, a0, a1, z0 + 5, z1))
    return C(b, *holes)

def module(name, act, prox, dist, lo, hi, stage):
    """Return list of Parts in MODULE-LOCAL frame, plus geometry info."""
    P = []
    flags = act["flags"]
    Dp = (act["body_d"] / 2 + 4) if act["kind"] == "AK80-64" else act["box"] / 2 + 4
    Dp = max(Dp, R_PLATE)
    if act["kind"] == "RS04":
        Dp += 8.0   # fix F-009: knee arms clipped proximal bridge at 130 deg (41 mm^3) -> +8 mm clearance
    Db = R_PLATE
    Hp = H_TUBE if prox == "tube" else H_FLAT
    Hd = H_TUBE if dist == "tube" else H_FLAT
    yA_arm0, yA_arm1 = G / 2, G / 2 + T_ARM
    env, y_rear, y_step = actuator_envelope(act, yA_arm1)
    yPA0, yPA1 = y_rear, y_rear + T_PA
    yB_arm0, yB_arm1 = -G / 2 - T_ARM, -G / 2
    yPB1 = yB_arm0 - AX_GAP; yPB0 = yPB1 - T_PB
    ztop = Dp
    # plates (waterjet + CNC finish): circle around axis + web up to the bridge
    def plate(y0, y1):
        return U(cyl_y(R_PLATE, y0, y1), box(-R_PLATE, R_PLATE, y0, y1, 0, ztop + Hp))
    pa = plate(yPA0, yPA1)
    if act["kind"] == "AK80-64":
        pa = C(pa, cyl_y(act["rear_d"] / 2 + 0.2, yPA0, yPA1))       # Ø80.4 H8 locating bore (step)
        for k in range(act["rear_n"]):
            a = math.radians(act["rear_phase_deg"] + 360 * k / act["rear_n"])
            pa = pa.cut(cyl_y(1.6, yPA0, yPA1, act["rear_pcd"] / 2 * math.sin(a), act["rear_pcd"] / 2 * math.cos(a)))
    else:
        pa = C(pa, cyl_y(20.0, yPA0, yPA1))                           # cable hole only; pattern BLOCKED
    for k in range(3):                                               # 3x M3 tapped for rear cap (Ø2.5)
        a = math.radians(60 + 120 * k)
        pa = pa.cut(cyl_y(1.25, yPA0, yPA1, 50 * math.sin(a), 50 * math.cos(a)))
    pb = plate(yPB0, yPB1)
    pb = C(pb, cyl_y(B6004["D"] / 2, yPB0, yPB1), arc_slot(lo, hi, yPB0, yPB1))
    for k in range(3):
        a = math.radians(90 + 120 * k)
        pb = pb.cut(cyl_y(1.65, yPB0, yPB1, 25 * math.sin(a), 25 * math.cos(a)))   # M4 tapped (Ø3.3) cap
    for pl, (y0, y1) in ((pa, (yPA0, yPA1)), (pb, (yPB0, yPB1))):
        pass
    # bridge-to-plate M5 clearance holes in plates
    zb = (ztop + ztop + Hp) / 2 if prox == "flat" else ztop + 8
    for x in (-20.0, 20.0):
        pa = pa.cut(cyl_y(2.75, yPA0, yPA1, x, zb)); pb = pb.cut(cyl_y(2.75, yPB0, yPB1, x, zb))
    pbr = bridge(prox, "prox", yPB1, yPA0, ztop, ztop + Hp, prox == "tube")
    # distal arms
    def arm(y0, y1):
        return U(cyl_y(R_ARM, y0, y1), box(-R_ARM * 0.8, R_ARM * 0.8, y0, y1, -Db - Hd, 0))
    zd = -Db - Hd / 2 if dist == "flat" else -Db - 8
    aa = arm(yA_arm0, yA_arm1); ab = arm(yB_arm0, yB_arm1)
    if act["kind"] == "AK80-64":
        aa = aa.fuse(cyl_y(act["pilot_d"] / 2 - 0.01, yA_arm1, yA_arm1 + 4.0)).clean()   # Ø21 g6 spigot
        for k in range(act["out_n"]):
            a = math.radians(360 * k / act["out_n"])
            aa = aa.cut(cyl_y(2.25, yA_arm0, yA_arm1, act["out_pcd"] / 2 * math.sin(a), act["out_pcd"] / 2 * math.cos(a)))
    for x in (-20.0, 20.0):
        aa = aa.cut(cyl_y(2.75, yA_arm0, yA_arm1, x, zd)); ab = ab.cut(cyl_y(2.75, yB_arm0, yB_arm1, x, zd))
    ab = C(ab, cyl_y(B6004["d"] / 2, yB_arm0, yB_arm1))            # shaft press bore
    px, pz = pin_point(0.0)
    ab = ab.cut(cyl_y(STOP_PIN / 2, yB_arm0, yB_arm1, px, pz))     # stop pin press bore
    dbr = bridge(dist, "dist", yB_arm1, yA_arm0, -Db - Hd, -Db, dist == "tube")
    # shaft, spacer, bearing, cap, pin, circlip
    shaft = cyl_y(B6004["d"] / 2, yPB0 - 1.5, yB_arm1)
    spacer = C(cyl_y(13.0, yB_arm0 - AX_GAP, yB_arm0), cyl_y(B6004["d"] / 2, yB_arm0 - AX_GAP, yB_arm0))
    brg = C(cyl_y(B6004["D"] / 2, yPB0, yPB1), cyl_y(B6004["d"] / 2, yPB0, yPB1))
    cap = C(cyl_y(28.0, yPB0 - 3, yPB0), cyl_y(16.0, yPB0 - 3, yPB0))
    for k in range(3):
        a = math.radians(90 + 120 * k)
        cap = cap.cut(cyl_y(2.2, yPB0 - 3, yPB0, 25 * math.sin(a), 25 * math.cos(a)))
    clip = C(cyl_y(13.5, yPB0 - 1.2 - 0.3, yPB0 - 0.3), cyl_y(B6004["d"] / 2, yPB0 - 1.5, yPB0))
    pin = cyl_y(STOP_PIN / 2, yPB1 - 10, yB_arm1 - 0.5, px, pz)
    m = name
    al = "6061-T6"
    P += [
        Part(f"{m}-PLA", pa, material=al, process="waterjet 8 mm plate + CNC bore/holes", make_buy="make", kind="structure",
             module=m, stage=stage - 1, crit=("Ø80.4 H8 bore; 8xØ3.2 on Ø85 PCD; flatness 0.05" if act["kind"] == "AK80-64" else "DRAWING REQUIRED"),
             flags=flags),
        Part(f"{m}-PLB", pb, material=al, process="waterjet 12 mm plate + CNC Ø42 H7 bore, stop slot", make_buy="make", kind="structure",
             module=m, stage=stage - 1, crit="Ø42 H7 bearing bore coaxial with actuator axis ≤0.05; slot range +-2 deg"),
        Part(f"{m}-PBR", pbr, material=al, process="CNC 3-axis block (pocketed)", make_buy="make", kind="structure", module=m, stage=stage - 1,
             crit=("Ø40 H8 socket, slit, 2xM6, Ø6 H7 cross pin" if prox == "tube" else "4xØ6.6 on 44 sq, ±0.05")),
        Part(f"{m}-ARA", aa, material=al, process="waterjet 8 mm + CNC spigot/PCD", make_buy="make", kind="structure", module=m, stage=stage,
             crit=("Ø21 g6 spigot; 6xØ4.5 on Ø28 PCD" if act["kind"] == "AK80-64" else "DRAWING REQUIRED"), flags=flags),
        Part(f"{m}-ARB", ab, material=al, process="waterjet 8 mm + CNC Ø20 N7 press bore", make_buy="make", kind="structure", module=m, stage=stage,
             crit="Ø20 N7 shaft bore coaxial ≤0.05 with arm A' spigot (line-bore in assembly)"),
        Part(f"{m}-DBR", dbr, material=al, process="CNC 3-axis block", make_buy="make", kind="structure", module=m, stage=stage,
             crit=("Ø40 H8 socket, slit, 2xM6" if dist == "tube" else "4xØ6.6 on 44 sq")),
        Part(f"{m}-SFT", shaft, material="42CrMo4", process="turned + ground Ø20 k6, circlip groove", make_buy="make", kind="structure",
             module=m, stage=stage, crit="Ø20 k6 bearing seat; groove DIN 471"),
        Part(f"{m}-SPC", spacer, material="steel", process="turned", make_buy="make", kind="structure", module=m, stage=stage, crit="1.0 ±0.02 thick"),
        Part(f"{m}-CAP", cap, material=al, process="turned", make_buy="make", kind="structure", module=m, stage=stage - 1, crit="3xØ4.4 on Ø50"),
        Part(f"{m}-PIN", pin, material="steel", process="buy ISO 8734 Ø6 dowel, cut", make_buy="buy", kind="structure", module=m, stage=stage,
             crit="Ø6 m6 hardened"),
        Part(f"{m}-BRG", brg, material="steel", process="purchased", make_buy="buy", kind="envelope-purchased", module=m, stage=stage - 1,
             mass_kg=B6004["mass_kg"], crit="SKF 6004-2RSH (or eq.) 20x42x12", status="envelope"),
        Part(f"{m}-CLP", clip, material="steel", process="purchased DIN 471 20x1.2", make_buy="buy", kind="envelope-purchased", module=m,
             stage=stage, mass_kg=0.002, status="envelope", crit="DIN 471 20x1.2 retaining ring; groove not modelled"),
        Part(f"{m}-ACT", env, material="steel", process="purchased", make_buy="buy", kind="envelope-purchased", module=m, stage=stage - 1,
             mass_kg=act["mass_kg"], crit=act["kind"] + " envelope from datasheet", status="envelope", flags=flags),
    ]
    info = {"Dp": Dp, "Db": Db, "Hp": Hp, "Hd": Hd, "up": Dp + Hp, "down": Db + Hd, "y_out": yPA1, "y_in": yPB0 - 3,
            "act": act["kind"], "range": (lo, hi)}
    return P, info

# ------------------------------------------------------------------ printed covers for the module
def rear_cap(act, y0):
    """Rounded printed cap over plate A (actuator rear + connectors); cable exit with zip-tie bridge."""
    r = 54.0
    s0 = cyl_y(r, y0, y0 + 22)
    s = s0.fillet(8, [e for e in s0.Edges() if e.Center().y > y0 + 21])
    s = C(s, cyl_y(r - 2.4, y0, y0 + 19.6))
    s = C(s, box(-12, 12, y0, y0 + 14, r - 6, r + 1))                    # cable exit notch (top)
    s = s.fuse(box(-10, 10, y0 + 11, y0 + 14, r - 5, r - 1.8)).clean()        # strain-relief bridge for zip tie
    for k in range(3):
        a = math.radians(60 + 120 * k)
        s = s.fuse(C(cyl_y(4.0, y0, y0 + 19.6, 47 * math.sin(a), 47 * math.cos(a)),
                     cyl_y(1.7, y0, y0 + 19.6, 47 * math.sin(a), 47 * math.cos(a)))).clean()   # M3 boss
    return s

def bearing_cap_cover(y1):
    r = 50.0
    s0 = cyl_y(r, y1 - 12, y1)
    s = s0.fillet(6, [e for e in s0.Edges() if e.Center().y < y1 - 11])
    s = C(s, cyl_y(r - 2.4, y1 - 12 + 2.4, y1)).clean()
    return s

# ------------------------------------------------------------------ leg assembly
def Rz90():
    return np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], float).T  # columns: local x->-y? see below

def T_of(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T

R_Y = np.eye(3)                                   # local y -> world +y (pitch modules)
R_X = np.column_stack([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).astype(float)  # local x->-y, y->+x, z->z (roll modules)

JOINTS = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"]

def leg(side="l"):
    from ziggy.params import joint_table
    jt = joint_table()
    rng = lambda j: jt[f"{side}_{j}"][1:3] if side == "l" else jt[f"l_{j}"][1:3]   # right = mirror of left geometry
    parts, joints = [], {}
    SOLE_T, FOOT_T = 6.0, 10.0
    # ankle roll (axis x), distal flat -> foot plate
    ar, iar = module("AR", AK80, "flat", "flat", *rng("ankle_roll"), 6)
    z_ar = SOLE_T + FOOT_T + iar["down"]
    ap, iap = module("AP", AK80, "tube", "flat", *rng("ankle_pitch"), 5)
    z_ap = z_ar + iar["up"] + iap["down"]
    kn, ikn = module("KN", RS04, "tube", "tube", *rng("knee"), 4)
    hp, ihp = module("HP", AK80, "flat", "tube", *rng("hip_pitch"), 3)
    hr, ihr = module("HR", AK80, "flat", "flat", *rng("hip_roll"), 2)
    SHIN_EXPOSED, THIGH_EXPOSED = 120.0, 138.0   # thigh chosen so hip-pitch axis = 870 mm (sim v0.1 value)
    z_kn = z_ap + iap["up"] + SHIN_EXPOSED + ikn["down"]
    z_hp = z_kn + ikn["up"] + THIGH_EXPOSED + ihp["down"]
    z_hr = z_hp + ihp["up"] + ihr["down"]
    z_hr_top = z_hr + ihr["up"]
    y = HIP_Y
    placements = [(ar, R_X, (0, y, z_ar), "ankle_roll"), (ap, R_Y, (0, y, z_ap), "ankle_pitch"),
                  (kn, R_Y, (0, y, z_kn), "knee"), (hp, R_Y, (0, y, z_hp), "hip_pitch"), (hr, R_X, (0, y, z_hr), "hip_roll")]
    for plist, R, t, j in placements:
        for p in plist:
            p.T = T_of(R, t); parts.append(p)
        joints[j] = {"origin": list(t), "axis": list(R @ np.array([0, 1, 0.0])), "range_deg": rng(j)}
    # tubes (6061-T6 40x3), inserted 30 mm... full socket depth
    def tube(pid, z0, z1, stage):
        s = C(cyl_z(TUBE_OD / 2, z0, z1), cyl_z(TUBE_OD / 2 - TUBE_T, z0, z1),
              cyl_y(3.0, -30, 30, 0, z0 + H_TUBE / 2), cyl_y(3.0, -30, 30, 0, z1 - H_TUBE / 2))
        return Part(pid, s, material="6061-T6", process="saw-cut tube + drill Ø6 H7 cross holes (jig)", make_buy="make",
                    kind="structure", module="TUBE", stage=stage, crit=f"length {z1 - z0:.1f} ±0.2; cross holes ⟂ ±0.1")
    shin0 = z_ap + iap["Dp"]; shin1 = z_kn - ikn["Db"]
    th0 = z_kn + ikn["Dp"]; th1 = z_hp - ihp["Db"]
    for pid, z0, z1, st in (("SHIN-TUBE", shin0, shin1, 4), ("THIGH-TUBE", th0, th1, 3)):
        p = tube(pid, z0, z1, st); p.T = T_of(np.eye(3), (0, y, 0)); parts.append(p)
    # foot plate (waterjet 10 mm), 4xM6 to ankle-roll distal bridge, sole pad screws
    fx = 30.0
    fp = box(-120 + fx, 120 + fx, -55, 55, SOLE_T, SOLE_T + FOOT_T)
    fp = fp.edges("|Z").fillet(15) if False else fp
    holes = [cyl_z(3.3, SOLE_T, SOLE_T + FOOT_T, x, yy) for x in (-22, 22) for yy in (-22, 22)]
    holes += [cyl_z(1.7, SOLE_T, SOLE_T + FOOT_T, x, yy) for x in (-50 + fx, 50 + fx) for yy in (-40, 40)]
    holes += [box(-100 + fx, -60 + fx, -35, 35, SOLE_T, SOLE_T + FOOT_T), box(60 + fx, 100 + fx, -35, 35, SOLE_T, SOLE_T + FOOT_T)]  # lightening
    fpl = Part("FOOT-PLATE", C(fp, *holes), material="6061-T6", process="waterjet 10 mm plate", make_buy="make", kind="structure",
               module="FOOT", stage=6, crit="4xØ6.6 on 44 sq; flatness 0.2")
    fpl.T = T_of(np.eye(3), (0, y, 0)); parts.append(fpl)
    # hip yaw: actuator axis z, output face down onto yaw output adapter which bolts to HR proximal flat bridge
    yaw_out = box(-WX / 2, WX / 2, -WX / 2, WX / 2, 0, 10)
    yaw_out = U(yaw_out, cyl_z(AK80["pilot_d"] / 2 - 0.01, 10, 14))
    yh = [cyl_z(3.3, 0, 10, x, yy) for x in (-22, 22) for yy in (-22, 22)]
    yh += [cyl_z(2.25, 0, 10, 14 * math.sin(math.radians(60 * k)), 14 * math.cos(math.radians(60 * k))) for k in range(6)]
    yaw_out = C(yaw_out, *yh, cyl_z(5.0, 0, 14))
    pyo = Part("HY-OUT", yaw_out, material="6061-T6", process="CNC plate 10 mm + Ø21 g6 spigot", make_buy="make", kind="structure",
               module="HY", stage=1, crit="Ø21 g6 spigot; 6xØ4.5 on Ø28; 4xØ6.6 on 44 sq",
               flags=["BLOCKER: hip yaw relies on actuator output bearing for leg moments; AK80-64 moment rating not published",
                      "no mechanical yaw hard stop yet (software limit only)"])
    pyo.T = T_of(np.eye(3), (0, y, z_hr_top)); parts.append(pyo)
    yenv, _, _ = actuator_envelope(AK80, 0.0)    # along +y from face; rotate so +y -> +z
    R_up = np.column_stack([[1, 0, 0], [0, 0, 1], [0, -1, 0]]).astype(float)  # local y -> world +z (fix F-010: was -z)
    pya = Part("HY-ACT", yenv, material="steel", process="purchased", make_buy="buy", kind="envelope-purchased", module="HY", stage=0,
               mass_kg=AK80["mass_kg"], status="envelope", flags=AK80["flags"], crit="AK80-64 envelope")
    pya.T = T_of(R_up, (0, y, z_hr_top + 10)); parts.append(pya)
    joints["hip_yaw"] = {"origin": [0, y, z_hr_top + 10], "axis": [0, 0, 1], "range_deg": rng("hip_yaw")}
    levels = {"sole": 0, "ankle_roll": z_ar, "ankle_pitch": z_ap, "knee": z_kn, "hip_pitch": z_hp, "hip_roll": z_hr,
              "yaw_face": z_hr_top + 10, "pelvis_plate_bottom": z_hr_top + 10 + AK80["boss_l"] + AK80["body_l"]}
    return parts, joints, levels

def pelvis_plate(levels):
    z0 = levels["pelvis_plate_bottom"]; t = 8.0
    p = box(-90, 90, -170, 170, z0, z0 + t)
    holes = []
    for sy in (1, -1):
        yy = sy * HIP_Y
        holes.append(cyl_z(AK80["rear_d"] / 2 + 0.2, z0, z0 + t, 0, yy))
        for k in range(AK80["rear_n"]):
            a = math.radians(AK80["rear_phase_deg"] + 45 * k)
            holes.append(cyl_z(1.6, z0, z0 + t, AK80["rear_pcd"] / 2 * math.sin(a), yy + AK80["rear_pcd"] / 2 * math.cos(a)))
    for x in (-75, 75):
        for yy in (-150, -30, 30, 150):
            holes.append(cyl_z(3.3, z0, z0 + t, x, yy))        # M6 to torso frame (frame not designed)
    holes += [box(-50, 50, -35, 35, z0, z0 + t)]                 # harness pass-through / lightening
    return Part("PELVIS-PLATE", C(p, *holes), material="6061-T6", process="waterjet 8 mm + CNC Ø80.4 bores", make_buy="make",
                kind="structure", module="PELVIS", stage=0, crit="2x Ø80.4 H8 at ±110 ±0.05; 16x Ø3.2 on Ø85 PCD",
                flags=AK80["flags"] + ["torso frame interface holes provisional (frame not designed)"])

def mirror_part(p):
    q = Part(p.pid.replace("-L", "") + "-R" if p.pid.endswith("-L") else p.pid + "-R", p.shape.mirror("XZ"), material=p.material,
             process=p.process, make_buy=p.make_buy, kind=p.kind, module=p.module, stage=p.stage, crit=p.crit, status=p.status,
             flags=p.flags, mass_kg=p.mass_override, printed=p.printed, orient=p.orient)
    M = np.diag([1, -1, 1.0]); q.T = T_of(M @ p.T[:3, :3] @ M, M @ p.T[:3, 3]); return q

def build():
    """Full lower body: both legs + pelvis plate + printed covers. Returns parts, joints, levels."""
    L, jl, lv = leg("l")
    for p in L: p.pid += "-L"
    covers = []
    # printed covers (module rear caps + bearing covers + tube covers + foot shell)
    from cad.parts import limb_cover
    for p in list(L):
        if p.pid.endswith("-PLA-L"):
            m = p.module
            if m in ("HP", "AP", "HR", "AR"):
                y_out = p.shape.BoundingBox().ymax
                c = Part(f"{m}-RCAP-L", rear_cap(AK80, y_out), material="PETG", process="FDM print", make_buy="make", kind="printed",
                         module=m, stage=p.stage, printed=True, orient="dome up", crit="M3 bosses on Ø100; cable notch + zip-tie bridge")
                c.T = p.T.copy(); covers.append(c)
        if p.pid.endswith("-PLB-L"):
            y_in = p.shape.BoundingBox().ymin
            c = Part(f"{p.module}-BCOV-L", bearing_cap_cover(y_in - 3.2), material="PETG", process="FDM print", make_buy="make",
                     kind="printed", module=p.module, stage=p.stage, printed=True, orient="open side down", crit="snap/glue to cap; TBD")
            c.T = p.T.copy(); covers.append(c)
    L += covers
    R = [mirror_part(p) for p in L]
    for p in R: p.pid = p.pid[:-2] + "-R" if p.pid.endswith("-L-R") else p.pid
    parts = L + R + [pelvis_plate(lv)]
    jr = {k: {**v, "origin": [v["origin"][0], -v["origin"][1], v["origin"][2]], "axis": [v["axis"][0], -v["axis"][1], v["axis"][2]]}
          for k, v in jl.items()}
    return parts, {"l": jl, "r": jr}, lv

def world(p):
    return _place(p.shape, p.T)
