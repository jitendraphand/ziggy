# SPDX-License-Identifier: CERN-OHL-S-2.0
"""ZIGGY v0.1 parametric printed parts (CadQuery). Run: python -m cad.build

Every part is exported in its intended PRINT ORIENTATION (Z = build direction).
load_class meanings (see docs/MECHANICAL.md):
  cover      - cosmetic / protective shell, carries no structural load
  guide      - cable guide, clip, locator; incidental load only
  functional - light-duty functional part (hand, sole pad); prototype, test before trust
  coupon     - calibration / tolerance test print
Primary load paths (actuator-to-actuator) are METAL (aluminium tube + CNC plate) and are
NOT printed. Nothing here is rated for primary structural load.
"""
import cadquery as cq

TUBE_OD = 40.0          # 6061-T6 tube 40 x 2 mm, structural (not printed)
CLR = 0.4               # nominal radial clearance printed-over-metal (tune with coupon)
M3_CLEAR = 3.4          # M3 clearance hole
M3_INSERT = 4.0         # M3 heat-set insert bore (verify against insert datasheet)
WALL = 2.4
AK80_D = 98.0           # CubeMars AK80-64 body diameter (manufacturer)

def _half_shell(length, inner_r, wall, flange_w=8.0, n_holes=4):
    """Half of a split cylindrical cover, printed lying flat on its flange (open side down)."""
    ro = inner_r + wall
    body = (cq.Workplane("XY").circle(ro).circle(inner_r).extrude(length)
            .cut(cq.Workplane("XY").center(0, -ro).rect(4*ro, 2*ro).extrude(length)))
    fl = (cq.Workplane("XY").pushPoints([(ro + flange_w/2 - wall/2, 0.0), (-(ro + flange_w/2 - wall/2), 0.0)])
          .rect(flange_w + wall, 3.0, centered=(True, False)).extrude(length))
    shell = body.union(fl)
    zs = [length*(i+0.5)/n_holes for i in range(n_holes)]
    for sx in (1, -1):
        x = sx*(ro + flange_w/2)
        for z in zs:
            shell = shell.cut(cq.Workplane("XZ").center(x, z).circle(M3_CLEAR/2).extrude(-10))
    # print orientation: axis along X, flange on bed (Z up = dome)
    return shell.rotate((0, 0, 0), (0, 1, 0), 90).rotate((0, 0, 0), (1, 0, 0), -90)

def limb_cover(length):
    return _half_shell(length, TUBE_OD/2 + CLR + 6.0, WALL)  # 6 mm cable channel annulus

def actuator_cover_ak80():
    ri = AK80_D/2 + 1.0
    c = cq.Workplane("XY").circle(ri + WALL).extrude(30).cut(cq.Workplane("XY").circle(ri).extrude(30)) \
        .union(cq.Workplane("XY").circle(ri + WALL).circle(30).extrude(2.0))
    vent = cq.Workplane("XZ").pushPoints([(0, 15)]).rect(20, 8).extrude(-(ri + 5), both=True)
    return c.cut(vent)

def cable_clip_tube40():
    r = TUBE_OD/2
    clip = (cq.Workplane("XY").circle(r + 3).circle(r + 0.2).extrude(12)
            .cut(cq.Workplane("XY").center(0, r).rect(28, 12).extrude(12)))
    boss = cq.Workplane("XY").center(0, -(r + 6)).rect(16, 8).extrude(12) \
        .cut(cq.Workplane("XY").center(0, -(r + 6)).rect(10, 4).extrude(12))  # zip-tie slot
    return clip.union(boss)

def foot_sole_pad():  # print in TPU 95A
    p = cq.Workplane("XY").rect(240, 110).extrude(6).edges("|Z").fillet(15)
    for x in range(-100, 101, 20):
        p = p.cut(cq.Workplane("XY").center(x, 0).rect(3, 100).extrude(2))
    return p.faces(">Z").workplane().pushPoints([(-80, 35), (-80, -35), (80, 35), (80, -35)]).hole(M3_CLEAR)

def foot_cover():
    o = cq.Workplane("XY").rect(240, 110).extrude(45).edges("|Z").fillet(15)
    i = cq.Workplane("XY").rect(240 - 2*WALL, 110 - 2*WALL).extrude(45 - WALL).edges("|Z").fillet(13)
    return o.cut(i).cut(cq.Workplane("XY").center(-30, 0).circle(35).extrude(50))  # ankle opening

def shell_box(w, d, h, name_opening=None):
    o = cq.Workplane("XY").rect(w, d).extrude(h).edges("|Z").fillet(20)
    i = cq.Workplane("XY").rect(w - 2*WALL, d - 2*WALL).extrude(h - WALL).edges("|Z").fillet(18)
    return o.cut(i)

def torso_shell_front():
    """340 wide x 440 tall x 110 deep, open toward the robot interior. Printed standing on its
    340x110 edge so the open face is vertical (fits 470 mm Z). Fix F-005: v0.1.0 was open at its end."""
    return shell_box(340, 440, 110).rotate((0, 0, 0), (1, 0, 0), 90)
def torso_shell_back():  return torso_shell_front()
def pelvis_cover():      return shell_box(280, 160, 120)

def head_shell_half():
    """Hemispherical shell from a revolved annular profile (watertight; QA finding F-004)."""
    import math
    R, t, rv = 110.0, WALL, 12.0  # rv: top vent / sensor cable hole radius (keeps profile off the axis)
    zo, zi = math.sqrt(R**2 - rv**2), math.sqrt((R - t)**2 - rv**2)
    prof = (cq.Workplane("XZ").moveTo(R - t, 0).lineTo(R, 0)
            .threePointArc((R*0.7071, R*0.7071), (rv, zo)).lineTo(rv, zi)
            .threePointArc(((R - t)*0.7071, (R - t)*0.7071), (R - t, 0)).close())
    return prof.revolve(360, (0, 0, 0), (0, 1, 0))

def estop_mount():  # for 22 mm panel-mount E-stop pushbutton
    b = cq.Workplane("XY").rect(60, 60).extrude(4).edges("|Z").fillet(6)
    b = b.faces(">Z").workplane().hole(22.5)
    return b.faces(">Z").workplane().rect(46, 46, forConstruction=True).vertices().hole(M3_CLEAR)

# ---------------- dexterous hand (tendon-driven, functional prototype) ----------
PIN = 3.0 + 0.1   # 3 mm hardened steel dowel pin, printed bore +0.1 (tune with coupon)
TENDON = 1.6      # PTFE-lined channel for 0.8 mm UHMWPE/Dyneema tendon

def phalanx(length, w=16.0, h=14.0, distal=False):
    """Phalanx printed on its side (pin axes vertical => round, strong bores)."""
    p = cq.Workplane("XY").rect(length, h).extrude(w).edges("|Z").fillet(3)
    p = p.cut(cq.Workplane("XY").center(length/2 - 4, 0).rect(8.5, h + 2).extrude(w).translate((0, 0, 0))
              .intersect(cq.Workplane("XY").rect(4*length, 4*h).extrude(w).translate((0, 0, 0)))
              .cut(cq.Workplane("XY").rect(4*length, 4*h).extrude(3.5))
              .cut(cq.Workplane("XY").rect(4*length, 4*h).extrude(3.5).translate((0, 0, w - 3.5))))  # clevis
    p = p.cut(cq.Workplane("XY").center(length/2 - 4, 0).circle(PIN/2).extrude(w))  # distal joint bore
    p = p.cut(cq.Workplane("XY").center(-length/2 + 4, 0).circle(PIN/2).extrude(w))  # proximal bore
    p = p.cut(cq.Workplane("YZ").center(-h/2 + 3.2, w/2).circle(TENDON/2).extrude(length, both=True))  # flexor
    p = p.cut(cq.Workplane("YZ").center(h/2 - 3.2, w/2).circle(TENDON/2).extrude(length, both=True))   # extensor
    if distal:
        p = p.cut(cq.Workplane("XY").center(length/2 - 4, 0).rect(9, h + 2).extrude(w)).union(
            cq.Workplane("XY").center(length/2 - 6, 0).rect(5, h).extrude(w).edges("|Z").fillet(2))
    return p

def palm_base():
    p = cq.Workplane("XY").rect(95, 90).extrude(32).edges("|Z").fillet(6)
    p = p.cut(cq.Workplane("XY").rect(95 - 2*3, 90 - 2*3).extrude(29).translate((0, 0, 3)))
    xs = [-31.5, -10.5, 10.5, 31.5]
    for x in xs:  # MCP clevis bores along Y edge
        p = p.cut(cq.Workplane("XZ").center(x, 16).circle(PIN/2).extrude(-50, both=True))
        p = p.cut(cq.Workplane("XY").center(x, 45 - 4).rect(8.5, 9).extrude(32).translate((0, 0, 0)))
    p = p.faces("<Y").workplane(centerOption="CenterOfBoundBox").rect(40, 20, forConstruction=True).vertices().hole(M3_INSERT, 10)
    return p.union(cq.Workplane("XY").center(-40, -30).circle(8).extrude(32).cut(
        cq.Workplane("XY").center(-40, -30).circle(PIN/2).extrude(32)))  # thumb CMC boss

def tendon_spool():
    s = cq.Workplane("XY").circle(12).extrude(2).faces(">Z").workplane().circle(8).extrude(8) \
        .faces(">Z").workplane().circle(12).extrude(2)
    s = s.cut(cq.Workplane("XY").circle(2.6).extrude(12))  # servo horn hub (adapt to selected servo)
    return s.cut(cq.Workplane("XZ").center(0, 6).circle(0.6).extrude(-15, both=True))

# ---------------- tolerance coupons ----------------
def coupon_holes():
    c = cq.Workplane("XY").rect(110, 22).extrude(5)
    for i, dia in enumerate([2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 4.0, 4.2]):
        c = c.cut(cq.Workplane("XY").center(-48 + i*12, 0).circle(dia/2).extrude(5))
    return c

def coupon_tube_fit():
    c = cq.Workplane("XY")
    for i, clr in enumerate([0.2, 0.3, 0.4, 0.5, 0.6]):
        r = TUBE_OD/2 + clr
        c = c.union(cq.Workplane("XY").center(i*52, 0).circle(r + 2).circle(r).extrude(10))
    return c

PARTS = {
    # name: (builder, material, qty_per_robot, load_class, orientation_note)
    "thigh_cover_half":   (lambda: limb_cover(340), "PETG", 4, "cover", "flange on bed, dome up"),
    "shin_cover_half":    (lambda: limb_cover(330), "PETG", 4, "cover", "flange on bed, dome up"),
    "upper_arm_cover_half": (lambda: limb_cover(220), "PETG", 4, "cover", "flange on bed"),
    "forearm_cover_half": (lambda: limb_cover(200), "PETG", 4, "cover", "flange on bed"),
    "actuator_cover_ak80": (actuator_cover_ak80, "PETG", 17, "cover", "lip on bed"),
    "cable_clip_tube40":  (cable_clip_tube40, "PETG", 24, "guide", "flat on bed"),
    "foot_sole_pad":      (foot_sole_pad, "TPU-95A", 2, "functional", "tread down"),
    "foot_cover":         (foot_cover, "PETG", 2, "cover", "open side down"),
    "torso_shell_front":  (torso_shell_front, "PETG", 1, "cover", "standing on 340x110 edge, open face vertical, brim"),
    "torso_shell_back":   (torso_shell_back, "PETG", 1, "cover", "standing on 340x110 edge, open face vertical, brim"),
    "pelvis_cover":       (pelvis_cover, "PETG", 1, "cover", "open side down"),
    "head_shell_half":    (head_shell_half, "PETG", 2, "cover", "rim on bed, tree supports"),
    "estop_mount":        (estop_mount, "PETG", 1, "guide", "flat"),
    "palm_base":          (palm_base, "PETG", 2, "functional", "back down"),
    "phalanx_proximal":   (lambda: phalanx(45), "PETG", 10, "functional", "on side, pin bores vertical"),
    "phalanx_middle":     (lambda: phalanx(30), "PETG", 8, "functional", "on side"),
    "phalanx_distal":     (lambda: phalanx(25, distal=True), "PETG", 10, "functional", "on side"),
    "tendon_spool":       (tendon_spool, "PETG", 12, "functional", "flat"),
    "coupon_holes":       (coupon_holes, "PETG", 1, "coupon", "flat"),
    "coupon_tube_fit":    (coupon_tube_fit, "PETG", 1, "coupon", "flat"),
}
