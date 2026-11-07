# SPDX-License-Identifier: Apache-2.0
"""Single source of truth for ZIGGY v0.1 geometry, mass budget and actuator classes.

All values are PROVISIONAL design assumptions (see docs/ASSUMPTIONS.md) except
actuator ratings, which are copied from manufacturer pages cited in docs/BOM.md.
"""
from math import radians as r

PRINTER = {  # ELEGOO Neptune 4 Max, official spec 420 x 420 x 480 mm (docs/REFERENCES.md)
    "name": "ELEGOO Neptune 4 Max",
    "build_mm": (420.0, 420.0, 480.0),
    "clearance_mm": 10.0,  # per side margin kept free (skirt/brim, bed edge, homing)
}

def print_envelope_mm():
    c = PRINTER["clearance_mm"]
    x, y, z = PRINTER["build_mm"]
    return (x - 2 * c, y - 2 * c, z - c)  # 400 x 400 x 470

# ---- Body dimensions (m) -- target stature ~1.65 m, mass ~45 kg (provisional)
DIM = {
    "hip_half_width": 0.10, "pelvis_to_hip": 0.05,
    "thigh": 0.40, "shin": 0.40, "ankle_height": 0.07,
    "foot_len": 0.24, "foot_wid": 0.11, "foot_fwd": 0.03,
    "waist": 0.08, "torso": 0.45, "shoulder_half_width": 0.20,
    "upper_arm": 0.28, "forearm": 0.26, "neck": 0.10,
    "palm_len": 0.09, "phalanx": (0.045, 0.030, 0.025), "finger_pitch": 0.021,
}

# ---- Mass budget (kg) incl. actuators, metal structure, covers, wiring
MASS = {
    "pelvis": 5.0, "hip_yaw": 0.9, "hip_roll": 0.9, "thigh": 3.0, "shin": 2.2, "foot": 1.2,
    "torso": 12.0, "upper_arm_a": 0.5, "upper_arm_b": 0.5, "upper_arm": 1.8, "forearm": 1.2,
    "wrist_a": 0.15, "wrist_b": 0.15, "hand_palm": 0.40, "phalanx": 0.02, "neck": 0.3, "head": 1.5,
}

# ---- Actuator classes. Ratings from manufacturer pages (docs/BOM.md). "assumed" = NOT a spec.
ACTUATORS = {
    "AK80-64": {"rated_nm": 48.0, "peak_nm": 120.0, "mass_kg": 0.85, "source": "cubemars.com AK80-64 KV80", "verified": True},
    "RS04":    {"rated_nm": 40.0, "peak_nm": 120.0, "mass_kg": 1.42, "source": "RobStride 04 user manual", "verified": True},
    "AK70-10": {"rated_nm": 8.3,  "peak_nm": 24.8,  "mass_kg": 0.62, "source": "cubemars.com AK series table", "verified": True},
    "WRIST_TBD": {"rated_nm": 3.0, "peak_nm": 6.0, "mass_kg": 0.15, "source": "ASSUMPTION - no part selected", "verified": False},
    "FINGER_SERVO_TBD": {"rated_nm": 0.5, "peak_nm": 1.0, "mass_kg": 0.06, "source": "ASSUMPTION - tendon servo not selected", "verified": False},
}

# ---- Joint table: name -> (axis, lower_deg, upper_deg, actuator class)
def _leg(s):
    return {
        f"{s}_hip_yaw": ("0 0 1", -30, 30, "AK80-64"),
        f"{s}_hip_roll": ("1 0 0", -25 if s == "l" else -45, 45 if s == "l" else 25, "AK80-64"),
        f"{s}_hip_pitch": ("0 1 0", -110, 30, "AK80-64"),
        f"{s}_knee": ("0 1 0", 0, 130, "RS04"),
        f"{s}_ankle_pitch": ("0 1 0", -40, 40, "AK80-64"),
        f"{s}_ankle_roll": ("1 0 0", -20, 20, "AK80-64"),
    }

def _arm(s):
    sgn = 1 if s == "l" else -1
    lo, hi = (-10, 160) if s == "l" else (-160, 10)
    return {
        f"{s}_shoulder_pitch": ("0 1 0", -170, 60, "AK80-64"),
        f"{s}_shoulder_roll": ("1 0 0", lo, hi, "AK80-64"),
        f"{s}_shoulder_yaw": ("0 0 1", -90, 90, "AK70-10"),
        f"{s}_elbow": ("0 1 0", -140, 0, "AK80-64"),  # upgraded v0.1: QA finding F-003
        f"{s}_wrist_yaw": ("0 0 1", -90, 90, "WRIST_TBD"),
        f"{s}_wrist_pitch": ("0 1 0", -60, 60, "WRIST_TBD"),
        f"{s}_wrist_roll": ("1 0 0", -45, 45, "WRIST_TBD"),
    }

FINGERS = ["thumb", "index", "middle", "ring", "little"]

def joint_table():
    t = {}
    for s in ("l", "r"):
        t.update(_leg(s))
    t["waist_yaw"] = ("0 0 1", -60, 60, "AK80-64")
    t["neck_yaw"] = ("0 0 1", -80, 80, "AK70-10")
    t["neck_pitch"] = ("0 1 0", -30, 40, "AK70-10")
    for s in ("l", "r"):
        t.update(_arm(s))
    return t

def deg2rad_range(lo, hi):
    return r(lo), r(hi)
