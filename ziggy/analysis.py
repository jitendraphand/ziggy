# SPDX-License-Identifier: Apache-2.0
"""Quantitative checks: IK, static support margin, PD stance sim, static joint torque."""
import numpy as np
import mujoco
from .params import ACTUATORS, joint_table

JT = joint_table()
MODEL = "sim/ziggy.xml"

def load(path=MODEL):
    m = mujoco.MjModel.from_xml_path(path)
    return m, mujoco.MjData(m)

# ---------------- support polygon -----------------
def _hull(pts):
    pts = sorted(map(tuple, pts))
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return np.array(lo[:-1] + up[:-1])

def support_margin(m, d, feet=("l_sole", "r_sole")):
    """Signed distance (m) of CoM ground projection to support hull edge (+ inside)."""
    pts = []
    for g in feet:
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, g)
        R = d.geom_xmat[gid].reshape(3, 3); c = d.geom_xpos[gid]; h = m.geom_size[gid]
        for sx in (-1, 1):
            for sy in (-1, 1):
                pts.append((c + R @ np.array([sx*h[0], sy*h[1], -h[2]]))[:2])
    H = _hull(pts)
    com = d.subtree_com[1][:2]
    dmin = np.inf
    for i in range(len(H)):
        a, b = H[i], H[(i+1) % len(H)]
        e = b - a; n = np.array([e[1], -e[0]]) / np.linalg.norm(e)
        dist = -np.dot(com - a, n)  # CCW hull -> inward is left of edge
        dmin = min(dmin, dist)
    return float(dmin)

# ---------------- PD stance simulation -----------------
def stance_sim(seconds=5.0, push_n=0.0, push_t=(2.0, 2.1)):
    m, d = load()
    mujoco.mj_forward(m, d)
    z0 = d.qpos[2]; minz = z0; mintilt = 1.0; margins = []
    while d.time < seconds:
        d.ctrl[:] = 0.0
        d.xfrc_applied[1, 0] = push_n if push_t[0] <= d.time < push_t[1] else 0.0
        mujoco.mj_step(m, d)
        minz = min(minz, d.qpos[2])
        mintilt = min(mintilt, 1 - 2*(d.qpos[4]**2 + d.qpos[5]**2))  # cos(tilt) of z axis
        margins.append(support_margin(m, d))
    tau = {m.actuator(i).name[4:]: float(d.actuator_force[i]) for i in range(m.nu)}
    return {"pelvis_drop_m": float(z0 - minz), "min_cos_tilt": float(mintilt),
            "min_support_margin_m": float(min(margins)), "final_margin_m": float(margins[-1]),
            "steady_torques_nm": tau}

# ---------------- IK (damped least squares) -----------------
def arm_dofs(m, side, with_waist=False):
    names = [n for n in JT if n.startswith(f"{side}_shoulder") or n.startswith(f"{side}_elbow")
             or n.startswith(f"{side}_wrist")]
    if with_waist: names = ["waist_yaw"] + names
    jids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    return names, [m.jnt_qposadr[j] for j in jids], [m.jnt_dofadr[j] for j in jids], jids

def solve_ik(m, d, site, target, side="l", with_waist=False, iters=200, lam=0.03, tol=1e-3, q0=None):
    names, qa, da, jids = arm_dofs(m, side, with_waist)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site)
    if q0 is not None: d.qpos[:] = q0
    lo = m.jnt_range[jids, 0]; hi = m.jnt_range[jids, 1]
    jacp = np.zeros((3, m.nv))
    err = np.inf
    for k in range(iters):
        mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
        e = np.asarray(target) - d.site_xpos[sid]; err = np.linalg.norm(e)
        if err < tol: break
        mujoco.mj_jacSite(m, d, jacp, None, sid)
        J = jacp[:, da]
        dq = J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(3), e)
        dq = np.clip(dq, -0.2, 0.2)
        d.qpos[qa] = np.clip(d.qpos[qa] + dq, lo, hi)
    mujoco.mj_kinematics(m, d)
    err = float(np.linalg.norm(np.asarray(target) - d.site_xpos[sid]))
    within = bool(np.all(d.qpos[qa] >= lo - 1e-9) and np.all(d.qpos[qa] <= hi + 1e-9))
    return {"error_m": err, "iters": k + 1, "within_limits": within, "q": d.qpos[qa].copy()}

def ik_benchmark(n=200, seed=0, side="l", tol=1e-3):
    """Targets from FK of random in-limit configs => guaranteed reachable."""
    m, d = load(); rng = np.random.default_rng(seed)
    names, qa, da, jids = arm_dofs(m, side)
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{side}_grasp")
    home = m.qpos0.copy(); errs, ok, lim = [], 0, 0
    for _ in range(n):
        d.qpos[:] = home
        d.qpos[qa] = rng.uniform(m.jnt_range[jids, 0], m.jnt_range[jids, 1])
        mujoco.mj_kinematics(m, d); tgt = d.site_xpos[sid].copy()
        q0 = home.copy(); q0[qa] = np.array([-0.3, 0.2 if side == "l" else -0.2, 0, -0.6, 0, 0, 0])
        best = None
        for restart in range(4):  # bounded restarts
            r = solve_ik(m, d, f"{side}_grasp", tgt, side, q0=q0, tol=tol)
            if best is None or r["error_m"] < best["error_m"]: best = r
            if r["error_m"] < tol: break
            q0 = home.copy(); q0[qa] = rng.uniform(m.jnt_range[jids, 0], m.jnt_range[jids, 1])
        errs.append(best["error_m"]); ok += best["error_m"] < tol; lim += best["within_limits"]
    e = np.array(errs)
    return {"n": n, "success_rate": ok / n, "median_err_mm": float(np.median(e)*1e3),
            "p95_err_mm": float(np.percentile(e, 95)*1e3), "max_err_mm": float(e.max()*1e3),
            "limits_respected_rate": lim / n}

HOUSEHOLD_TARGETS = {  # world frame, robot standing at origin facing +x (m)
    "kitchen_counter_0.90m": (0.35, 0.20, 0.90),
    "dining_table_0.75m": (0.35, 0.25, 0.75),
    "upper_shelf_1.40m": (0.30, 0.20, 1.40),
    "door_handle_1.00m": (0.40, 0.15, 1.00),
    "light_switch_1.20m": (0.35, 0.30, 1.20),
    "floor_pickup_0.10m": (0.30, 0.20, 0.10),
}

def household_reach(side="l"):
    m, d = load(); out = {}
    for k, t in HOUSEHOLD_TARGETS.items():
        best = None
        for ww in (False, True):
            q0 = m.qpos0.copy()
            r = solve_ik(m, d, f"{side}_grasp", t, side, with_waist=ww, q0=q0, iters=400)
            if best is None or r["error_m"] < best["error_m"]: best = r
        out[k] = round(best["error_m"] * 1e3, 2)
    return out  # mm residual; >5 mm = not reachable standing (needs squat/step)

# ---------------- static torque vs rating -----------------
def static_torque_cases():
    """Jacobian-transpose static torques, legs share weight equally; arm carries payload."""
    m, d = load(); g = 9.81; total = float(sum(m.body_mass))
    def setq(dct):
        d.qpos[:] = m.qpos0
        for n, v in dct.items():
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = np.radians(v)
    cases = {
        "stand": {},
        "squat_knee90": {f"{s}_{j}": v for s in "lr" for j, v in (("hip_pitch", -45), ("knee", 90), ("ankle_pitch", -45))},
        "arm_2kg_forward_reach": {"l_shoulder_pitch": -90},
    }
    res = {}
    for cname, q in cases.items():
        setq(q); d.qvel[:] = 0; mujoco.mj_forward(m, d)
        tau = d.qfrc_bias.copy()
        for s in "lr":
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{s}_sole_site")
            jp = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, jp, None, sid)
            tau -= jp.T @ np.array([0, 0, total*g/2])
        if cname == "arm_2kg_forward_reach":
            sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "l_grasp")
            jp = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, jp, None, sid)
            tau -= jp.T @ np.array([0, 0, -2.0*g])
        rows = {}
        for n, (_, _, _, cls) in JT.items():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            t = abs(float(tau[m.jnt_dofadr[j]])); a = ACTUATORS[cls]
            rows[n] = {"tau_nm": round(t, 2), "class": cls, "rated_nm": a["rated_nm"],
                       "ratio_rated": round(t / a["rated_nm"], 3), "rating_verified": a["verified"]}
        res[cname] = rows
    return res
