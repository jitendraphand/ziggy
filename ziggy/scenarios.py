# SPDX-License-Identifier: Apache-2.0
"""Deterministic MuJoCo scenarios: standing, push response (sweep), IK tracking.

Physics: the robot is a floating base (free joint) under gravity -9.81 m/s^2 with box
foot contacts. Joints are driven by MuJoCo <position> actuators (ideal PD servo, gear=1,
so actuator force == joint torque in N*m), clamped at the manufacturer PEAK torque.
Motor dynamics, current limits, thermal limits, backlash, latency and sensor noise are
NOT modelled -> results are optimistic upper bounds on achievable performance.

Controllers:
  "pd"    - every joint holds the zero posture (no balance feedback).
  "ankle" - pd + ankle/hip strategy: sagittal/lateral CoM error and velocity feedback
            added to ankle pitch/roll setpoints (simple, linear, hand-tuned).
"""
import json, numpy as np, mujoco
KP, KD, HIP = 3.0, 0.5, 0.0  # hand-tuned on the sagittal push sweep (see docs/SIMULATION.md)
from .analysis import load, support_margin

def jid(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(m, n): return mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

def contact_feet(m, d):
    """Names of foot geoms with an active floor contact carrying normal force > 1 N."""
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor"); out = set(); f6 = np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        if floor in (c.geom1, c.geom2):
            other = c.geom2 if c.geom1 == floor else c.geom1
            mujoco.mj_contactForce(m, d, i, f6)
            if f6[0] > 1.0: out.add(m.geom(other).name)
    return sorted(out)

def run_stance(duration=4.0, push_n=0.0, push_dir=(1, 0), push_t=1.0, push_dt=0.1,
               controller="pd", seed=0, noise_deg=0.0, record=None, fps=30):
    """Return metrics dict; optionally append rendered frames into `record` (list)."""
    m, d = load(); rng = np.random.default_rng(seed)
    if noise_deg:
        for i in range(7, m.nq): d.qpos[i] += np.radians(rng.normal(0, noise_deg))
    mujoco.mj_forward(m, d)
    fb = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'{s}_foot') for s in 'lr']; foot0 = d.xpos[fb, :2].copy()
    com0 = d.subtree_com[1][:2].copy(); z0 = d.qpos[2]
    ank = {s: (aid(m, f"act_{s}_ankle_pitch"), aid(m, f"act_{s}_ankle_roll")) for s in "lr"}
    hipr = {s: aid(m, f"act_{s}_hip_roll") for s in "lr"}
    pz = {s: (aid(m, f"act_{s}_hip_pitch")) for s in "lr"}
    Kp, Kd = KP, KD  # rad per m, rad per m/s  (hand tuned; see docs/SIMULATION.md)
    peak = m.actuator_forcerange[:, 1]
    log = {"t": [], "com_x": [], "com_y": [], "pelvis_z": [], "margin": [], "sat": [], "nfeet": []}
    fell = False; t_fall = None; ren = None; cam = None
    if record is not None:
        ren = mujoco.Renderer(m, 360, 480); cam = mujoco.MjvCamera()
        cam.lookat[:] = (0.55, 0, 0.6); cam.distance = 4.2; cam.azimuth = 120; cam.elevation = -14
    next_frame = 0.0; dirv = np.array(push_dir, float); dirv /= np.linalg.norm(dirv)
    vcom = np.zeros(2); prev = com0.copy(); base = None
    integ = {"nonfinite": False, "max_anchor_err_m": 0.0, "max_penetration_m": 0.0, "min_pelvis_z_m": 9.0, "max_eq_violation": 0.0}
    while d.time < duration:
        com = d.subtree_com[1][:2]
        mujoco.mj_subtreeVel(m, d); vcom = d.subtree_linvel[1][:2].copy()
        d.ctrl[:] = 0.0
        if controller == "ankle":
            e = com - com0
            ux = np.clip(Kp * e[0] + Kd * vcom[0], -0.6, 0.6)
            uy = np.clip(Kp * e[1] + Kd * vcom[1], -0.3, 0.3)
            for s in "lr":
                d.ctrl[ank[s][0]] = ux        # +foot pitch tilts shin back -> CoM back
                d.ctrl[ank[s][1]] = -uy
                d.ctrl[pz[s]] = HIP * ux      # optional hip strategy
        on = push_t <= d.time < push_t + push_dt
        d.xfrc_applied[1, :2] = dirv * push_n if on else 0.0
        mujoco.mj_step(m, d)
        _integrity(m, d, integ)
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (d.qpos[4]**2 + d.qpos[5]**2), -1, 1)))
        if not fell and (d.qpos[2] < z0 - 0.25 or tilt > 30):
            fell = True; t_fall = float(d.time)
        if base is None and d.time >= push_t - 0.05:
            base = d.subtree_com[1][:2].copy()
        if int(d.time / m.opt.timestep) % 5 == 0 and d.time > 0.5:
            com = d.subtree_com[1][:2]
            log["t"].append(float(d.time)); log["com_x"].append(float(com[0] - (base if base is not None else com0)[0]))
            log["com_y"].append(float(com[1] - (base if base is not None else com0)[1])); log["pelvis_z"].append(float(d.qpos[2]))
            feet = contact_feet(m, d); log["nfeet"].append(len(feet))
            log["margin"].append(support_margin(m, d, feet) if feet else -1.0)
            log["sat"].append(int(np.sum(np.abs(d.actuator_force) >= peak * 0.999)))
        if ren is not None and d.time >= next_frame:
            ren.update_scene(d, cam); _push_arrow(ren.scene, d, dirv, push_n > 0 and push_t - 0.05 <= d.time < push_t + push_dt + 0.35, on)
            record.append(ren.render().copy())
            next_frame += 1.0 / fps
    t = np.array(log["t"]); cx = np.array(log["com_x"]); cy = np.array(log["com_y"])
    disp = np.hypot(cx, cy); post = t > push_t + push_dt
    final_margin = float(log["margin"][-1])
    if not fell and final_margin < 0:   # CoM outside support at the end = falling (finding F-006:
        fell = True; t_fall = None      # tilt/height-only criterion called slow topples "survived")
    final = float(disp[-1]); tail = t > duration - 0.5
    # recovered: upright, CoM back within 10 mm of pre-push position AND still (<3 mm p-p over last 0.5 s)
    recovered = (not fell) and final < 0.01 and float(np.ptp(disp[tail])) < 0.003
    settle = None
    if push_n and not fell and post.any():
        bad = np.where(post & (disp > 0.01))[0]
        settle = float(t[bad[-1]] - (push_t + push_dt)) if len(bad) else 0.0
    return {"controller": controller, "push_n": push_n, "push_dt_s": push_dt, "impulse_ns": push_n * push_dt,
            "push_dir": list(dirv), "seed": seed, "duration_s": duration, "fell": fell, "t_fall_s": t_fall,
            "survived": not fell, "recovered": bool(recovered), "settle_time_s": settle,
            "max_com_disp_mm": float(disp.max() * 1e3), "final_com_disp_mm": final * 1e3,
            "min_contact_margin_mm": float(min(log["margin"]) * 1e3), "final_contact_margin_mm": final_margin * 1e3,
            "max_saturated_actuators": int(max(log["sat"])),
            "integrity": integ,
            "foot_slip_mm": float(np.linalg.norm(d.xpos[fb, :2] - foot0, axis=1).max() * 1e3), "min_feet_in_contact": int(min(log["nfeet"])),
            "log": log}

def _integrity(m, d, g):
    """Per-step physical-consistency checks (F-008 investigation)."""
    g["nonfinite"] |= not (np.all(np.isfinite(d.qpos)) and np.all(np.isfinite(d.qvel)))
    for b in range(2, m.nbody):   # child origin must equal parent pose * fixed offset (tree cannot separate)
        p = m.body_parentid[b]
        g["max_anchor_err_m"] = max(g["max_anchor_err_m"], float(np.linalg.norm(d.xpos[p] + d.xmat[p].reshape(3, 3) @ m.body_pos[b] - d.xpos[b])))
    pen = max([float(-d.contact[i].dist) for i in range(d.ncon)] + [0.0])
    g["max_penetration_m"] = max(g["max_penetration_m"], pen); g["last_penetration_m"] = pen
    g["min_pelvis_z_m"] = min(g["min_pelvis_z_m"], float(d.qpos[2]))
    if d.nefc:  # finger tendon-coupling equality residuals
        eq = d.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        if eq.any(): g["max_eq_violation"] = max(g["max_eq_violation"], float(np.abs(d.efc_pos[eq]).max()))

def _push_arrow(scn, d, dirv, show, active):
    """Visual marker only: bright red while the force is applied, dim red for 0.35 s afterwards."""
    if not show or scn.ngeom >= scn.maxgeom: return
    p = d.xpos[1].copy(); a = p - np.r_[dirv, 0] * 0.7
    col = np.array([1, 0.05, 0.05, 1] if active else [0.6, 0.2, 0.2, 0.6], np.float32)
    mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.zeros(9), col)
    mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_ARROW, 0.04, a, p); scn.ngeom += 1

def push_sweep(controller, forces=range(0, 401, 25), push_dt=0.1, duration=4.0, direction=(1, 0)):
    rows = []
    for f in forces:
        r = run_stance(duration, float(f), direction, push_dt=push_dt, controller=controller); r.pop("log")
        rows.append(r)
    surv = [r["push_n"] for r in rows if r["survived"]]; rec = [r["push_n"] for r in rows if r["recovered"]]
    first_fail = next((r["push_n"] for r in rows if not r["survived"]), None)
    return {"controller": controller, "direction": list(direction), "rows": rows,
            "max_survived_n": max(surv) if surv else None, "max_recovered_n": max(rec) if rec else None,
            "first_fall_n": first_fail}
