# SPDX-License-Identifier: Apache-2.0
"""Simulation assumption checks, failure identification, determinism."""
import json, pathlib, numpy as np, mujoco, pytest
from ziggy.analysis import load, solve_ik, support_margin, household_reach
from ziggy.params import joint_table, ACTUATORS
from ziggy.scenarios import run_stance, contact_feet

def test_physics_settings():
    m, _ = load()
    assert np.allclose(m.opt.gravity, [0, 0, -9.81]) and m.opt.timestep == 0.002
    assert m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE          # floating base, not pinned
    assert np.isclose(m.body_mass.sum(), 46.0, atol=0.5)

def test_actuator_units_gear_and_ctrlrange_radians():
    """F-007 regression: ctrlrange must be radians and equal the joint range."""
    m, _ = load(); jt = joint_table()
    for i in range(m.nu):
        j = m.actuator_trnid[i, 0]
        assert m.actuator_gear[i, 0] == 1.0                     # force == joint torque (N*m)
        assert np.allclose(m.actuator_ctrlrange[i], m.jnt_range[j], atol=1e-4), m.actuator(i).name
        n = m.actuator(i).name[4:]
        if n in jt: assert m.actuator_forcerange[i, 1] == ACTUATORS[jt[n][3]]["peak_nm"]

def test_contact_based_support_both_feet():
    m, d = load()
    for _ in range(500): mujoco.mj_step(m, d)
    feet = contact_feet(m, d)
    assert feet == ["l_sole", "r_sole"] and support_margin(m, d, feet) > 0.03

def test_single_foot_support_margin_is_smaller():
    m, d = load()
    for _ in range(500): mujoco.mj_step(m, d)
    assert support_margin(m, d, ["l_sole"]) < 0 < support_margin(m, d, ["l_sole", "r_sole"])

def test_ik_unreachable_flagged_and_limits_respected():
    m, d = load()
    r = solve_ik(m, d, "l_grasp", (2.0, 0.2, 1.0), "l", q0=m.qpos0.copy())
    assert r["error_m"] > 0.5 and r["within_limits"]

def test_household_known_gaps_reported():
    r = household_reach("l")
    assert r["dining_table_0.75m"] > 5 and r["floor_pickup_0.10m"] > 5   # requires squat/whole-body IK (not implemented)

def test_push_pd_survives_small_but_does_not_settle_quickly():
    r = run_stance(4.0, 150.0, controller="pd")
    assert not r["fell"] and not r["recovered"]

def test_push_ankle_recovers_250N():
    r = run_stance(4.0, 250.0, controller="ankle")
    assert r["recovered"] and r["settle_time_s"] < 2.5 and r["max_saturated_actuators"] == 0

@pytest.mark.parametrize("ctrl", ["pd", "ankle"])
def test_large_push_detected_as_fall(ctrl):
    assert run_stance(4.0, 400.0, controller=ctrl)["fell"]

def test_slow_topple_detected():
    """F-006 regression: CoM outside support at end must count as a fall."""
    r = run_stance(4.0, 200.0, controller="pd")
    assert r["fell"] and r["final_contact_margin_mm"] < 0

def test_deterministic():
    a = run_stance(2.0, 100.0, seed=3, noise_deg=1.0); b = run_stance(2.0, 100.0, seed=3, noise_deg=1.0)
    assert a["max_com_disp_mm"] == b["max_com_disp_mm"]

def test_assembly_manifest_consistent():
    A = json.loads(pathlib.Path("hardware/assembly.json").read_text())
    man = {p["name"] for p in json.loads(pathlib.Path("hardware/manifest.json").read_text())["parts"]}
    placed = {i["part"] for i in A["instances"]}
    assert placed <= man and man - placed == {"coupon_holes", "coupon_tube_fit"}
    assert "SCHEMATIC" in A["status"]

# ---- F-008 regression: falls must stay physically consistent (tree intact, no floor pass-through)
@pytest.mark.parametrize("force,ctrl", [(0.0, "pd"), (250.0, "ankle"), (300.0, "ankle"), (400.0, "pd")])
def test_trajectory_integrity(force, ctrl):
    g = run_stance(4.0, force, controller=ctrl)["integrity"]
    assert not g["nonfinite"]
    assert g["max_anchor_err_m"] < 1e-9            # generalized coords: bodies cannot separate
    assert g["min_pelvis_z_m"] > 0.05              # torso/pelvis rest ON the floor, not through it
    assert g["max_penetration_m"] < 0.05           # soft-contact transient at impact (solref 0.02 s default)
    assert g["last_penetration_m"] < 0.01          # resting penetration after settling
    assert g["max_eq_violation"] < 0.03            # finger coupling equality (rad), soft constraint

def test_body_geoms_collide_with_floor_only():
    m, _ = load()
    fl = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    for g in range(m.ngeom):
        if g == fl: continue
        hits_floor = (m.geom_contype[g] & m.geom_conaffinity[fl]) or (m.geom_contype[fl] & m.geom_conaffinity[g])
        assert hits_floor, m.geom(g).name
