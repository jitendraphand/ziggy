# SPDX-License-Identifier: Apache-2.0
import mujoco, numpy as np
from ziggy.params import joint_table, ACTUATORS
from ziggy.analysis import load

def test_model_loads_and_counts():
    m, d = load()
    jt = joint_table()
    assert len(jt) == 29                       # 12 leg + 1 waist + 2 neck + 14 arm
    assert m.nu == len(jt) + 12                # + 2 hands x (5 flex + thumb opposition)
    for n in jt: assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) >= 0

def test_joint_limits_sane_and_symmetric():
    jt = joint_table()
    for n, (axis, lo, hi, cls) in jt.items():
        assert lo < hi and -180 <= lo and hi <= 180, n
        assert cls in ACTUATORS
    for n, (_, lo, hi, _) in jt.items():
        if n.startswith("l_"):
            _, rlo, rhi, _ = jt["r_" + n[2:]]
            assert sorted([abs(lo), abs(hi)]) == sorted([abs(rlo), abs(rhi)]), n

def test_actuator_force_limits_match_ratings():
    m, _ = load(); jt = joint_table()
    for i in range(m.nu):
        n = m.actuator(i).name[4:]
        if n in jt:
            assert np.isclose(m.actuator_forcerange[i][1], ACTUATORS[jt[n][3]]["peak_nm"])

def test_mass_and_stature_budget():
    m, d = load(); mujoco.mj_forward(m, d)
    assert 40 <= m.body_mass.sum() <= 50
    assert 1.60 <= d.site("head_top").xpos[2] <= 1.75
