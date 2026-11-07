# SPDX-License-Identifier: Apache-2.0
"""Scope: quasi-static standing under joint PD only. NOT walking, NOT dynamic balance."""
import mujoco
from ziggy.analysis import load, support_margin, stance_sim

def test_static_support_margin_positive():
    m, d = load(); mujoco.mj_forward(m, d)
    assert support_margin(m, d) > 0.03

def test_pd_stance_holds_5s():
    r = stance_sim(5.0)
    assert r["pelvis_drop_m"] < 0.02 and r["min_cos_tilt"] > 0.995 and r["min_support_margin_m"] > 0.03, r
