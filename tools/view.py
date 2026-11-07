# SPDX-License-Identifier: Apache-2.0
"""Interactive MuJoCo viewer running the same PD/ankle controller (needs a display).
  python tools/view.py [--controller ankle]   (double-click a body, Ctrl+right-drag to push it)"""
import argparse, os, sys, time
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); os.chdir(ROOT); sys.path.insert(0, ROOT)
import mujoco, mujoco.viewer, numpy as np
import ziggy.scenarios as S
from ziggy.analysis import load
ap = argparse.ArgumentParser(); ap.add_argument("--controller", default="pd", choices=["pd", "ankle"]); a = ap.parse_args()
m, d = load(); mujoco.mj_forward(m, d); com0 = d.subtree_com[1][:2].copy()
with mujoco.viewer.launch_passive(m, d) as v:
    while v.is_running():
        d.ctrl[:] = 0
        if a.controller == "ankle":
            mujoco.mj_subtreeVel(m, d); e = d.subtree_com[1][:2] - com0; vc = d.subtree_linvel[1][:2]
            ux = np.clip(S.KP * e[0] + S.KD * vc[0], -0.6, 0.6); uy = np.clip(S.KP * e[1] + S.KD * vc[1], -0.3, 0.3)
            for s in "lr":
                d.ctrl[S.aid(m, f"act_{s}_ankle_pitch")] = ux; d.ctrl[S.aid(m, f"act_{s}_ankle_roll")] = -uy
        mujoco.mj_step(m, d); v.sync(); time.sleep(m.opt.timestep)
