# SPDX-License-Identifier: Apache-2.0
"""Generate the ZIGGY MuJoCo MJCF model from ziggy.params (no hand-edited XML)."""
from pathlib import Path
from math import radians
from .params import DIM, MASS, ACTUATORS, FINGERS, joint_table

JT = joint_table()
D = DIM

def _j(name, armature=0.02, damping=1.0):
    axis, lo, hi, _ = JT[name]
    return (f'<joint name="{name}" axis="{axis}" range="{lo} {hi}" '
            f'armature="{armature}" damping="{damping}"/>')

def _leg(s):
    y = D["hip_half_width"] * (1 if s == "l" else -1)
    th, sh, ah = D["thigh"], D["shin"], D["ankle_height"]
    fx = D["foot_fwd"]
    return f'''
    <body name="{s}_hip_yaw_link" pos="0 {y} {-D["pelvis_to_hip"]}">
      {_j(f"{s}_hip_yaw", 0.05, 2)}
      <inertial pos="0 0 0" mass="{MASS['hip_yaw']}" diaginertia="0.002 0.002 0.002"/>
      <body name="{s}_hip_roll_link">
        {_j(f"{s}_hip_roll", 0.05, 2)}
        <inertial pos="0 0 0" mass="{MASS['hip_roll']}" diaginertia="0.002 0.002 0.002"/>
        <body name="{s}_thigh">
          {_j(f"{s}_hip_pitch", 0.05, 2)}
          <inertial pos="0 0 {-th/2}" mass="{MASS['thigh']}" diaginertia="0.04 0.04 0.005"/>
          <geom class="vis" type="capsule" fromto="0 0 0 0 0 {-th}" size="0.045"/>
          <body name="{s}_shin" pos="0 0 {-th}">
            {_j(f"{s}_knee", 0.05, 2)}
            <inertial pos="0 0 {-sh/2}" mass="{MASS['shin']}" diaginertia="0.03 0.03 0.003"/>
            <geom class="vis" type="capsule" fromto="0 0 0 0 0 {-sh}" size="0.04"/>
            <body name="{s}_ankle_link" pos="0 0 {-sh}">
              {_j(f"{s}_ankle_pitch", 0.03, 1)}
              <inertial pos="0 0 0" mass="0.3" diaginertia="0.0005 0.0005 0.0005"/>
              <body name="{s}_foot">
                {_j(f"{s}_ankle_roll", 0.03, 1)}
                <inertial pos="{fx} 0 {-ah+0.02}" mass="{MASS['foot']}" diaginertia="0.002 0.006 0.007"/>
                <geom name="{s}_sole" class="col" type="box" pos="{fx} 0 {-ah+0.015}" size="{D['foot_len']/2} {D['foot_wid']/2} 0.015"/>
                <site name="{s}_sole_site" pos="{fx} 0 {-ah}" size="0.01"/>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>'''

def _finger(s, f, i):
    pl = D["phalanx"]
    if f == "thumb":
        ysign = 1 if s == "l" else -1
        pos = f'0.02 {ysign*0.03} -0.03'
        j0 = f'<joint name="{s}_thumb_opp" axis="0 0 1" range="0 90" armature="0.001" damping="0.05"/>'
    else:
        off = (i - 2.5) * D["finger_pitch"] * (1 if s == "l" else -1)
        pos = f'0 {off} {-D["palm_len"]}'
        j0 = ''
    names = ["mcp", "pip", "dip"]
    xml_open, xml_close = "", ""
    z = 0.0
    for k, n in enumerate(names):
        p = pos if k == 0 else f"0 0 {-pl[k-1]}"
        xml_open += f'''
          <body name="{s}_{f}_{n}_link" pos="{p}">
            {j0 if k == 0 else ''}
            <joint name="{s}_{f}_{n}" axis="0 1 0" range="0 95" armature="0.0005" damping="0.02"/>
            <inertial pos="0 0 {-pl[k]/2}" mass="{MASS['phalanx']}" diaginertia="1e-6 1e-6 1e-6"/>
            <geom class="hand" type="capsule" fromto="0 0 0 0 0 {-pl[k]}" size="0.008"/>'''
        xml_close += "</body>"
        z += pl[k]
    xml_open += f'<site name="{s}_{f}_tip" pos="0 0 {-pl[2]}" size="0.004"/>'
    return xml_open + xml_close

def _hand(s):
    fingers = "".join(_finger(s, f, i) for i, f in enumerate(FINGERS))
    return f'''
            <body name="{s}_palm" pos="0 0 -0.04">
              <inertial pos="0 0 -0.045" mass="{MASS['hand_palm']}" diaginertia="0.0005 0.0005 0.0003"/>
              <geom class="hand" type="box" pos="0 0 -0.045" size="0.02 0.045 0.045"/>
              <site name="{s}_grasp" pos="0.03 0 -0.07" size="0.01"/>
              {fingers}
            </body>'''

def _arm(s):
    y = D["shoulder_half_width"] * (1 if s == "l" else -1)
    ua, fa = D["upper_arm"], D["forearm"]
    return f'''
      <body name="{s}_shoulder_a" pos="0 {y} {D['torso']-0.05}">
        {_j(f"{s}_shoulder_pitch", 0.03, 1)}
        <inertial pos="0 0 0" mass="{MASS['upper_arm_a']}" diaginertia="0.0006 0.0006 0.0006"/>
        <body name="{s}_shoulder_b">
          {_j(f"{s}_shoulder_roll", 0.03, 1)}
          <inertial pos="0 0 0" mass="{MASS['upper_arm_b']}" diaginertia="0.0006 0.0006 0.0006"/>
          <body name="{s}_upper_arm">
            {_j(f"{s}_shoulder_yaw", 0.02, 1)}
            <inertial pos="0 0 {-ua/2}" mass="{MASS['upper_arm']}" diaginertia="0.012 0.012 0.002"/>
            <geom class="vis" type="capsule" fromto="0 0 0 0 0 {-ua}" size="0.035"/>
            <body name="{s}_forearm" pos="0 0 {-ua}">
              {_j(f"{s}_elbow", 0.02, 1)}
              <inertial pos="0 0 {-fa/2}" mass="{MASS['forearm']}" diaginertia="0.007 0.007 0.001"/>
              <geom class="vis" type="capsule" fromto="0 0 0 0 0 {-fa}" size="0.03"/>
              <body name="{s}_wrist_a" pos="0 0 {-fa}">
                {_j(f"{s}_wrist_yaw", 0.005, 0.2)}
                <inertial pos="0 0 0" mass="{MASS['wrist_a']}" diaginertia="1e-4 1e-4 1e-4"/>
                <body name="{s}_wrist_b">
                  {_j(f"{s}_wrist_pitch", 0.005, 0.2)}
                  <inertial pos="0 0 0" mass="{MASS['wrist_b']}" diaginertia="1e-4 1e-4 1e-4"/>
                  <body name="{s}_hand">
                    {_j(f"{s}_wrist_roll", 0.005, 0.2)}
                    <inertial pos="0 0 -0.02" mass="0.1" diaginertia="1e-4 1e-4 1e-4"/>
                    {_hand(s)}
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>'''

def standing_height():
    return D["pelvis_to_hip"] + D["thigh"] + D["shin"] + D["ankle_height"]

def build_xml():
    kp = {"AK80-64": 400, "RS04": 400, "AK70-10": 120, "WRIST_TBD": 20}
    acts, eqs = [], []
    for name, (_, lo, hi, cls) in JT.items():
        a = ACTUATORS[cls]
        acts.append(f'<position name="act_{name}" joint="{name}" kp="{kp[cls]}" kv="{kp[cls]*0.05}" '
                    f'ctrlrange="{radians(lo):.5f} {radians(hi):.5f}" forcerange="{-a["peak_nm"]} {a["peak_nm"]}"/>')
    fs = ACTUATORS["FINGER_SERVO_TBD"]["peak_nm"]
    for s in ("l", "r"):
        acts.append(f'<position name="act_{s}_thumb_opp" joint="{s}_thumb_opp" kp="2" ctrlrange="0 1.5708" forcerange="{-fs} {fs}"/>')
        for f in FINGERS:
            acts.append(f'<position name="act_{s}_{f}_flex" joint="{s}_{f}_mcp" kp="2" ctrlrange="0 1.6581" forcerange="{-fs} {fs}"/>')
            # tendon coupling (underactuated): pip = mcp, dip = 0.8*pip
            eqs.append(f'<joint joint1="{s}_{f}_pip" joint2="{s}_{f}_mcp" polycoef="0 1 0 0 0"/>')
            eqs.append(f'<joint joint1="{s}_{f}_dip" joint2="{s}_{f}_pip" polycoef="0 0.8 0 0 0"/>')
    z0 = standing_height() + 0.002
    return f'''<!-- SPDX-License-Identifier: CERN-OHL-S-2.0 ; GENERATED by ziggy/mjcf.py - do not edit -->
<mujoco model="ziggy_v0_1">
  <compiler angle="degree" autolimits="true"/>
  <!-- NOTE: angle=degree converts joint ranges only; actuator ctrlrange is always RADIANS (fix F-007) -->
  <option timestep="0.002" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="960"/></visual>
  <default>
    <geom contype="0" conaffinity="0" rgba="0.75 0.78 0.82 1"/>
    <!-- F-008: limb/torso/head geoms collide with the FLOOR only (contype 2 vs floor conaffinity 2); self-collision is off -->
    <default class="vis"><geom contype="2" conaffinity="0" friction="0.9 0.01 0.001"/></default>
    <default class="hand"><geom contype="2" conaffinity="0" friction="0.9 0.01 0.001"/></default>
    <default class="col"><geom contype="1" conaffinity="1" friction="0.9 0.01 0.001"/></default>
  </default>
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="5 5 0.1" contype="1" conaffinity="3" rgba="0.3 0.3 0.3 1"/>
    <body name="pelvis" pos="0 0 {z0:.4f}">
      <freejoint name="root"/>
      <inertial pos="0 0 0" mass="{MASS['pelvis']}" diaginertia="0.05 0.03 0.05"/>
      <geom class="vis" type="box" size="0.08 0.14 0.06"/>
      <site name="imu" pos="0 0 0" size="0.01"/>
      {_leg("l")}{_leg("r")}
      <body name="torso" pos="0 0 {D['waist']}">
        {_j("waist_yaw", 0.05, 2)}
        <inertial pos="0 0 {D['torso']*0.45}" mass="{MASS['torso']}" diaginertia="0.35 0.28 0.12"/>
        <geom class="vis" type="box" pos="0 0 {D['torso']/2}" size="0.11 0.17 {D['torso']/2}"/>
        <body name="neck" pos="0 0 {D['torso']}">
          {_j("neck_yaw", 0.01, 0.5)}
          <inertial pos="0 0 0.03" mass="{MASS['neck']}" diaginertia="3e-4 3e-4 3e-4"/>
          <body name="head" pos="0 0 0.05">
            {_j("neck_pitch", 0.01, 0.5)}
            <inertial pos="0 0 0.09" mass="{MASS['head']}" diaginertia="0.008 0.008 0.006"/>
            <geom class="vis" type="sphere" pos="0 0 0.09" size="0.11"/>
            <site name="head_top" pos="0 0 0.20" size="0.01"/>
          </body>
        </body>
        {_arm("l")}{_arm("r")}
      </body>
    </body>
  </worldbody>
  <equality>
    {chr(10).join("    "+e for e in eqs)}
  </equality>
  <actuator>
    {chr(10).join("    "+a for a in acts)}
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <accelerometer name="imu_acc" site="imu"/>
    <touch name="l_foot_touch" site="l_sole_site"/>
    <touch name="r_foot_touch" site="r_sole_site"/>
  </sensor>
</mujoco>
'''

def write(path="sim/ziggy.xml"):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(build_xml())
    return p

if __name__ == "__main__":
    print(write())
