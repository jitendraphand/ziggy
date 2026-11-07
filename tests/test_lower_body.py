# SPDX-License-Identifier: Apache-2.0
"""v0.2 lower body: evidence-backed checks. Heavy geometry runs in tools/lower_body_qa.py (called by tools/qa.py);
these tests validate the evidence and artifacts and that gate statuses are derived, not hand-set."""
import json, pathlib, math
import cadquery as cq, trimesh, pytest
EV = json.loads(pathlib.Path("docs/validation/lower_body_v0.2.json").read_text())
MAN = json.loads(pathlib.Path("hardware/lower_body/manifest.json").read_text())["parts"]
HC = json.loads(pathlib.Path("docs/validation/hand_calcs_v0.2.json").read_text())["checks"]

def test_no_static_interference():
    assert EV["interference"]["pairs_checked"] > 10000 and EV["interference"]["interferences"] == []

@pytest.mark.parametrize("j", ["hip_pitch", "knee", "ankle_pitch", "ankle_roll", "hip_yaw"])
def test_joint_sweep_clear(j):
    assert EV["joint_sweep_left"][j]["collisions"] == []

def test_hip_roll_adduction_collision_is_recorded():
    """Known failure: left-leg adduction <= -10 deg hits the right leg (other leg at zero)."""
    c = EV["joint_sweep_left"]["hip_roll"]["collisions"]
    assert c and all(x["angle"] < 0 for x in c) and any(x["fixed"].endswith("-R") for x in c)

@pytest.mark.parametrize("j", ["hip_roll", "hip_pitch", "knee", "ankle_pitch", "ankle_roll"])
def test_hard_stop_engages_outside_range_only(j):
    s = EV["joint_sweep_left"][j]["hard_stop"]
    assert s["free_within_range"] and s["engages_beyond_range"]

def test_hip_yaw_has_no_stop_yet():
    assert EV["joint_sweep_left"]["hip_yaw"]["hard_stop"] is None

def test_manifest_fields_and_files():
    for p in MAN:
        for k in ("id", "make_buy", "material", "process", "critical", "status", "mass_kg", "kind"):
            assert p[k] is not None and p[k] != "", (p["id"], k)
        if p["mirror_of"]:
            assert any(q["id"] == p["mirror_of"] for q in MAN); continue
        assert pathlib.Path(p["step"]).exists()
        if p["kind"] == "envelope-purchased": assert p["status"] == "envelope" and p["stl"] is None
        else: assert pathlib.Path(p["stl"]).exists()
        if p["stl"] and not p["printed"]: assert "NOT for printing" in p["stl_note"]

def test_step_files_valid():
    for p in MAN:
        if p["step"]:
            s = cq.importers.importStep(p["step"]).val()
            assert s.isValid() and s.Volume() > 0, p["id"]

def test_printed_stl_watertight_and_fit():
    assert all(EV["printed_fit"]["fits"].values())
    for p in MAN:
        if p["printed"] and p["stl"]:
            assert trimesh.load(p["stl"]).is_watertight, p["id"]

def test_blockers_are_flagged_not_invented():
    rs = [p for p in MAN if p["id"].startswith("KN-PLA") or p["id"].startswith("KN-ARA")]
    assert rs and all(p["critical"] == "DRAWING REQUIRED" for p in rs)
    assert any("BLOCKER" in f for p in MAN for f in p["flags"])

def test_mass_rollup_consistent():
    s = sum(p["mass_kg"] for p in MAN)
    assert math.isclose(s, EV["mass"]["total_known_kg"], rel_tol=0.01)
    assert EV["mass"]["fasteners_wiring_kg"] is None   # unknown, not invented

def test_hand_calc_status_matches_sf():
    for r in HC:
        if r["SF"] is not None and r["SF_required"] is not None:
            assert (r["status"] == "PASS") == (r["SF"] >= r["SF_required"]), r["id"]

def test_gates_are_derived_from_evidence():
    import sys; sys.path.insert(0, "tools")
    from engineering_gates import compute
    recorded = {g["id"]: g["status"] for g in json.loads(pathlib.Path("docs/ENGINEERING_GATES.json").read_text())}
    assert recorded == {g["id"]: g["status"] for g in compute()}
    assert recorded["G-10"] == "FAIL"     # no physical verification -> release blocked
