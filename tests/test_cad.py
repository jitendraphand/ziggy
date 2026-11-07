# SPDX-License-Identifier: Apache-2.0
import json, pathlib
import cadquery as cq, trimesh, pytest
from cad.parts import PARTS
from ziggy.params import print_envelope_mm

MAN = json.loads(pathlib.Path("hardware/manifest.json").read_text())
ENV = print_envelope_mm()
ROWS = {p["name"]: p for p in MAN["parts"]}

def test_manifest_covers_all_parts():
    assert set(ROWS) == set(PARTS)

@pytest.mark.parametrize("name", sorted(PARTS))
def test_step_reimports_valid(name):
    s = cq.importers.importStep(ROWS[name]["step"]).val()
    assert s.isValid() and s.Volume() > 0

@pytest.mark.parametrize("name", sorted(PARTS))
def test_stl_watertight(name):
    m = trimesh.load(ROWS[name]["stl"])
    assert m.is_watertight and m.volume > 0

@pytest.mark.parametrize("name", sorted(PARTS))
def test_fits_neptune4max_with_clearance(name):
    x, y, z = ROWS[name]["bbox_mm"]
    assert x <= ENV[0] and y <= ENV[1] and z <= ENV[2], (name, ROWS[name]["bbox_mm"], ENV)

def test_no_printed_part_claims_primary_structure():
    assert {p["load_class"] for p in MAN["parts"]} <= {"cover", "guide", "functional", "coupon"}

def test_pin_bore_matches_coupon_range():
    from cad import parts
    assert 2.9 <= parts.PIN <= 3.5   # covered by coupon_holes sizes
