# SPDX-License-Identifier: Apache-2.0
"""Export every part to hardware/step + hardware/stl and write hardware/manifest.json."""
import hashlib, json, sys
from pathlib import Path
import cadquery as cq
from cad.parts import PARTS

def main(out="hardware"):
    out = Path(out); (out/"step").mkdir(parents=True, exist_ok=True); (out/"stl").mkdir(parents=True, exist_ok=True)
    man = {"generator": "cad/parts.py (CadQuery)", "units": "mm", "parts": []}
    for name, (fn, mat, qty, lc, orient) in PARTS.items():
        s = fn().val()
        bb = s.BoundingBox()
        # place on bed: min z = 0
        s = s.translate(cq.Vector(-bb.center.x, -bb.center.y, -bb.zmin)); bb = s.BoundingBox()
        st, sl = out/"step"/f"{name}.step", out/"stl"/f"{name}.stl"
        cq.exporters.export(cq.Workplane().add(s), str(st))
        cq.exporters.export(cq.Workplane().add(s), str(sl), tolerance=0.05, angularTolerance=0.2)
        man["parts"].append({"name": name, "material": mat, "qty": qty, "load_class": lc,
            "print_orientation": orient, "bbox_mm": [round(bb.xlen, 2), round(bb.ylen, 2), round(bb.zlen, 2)],
            "volume_cm3": round(s.Volume()/1000, 2), "valid_brep": bool(s.isValid()),
            "step": str(st), "stl": str(sl), "stl_sha256": hashlib.sha256(sl.read_bytes()).hexdigest()})
        print(f"{name:24s} {bb.xlen:7.1f} x {bb.ylen:6.1f} x {bb.zlen:6.1f} mm valid={s.isValid()}")
    (out/"manifest.json").write_text(json.dumps(man, indent=2) + "\n")

if __name__ == "__main__":
    main(*sys.argv[1:])
