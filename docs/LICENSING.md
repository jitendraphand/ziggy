<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Licensing scopes

| Path | SPDX |
|---|---|
| `ziggy/`, `tools/`, `tests/`, `cad/build.py`, `cad/__init__.py`, CI | Apache-2.0 |
| `cad/parts.py` (hardware design source), `hardware/**` (STEP, STL, manifest), `sim/ziggy.xml` | CERN-OHL-S-2.0 |
| `README.md`, `SAFETY.md`, `docs/**` | CC-BY-4.0 |

Third-party content: none vendored. Python dependencies (MuJoCo Apache-2.0, CadQuery Apache-2.0,
trimesh MIT, NumPy BSD-3-Clause, pytest MIT) are installed from PyPI under their own licenses.
Under CERN-OHL-S, the "Source" of a hardware modification is the edited `cad/parts.py` and its outputs.
