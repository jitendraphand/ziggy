# SPDX-License-Identifier: Apache-2.0
"""Release gates: block unvalidated hardware claims from reaching docs."""
import json, pathlib, re
CLAIMS = json.loads(pathlib.Path("docs/claims.json").read_text())
ALLOWED = {"designed", "simulated", "untested", "validated-physical"}

def test_claim_status_and_evidence():
    for c in CLAIMS:
        assert c["status"] in ALLOWED, c
        if c["status"] == "validated-physical":
            for e in c["evidence"]:
                assert pathlib.Path(e).exists(), c

def test_no_overclaims_in_docs():
    validated = {c["id"] for c in CLAIMS if c["status"] == "validated-physical"}
    banned = {"WALK": r"\b(walks|is walking|walking verified|dynamically balanced)\b",
              "SAFE_HOME": r"\b(safe for (home|household) use|household[- ]ready|production[- ]ready)\b",
              "STRUCT": r"\bprinted (joints?|parts?) (are|is) (load[- ]rated|structural)\b",
              "WORLD_CLASS": r"\b(is|are) (a )?(world[- ]class|best[- ]in[- ]class)|surpass(es|ing)? world|\bis (a )?finished (engineering )?product",
              "MFG_RELEASE": r"\breleased for manufactur|\bready for manufactur"}
    for f in [pathlib.Path("README.md")] + list(pathlib.Path("docs").rglob("*.md")):
        t = f.read_text().lower()
        for cid, pat in banned.items():
            if cid not in validated:
                assert not re.search(pat, t), (f, cid)

def test_spdx_headers():
    for f in list(pathlib.Path("ziggy").glob("*.py")) + list(pathlib.Path("cad").glob("*.py")) + list(pathlib.Path("tools").glob("*.py")):
        assert "SPDX-License-Identifier" in f.read_text().splitlines()[0], f
