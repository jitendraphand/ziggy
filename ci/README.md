# CI workflow (pending enablement)

`github-actions-qa.yml` runs `tools/qa.py` on every push and PR. To enable it, copy it to `.github/workflows/qa.yml` (the push needs a token with `workflow` scope).
