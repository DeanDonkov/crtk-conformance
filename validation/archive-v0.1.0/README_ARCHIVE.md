# Archived v0.1.0 validation outputs (historical, immutable)

These 185 JSON files and the `tables/` directory are the outputs of the v0.1.0 validation run reported in the
RC1/RC2 manuscripts (commit 5393272c, 2026-09-02T19:05:40Z; see `meta.json`). They were moved here from
`validation/results/` unchanged on 2026-09-03 (git mv; sha256 of every file recorded in
`preprint/rc2/BASELINE_RC1_HASHES.txt` and `preprint/rc3/BASELINE_RC2_HASHES.txt`). They document what version
0.1.0 did, including the defects found by Reviewer #2 (M4, M5, M6); they are not evidence for version 0.1.1,
whose archive is `validation/v0.1.1/`. `validation/analyze.py` regenerates the v0.1.0 tables and figures from
this directory (`python validation/analyze.py validation/archive-v0.1.0`).
