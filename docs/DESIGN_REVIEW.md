<!-- SPDX-License-Identifier: CC-BY-4.0 -->
# Design review v0.2: fail, fix, retest

| ID | Finding | Fix | Retest |
|---|---|---|---|
| F-009 | Knee arms clipped the proximal bridge (41 mm³ at 130°) | knee bridge offset +8 mm | sweep: 0 collisions at knee |
| F-010 | Hip yaw envelope mapped downward into hip roll | corrected the R_up yaw orientation | static interference 0 |
| F-011 | Rear-cap M3 bosses were tangent to the outer wall, so the STEP reimported as an invalid solid (4 files) | bosses moved to a 47 mm radius | tests/test_lower_body.py::test_step_files_valid |
| F-012 | Leg-to-leg check added: left hip roll adduction ≤-10° collides with the right leg | **not fixed**: the options are to widen hip spacing (HIP_Y 110 mm) or limit adduction in software and at the stop | G-02 stays FAIL |

## Open fails (kept failed)
G-02 leg-to-leg collision · G-03 hip yaw has no stop · G-05 hand calcs HC-04/06/09/14/16 · G-06 mass 31 vs 22 kg · G-07 CAD/sim geometry mismatch (sim not updated, on purpose)
· G-08 stature 1870 mm · G-09 RobStride 04 drawing and the AK80 tilting-moment rating · G-10 no physical verification · G-11 self-collision checked only for the left-leg sweep.

The pytest suite passing means the evidence is self-consistent. It does **not** mean the design is ready for production.
