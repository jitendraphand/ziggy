# SPDX-License-Identifier: Apache-2.0
from ziggy.analysis import ik_benchmark, household_reach

def test_ik_benchmark_reachable_targets():
    for side in "lr":
        r = ik_benchmark(n=150, seed=1, side=side)
        assert r["success_rate"] >= 0.95, r
        assert r["median_err_mm"] < 1.0, r
        assert r["limits_respected_rate"] == 1.0, r

def test_household_reach_standing():
    r = household_reach("l")
    for k in ("kitchen_counter_0.90m", "upper_shelf_1.40m", "door_handle_1.00m", "light_switch_1.20m"):
        assert r[k] < 5.0, (k, r)
    # Known gaps (documented, need squat/whole-body IK): table 0.75 m, floor pickup.
