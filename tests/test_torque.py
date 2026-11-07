# SPDX-License-Identifier: Apache-2.0
from ziggy.analysis import static_torque_cases

def test_standing_within_continuous_rating():
    for n, row in static_torque_cases()["stand"].items():
        assert row["ratio_rated"] < 1.0, (n, row)

def test_all_static_cases_within_peak():
    from ziggy.params import ACTUATORS
    for case, rows in static_torque_cases().items():
        for n, row in rows.items():
            if row["rating_verified"]:
                assert row["tau_nm"] < ACTUATORS[row["class"]]["peak_nm"], (case, n, row)
