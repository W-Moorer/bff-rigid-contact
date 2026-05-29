from pathlib import Path

import pandas as pd

from calg.experiments.final_strengthening import run_final_strengthening_suite


def test_final_strengthening_suite_outputs(tmp_path: Path):
    summary = run_final_strengthening_suite(tmp_path)
    assert (tmp_path / "randomized_subcell_statistics.csv").exists()
    assert (tmp_path / "contact_force_friction_smoothness.csv").exists()
    assert (tmp_path / "fallback_validation.csv").exists()
    assert summary["randomized_subcell_cases"] >= 8
    assert summary["randomized_calg_better_ratio"] > 0.5
    assert summary["force_max_jump_ratio_linear_over_calg"] > 10.0
    assert summary["fallback_suite_passed"]

    rand = pd.read_csv(tmp_path / "randomized_subcell_statistics.csv")
    assert set(rand["calg_better"].unique()).issubset({True, False})
