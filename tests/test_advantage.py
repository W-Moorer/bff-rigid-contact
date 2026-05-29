from pathlib import Path

import pandas as pd

from calg.experiments.baselines import run_calg, run_linear_triangle
from calg.experiments.cases import make_sphere_plane_case
from calg.experiments.advantage import run_subcell_valley_accuracy, run_sliding_normal_continuity


def test_advantage_targeted_outputs(tmp_path: Path):
    rows = run_subcell_valley_accuracy(tmp_path, resolutions=[4])
    assert (tmp_path / "subcell_valley_accuracy.csv").exists()
    df = pd.read_csv(tmp_path / "subcell_valley_accuracy.csv")
    calg = df[df["method"] == "calg_curved_graph"].iloc[0]
    lin = df[df["method"] == "linear_triangle_bvh"].iloc[0]
    assert calg["abs_error"] <= 1.0e-8
    assert lin["abs_error"] > 1.0e-3

    normals = run_sliding_normal_continuity(tmp_path, n=4, samples=60)
    assert (tmp_path / "sliding_normal_continuity.csv").exists()
    assert normals[0].max_jump_rad > normals[1].max_jump_rad


def test_calg_accuracy_not_worse_than_linear_on_pole_aligned_sphere():
    case = make_sphere_plane_case(gap=0.08, n_lat=5, n_lon=10, plane_n=6)
    calg = run_calg(case.mesh_a, case.mesh_b, case.d_hat)
    linear = run_linear_triangle(case.mesh_a, case.mesh_b, case.d_hat)
    expected = float(case.expected_gap)

    assert calg.min_gap >= expected - 1.0e-10
    assert abs(calg.min_gap - expected) <= abs(linear.min_gap - expected) + 1.0e-12
