from __future__ import annotations

from calg.experiments.gpu_benchmark import run_gpu_benchmark
from calg.experiments.convergence import run_convergence_study
from calg.experiments.baseline_comparison import run_baseline_comparison, comparison_cases
from calg.experiments.robustness import run_robustness_study


def test_v0_4_benchmark_harnesses(tmp_path):
    gpu_rows = run_gpu_benchmark(tmp_path / "gpu", sizes=[64], repeats=1)
    assert len(gpu_rows) == 1
    assert gpu_rows[0].n_a == 64

    conv_rows = run_convergence_study(tmp_path / "conv", resolutions=[4], families=["sphere_plane"])
    assert {r.method for r in conv_rows} >= {"calg_curved_graph", "linear_triangle_bvh"}

    base_rows = run_baseline_comparison(tmp_path / "base", cases=comparison_cases()[:1])
    assert any(r.method == "linear_triangle_bvh" for r in base_rows)
    assert any(r.method == "calg_curved_graph" for r in base_rows)

    rob_rows = run_robustness_study(tmp_path / "rob", num_cases=2, seed=7)
    assert len(rob_rows) == 2
    assert all(r.status == "passed" for r in rob_rows)
