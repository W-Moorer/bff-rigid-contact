from pathlib import Path

from calg.experiments.runner import run_cases
from calg.experiments.cases import make_plane_plane_case


def test_run_single_case(tmp_path: Path):
    metrics = run_cases(tmp_path, cases=[make_plane_plane_case(offset=0.05, n=2)], make_plots=False)
    assert len(metrics) == 1
    assert (tmp_path / "validation_summary.csv").exists()
    assert metrics[0].contacts > 0
    assert abs(metrics[0].min_gap - 0.05) < 1e-8
