from __future__ import annotations

import csv
import json
from dataclasses import dataclass, asdict
from pathlib import Path

from calg.experiments.cases import (
    default_cases,
    ValidationCase,
    make_plane_plane_case,
    make_sphere_plane_case,
    make_paraboloid_plane_case,
    make_wavy_sheet_plane_case,
)
from calg.modeling.generators import make_cylinder_patch, make_plane_grid
from calg.experiments.baselines import run_all_baselines


def comparison_cases() -> list[ValidationCase]:
    """Moderate-size cases for method-to-method timing comparisons.

    The default validation cylinder uses a denser plane to stress the detector;
    baseline comparisons keep sizes controlled so open-source sampled baselines
    and global subdivision baselines finish quickly in CPU-only CI.
    """
    gap = 0.05
    cyl = make_cylinder_patch(radius=0.5, length=1.2, n_theta=10, n_z=4, center=(0, 0, 0.5 + gap), axis="x", name="cylinder_small")
    plane = make_plane_grid(size=1.6, n=8, z=0.0, normal=(0, 0, 1), name="plane_small")
    cylinder_case = ValidationCase("cylinder_plane_line_contact_small", cyl, plane, d_hat=gap + 0.04, expected_gap=gap, description="Controlled-size cylinder-plane baseline comparison case.")
    return [
        make_plane_plane_case(n=4),
        make_sphere_plane_case(n_lat=5, n_lon=10, plane_n=6),
        make_paraboloid_plane_case(n=6),
        make_wavy_sheet_plane_case(n=6),
        cylinder_case,
    ]


@dataclass
class BaselineComparisonRow:
    case: str
    method: str
    expected_gap: float | str
    min_gap: float
    abs_error: float | str
    elapsed_s: float
    contacts: int
    candidate_pairs: int
    faces_a: int
    faces_b: int
    notes: str

    def to_dict(self) -> dict:
        return asdict(self)


def run_baseline_comparison(out_dir: str | Path, cases: list[ValidationCase] | None = None) -> list[BaselineComparisonRow]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = comparison_cases() if cases is None else cases
    rows: list[BaselineComparisonRow] = []
    for case in cases:
        include_subdivision = (len(case.mesh_a.faces) * len(case.mesh_b.faces) <= 8000)
        for res in run_all_baselines(case.mesh_a, case.mesh_b, case.d_hat, include_subdivision=include_subdivision):
            expected = case.expected_gap
            if expected is None:
                exp: float | str = ""
                err: float | str = ""
            else:
                exp = float(expected)
                err = abs(float(res.min_gap) - exp) if res.min_gap == res.min_gap else ""
            rows.append(
                BaselineComparisonRow(
                    case=case.name,
                    method=res.method,
                    expected_gap=exp,
                    min_gap=float(res.min_gap),
                    abs_error=err,
                    elapsed_s=float(res.elapsed_s),
                    contacts=int(res.contacts),
                    candidate_pairs=int(res.candidate_pairs),
                    faces_a=int(len(case.mesh_a.faces)),
                    faces_b=int(len(case.mesh_b.faces)),
                    notes=res.notes,
                )
            )
    csv_path = out_dir / "baseline_comparison.csv"
    with csv_path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].to_dict().keys()))
        writer.writeheader()
        for r in rows:
            writer.writerow(r.to_dict())
    (out_dir / "baseline_comparison.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")
    md = out_dir / "baseline_comparison.md"
    with md.open("w", encoding="utf8") as f:
        f.write("# Baseline comparison\n\n")
        f.write("Methods: CALG curved graph, linear triangle BVH, global midpoint-subdivision linear contact, and an optional open-source Trimesh sampled nearest-distance baseline.\n\n")
        f.write("| case | method | expected | min gap | abs error | time (s) | contacts | candidates/samples | notes |\n")
        f.write("|---|---|---:|---:|---:|---:|---:|---:|---|\n")
        for r in rows:
            f.write(f"| {r.case} | {r.method} | {r.expected_gap} | {r.min_gap:.6g} | {r.abs_error} | {r.elapsed_s:.4g} | {r.contacts} | {r.candidate_pairs} | {r.notes} |\n")
    return rows
