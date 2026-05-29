from __future__ import annotations

import csv
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from time import perf_counter

from calg.experiments.cases import (
    make_sphere_plane_case,
    make_paraboloid_plane_case,
    ValidationCase,
)
from calg.modeling.generators import make_cylinder_patch, make_plane_grid
from calg.experiments.baselines import run_calg, run_linear_triangle
from calg.modeling.mesh import TriangleMesh
import numpy as np
import math


@dataclass
class ConvergenceRow:
    family: str
    resolution: int
    faces_a: int
    faces_b: int
    h_a: float
    h_b: float
    method: str
    expected_gap: float
    min_gap: float
    abs_error: float
    elapsed_s: float
    contacts: int
    candidate_pairs: int
    notes: str

    def to_dict(self) -> dict:
        return asdict(self)


def _rotation_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / max(np.linalg.norm(axis), 1e-15)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x*x*C, x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C, y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ], dtype=float)


def _rotate_about_center(mesh: TriangleMesh, center: np.ndarray, angle: float = 0.37) -> TriangleMesh:
    R = _rotation_matrix(np.array([0.3, 1.0, 0.2]), angle)
    v = center + (R @ (mesh.vertices - center).T).T
    nrm = (R @ mesh.vertex_normals.T).T
    return TriangleMesh(v, mesh.faces.copy(), nrm, mesh.name + "_rotated")


def _case_family(name: str, n: int) -> ValidationCase:
    if name == "sphere_plane":
        case = make_sphere_plane_case(gap=0.08, n_lat=max(4, n), n_lon=max(8, 2 * n), plane_n=max(6, n))
        center = np.array([0.0, 0.0, 1.08])
        case.mesh_a = _rotate_about_center(case.mesh_a, center=center, angle=0.37)
        case.name = "rotated_sphere_plane_gap"
        case.description = "Rotated sphere sampling over plane; analytic gap is unchanged but the closest point is not forced to be a mesh vertex."
        return case
    if name == "paraboloid_plane":
        return make_paraboloid_plane_case(gap=0.06, n=max(4, n))
    if name == "cylinder_plane":
        gap = 0.05
        cyl = make_cylinder_patch(radius=0.5, length=1.2, n_theta=max(8, 2 * n), n_z=max(3, n // 2), center=(0, 0, 0.5 + gap), axis="x", name="cylinder")
        plane = make_plane_grid(size=1.6, n=max(6, n), z=0.0, normal=(0, 0, 1), name="plane")
        return ValidationCase("cylinder_plane_line_contact", cyl, plane, d_hat=gap + 0.04, expected_gap=gap, description="Cylinder-plane convergence case with controlled plane resolution.")
    raise ValueError(f"unknown convergence family {name!r}")


def run_convergence_study(
    out_dir: str | Path,
    resolutions: list[int] | None = None,
    families: list[str] | None = None,
) -> list[ConvergenceRow]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    resolutions = [4, 6, 8, 10, 12] if resolutions is None else resolutions
    families = ["sphere_plane", "paraboloid_plane", "cylinder_plane"] if families is None else families
    rows: list[ConvergenceRow] = []

    for family in families:
        for n in resolutions:
            case = _case_family(family, n)
            expected = float(case.expected_gap if case.expected_gap is not None else 0.0)
            for runner in (run_calg, run_linear_triangle):
                result = runner(case.mesh_a, case.mesh_b, case.d_hat)
                rows.append(
                    ConvergenceRow(
                        family=family,
                        resolution=int(n),
                        faces_a=int(len(case.mesh_a.faces)),
                        faces_b=int(len(case.mesh_b.faces)),
                        h_a=float(case.mesh_a.max_edge_length()),
                        h_b=float(case.mesh_b.max_edge_length()),
                        method=result.method,
                        expected_gap=expected,
                        min_gap=float(result.min_gap),
                        abs_error=abs(float(result.min_gap) - expected),
                        elapsed_s=float(result.elapsed_s),
                        contacts=int(result.contacts),
                        candidate_pairs=int(result.candidate_pairs),
                        notes=result.notes,
                    )
                )

    csv_path = out_dir / "mesh_convergence.csv"
    with csv_path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].to_dict().keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(row.to_dict())
    (out_dir / "mesh_convergence.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")

    md = out_dir / "mesh_convergence.md"
    with md.open("w", encoding="utf8") as f:
        f.write("# Mesh refinement convergence study\n\n")
        f.write("Generated by `calg.experiments.convergence`. Values are automatically computed.\n\n")
        f.write("| family | n | method | faces A/B | hA | expected | min gap | abs error | time (s) | contacts |\n")
        f.write("|---|---:|---|---:|---:|---:|---:|---:|---:|---:|\n")
        for r in rows:
            f.write(f"| {r.family} | {r.resolution} | {r.method} | {r.faces_a}/{r.faces_b} | {r.h_a:.4g} | {r.expected_gap:.6g} | {r.min_gap:.6g} | {r.abs_error:.3e} | {r.elapsed_s:.4g} | {r.contacts} |\n")
    return rows
