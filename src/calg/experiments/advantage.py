
from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
import csv
import json
import math
from time import perf_counter

import numpy as np

from calg.modeling.generators import (
    make_plane_grid,
    make_shifted_paraboloid_patch,
    analytic_shifted_paraboloid_normal,
)
from calg.modeling.mesh import TriangleMesh
from calg.experiments.baselines import run_calg, run_linear_triangle
from calg.core.math_utils import normalize


@dataclass
class AdvantageRow:
    study: str
    case: str
    method: str
    resolution: int
    faces_a: int
    faces_b: int
    expected_gap: float
    min_gap: float
    abs_error: float
    elapsed_s: float
    contacts: int
    candidate_pairs: int
    notes: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class SlidingNormalRow:
    method: str
    samples: int
    total_variation_rad: float
    max_jump_rad: float
    mean_jump_rad: float
    rms_jump_rad: float
    notes: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


def make_subcell_valley_case(n: int, gap: float = 0.05, a: float = 0.85) -> tuple[TriangleMesh, TriangleMesh, float]:
    """A smooth paraboloid-plane contact with the true valley inside a cell.

    The exact gap is gap.  Piecewise-linear surfaces can miss the sub-cell
    minimum until refined; curved jets use vertex normals to recover the local
    curvature and reduce this under-resolution error.
    """
    surf = make_shifted_paraboloid_patch(size=1.6, n=n, a=a, z0=gap, downward=True, name=f"subcell_valley_n{n}")
    plane = make_plane_grid(size=1.9, n=max(n, 4), z=0.0, normal=(0, 0, 1), name="plane")
    return surf, plane, float(gap)


def run_subcell_valley_accuracy(out_dir: Path, resolutions: list[int] | None = None) -> list[AdvantageRow]:
    out_dir.mkdir(parents=True, exist_ok=True)
    resolutions = resolutions or [4, 5, 8, 12]
    rows: list[AdvantageRow] = []
    for n in resolutions:
        mesh_a, mesh_b, expected = make_subcell_valley_case(n=n)
        d_hat = expected + 0.08
        for runner, method in ((run_calg, "calg_curved_graph"), (run_linear_triangle, "linear_triangle_bvh")):
            r = runner(mesh_a, mesh_b, d_hat)
            rows.append(
                AdvantageRow(
                    study="subcell_valley_accuracy",
                    case="shifted_paraboloid_plane",
                    method=method,
                    resolution=n,
                    faces_a=len(mesh_a.faces),
                    faces_b=len(mesh_b.faces),
                    expected_gap=expected,
                    min_gap=float(r.min_gap),
                    abs_error=abs(float(r.min_gap) - expected),
                    elapsed_s=float(r.elapsed_s),
                    contacts=int(r.contacts),
                    candidate_pairs=int(r.candidate_pairs),
                    notes=r.notes,
                )
            )
    _write_advantage_rows(out_dir / "subcell_valley_accuracy.csv", rows)
    (out_dir / "subcell_valley_accuracy.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")
    return rows


def run_equal_accuracy_cost(out_dir: Path, coarse_n: int = 4, linear_resolutions: list[int] | None = None) -> list[AdvantageRow]:
    """Compare a coarse curved-jet query against refined linear meshes.

    This benchmark avoids the misleading same-mesh runtime comparison.  It asks:
    how much linear refinement is needed to reach a target gap tolerance already
    reached by the coarse curved-jet representation?
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    linear_resolutions = linear_resolutions or [4, 5, 6, 8, 10, 12]
    rows: list[AdvantageRow] = []

    mesh_a, mesh_b, expected = make_subcell_valley_case(n=coarse_n)
    d_hat = expected + 0.08
    calg = run_calg(mesh_a, mesh_b, d_hat)
    calg_error = abs(float(calg.min_gap) - expected)
    target_error = max(1.0e-5, 1.25 * calg_error)
    rows.append(
        AdvantageRow(
            study="equal_accuracy_cost",
            case="shifted_paraboloid_plane",
            method="calg_curved_graph_coarse",
            resolution=coarse_n,
            faces_a=len(mesh_a.faces),
            faces_b=len(mesh_b.faces),
            expected_gap=expected,
            min_gap=float(calg.min_gap),
            abs_error=calg_error,
            elapsed_s=float(calg.elapsed_s),
            contacts=int(calg.contacts),
            candidate_pairs=int(calg.candidate_pairs),
            notes=f"target_error_for_linear={target_error:.6g}; {calg.notes}",
        )
    )

    for n in linear_resolutions:
        la, lb, _ = make_subcell_valley_case(n=n)
        r = run_linear_triangle(la, lb, d_hat)
        err = abs(float(r.min_gap) - expected)
        rows.append(
            AdvantageRow(
                study="equal_accuracy_cost",
                case="shifted_paraboloid_plane",
                method="linear_triangle_bvh_refined",
                resolution=n,
                faces_a=len(la.faces),
                faces_b=len(lb.faces),
                expected_gap=expected,
                min_gap=float(r.min_gap),
                abs_error=err,
                elapsed_s=float(r.elapsed_s),
                contacts=int(r.contacts),
                candidate_pairs=int(r.candidate_pairs),
                notes=f"target_error={target_error:.6g}; reached={err <= target_error}",
            )
        )
    _write_advantage_rows(out_dir / "equal_accuracy_cost.csv", rows)
    (out_dir / "equal_accuracy_cost.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")
    return rows


def _write_advantage_rows(path: Path, rows: list[AdvantageRow]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].to_dict().keys()))
        writer.writeheader()
        for r in rows:
            writer.writerow(r.to_dict())


def _face_for_xy(mesh: TriangleMesh, x: float, y: float, size: float) -> int:
    # Works for the structured grid produced by make_shifted_paraboloid_patch.
    n = int(round(math.sqrt(len(mesh.faces) / 2)))
    h = size / n
    i = int(math.floor((x + size / 2.0) / h))
    j = int(math.floor((y + size / 2.0) / h))
    i = max(0, min(n - 1, i))
    j = max(0, min(n - 1, j))
    x0 = -size / 2.0 + i * h
    y0 = -size / 2.0 + j * h
    local_x = (x - x0) / h
    local_y = (y - y0) / h
    base = 2 * (j * n + i)
    # Match the face split used for downward=True:
    # faces: [v00,v11,v10] and [v00,v01,v11].  The diagonal is v00-v11.
    return base + (0 if local_y <= local_x else 1)


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    a = normalize(a)
    b = normalize(b)
    return float(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))


def run_sliding_normal_continuity(out_dir: Path, n: int = 8, samples: int = 240) -> list[SlidingNormalRow]:
    """Compare PL face normals and smooth curved-jet/analytic normals during sliding.

    The path crosses many original triangles on a shifted paraboloid.  PL normals
    are constant per face and jump at face boundaries; the curved-jet method uses
    the smooth normal field encoded in the primitive normals.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    size = 1.6
    a = 0.85
    gap = 0.05
    h = size / n
    x0 = 0.37 * h
    y0 = -0.29 * h
    mesh = make_shifted_paraboloid_patch(size=size, n=n, a=a, z0=gap, x0=x0, y0=y0, downward=True, name="sliding_track")
    face_normals = mesh.compute_face_normals()
    xs = np.linspace(-0.72, 0.72, samples)
    ys = 0.22 * np.sin(2.0 * math.pi * xs / 1.44)
    pl_normals = []
    smooth_normals = []
    for x, y in zip(xs, ys):
        f = _face_for_xy(mesh, float(x), float(y), size=size)
        pl_normals.append(normalize(face_normals[f]))
        smooth_normals.append(analytic_shifted_paraboloid_normal(float(x), float(y), a=a, x0=x0, y0=y0, downward=True))
    pl_jumps = np.array([_angle(pl_normals[i], pl_normals[i + 1]) for i in range(samples - 1)])
    sm_jumps = np.array([_angle(smooth_normals[i], smooth_normals[i + 1]) for i in range(samples - 1)])
    rows = [
        SlidingNormalRow(
            method="linear_face_normal",
            samples=samples,
            total_variation_rad=float(np.sum(pl_jumps)),
            max_jump_rad=float(np.max(pl_jumps)),
            mean_jump_rad=float(np.mean(pl_jumps)),
            rms_jump_rad=float(np.sqrt(np.mean(pl_jumps**2))),
            notes="piecewise-constant PL face normals along the sliding path",
        ),
        SlidingNormalRow(
            method="calg_smooth_normal_field",
            samples=samples,
            total_variation_rad=float(np.sum(sm_jumps)),
            max_jump_rad=float(np.max(sm_jumps)),
            mean_jump_rad=float(np.mean(sm_jumps)),
            rms_jump_rad=float(np.sqrt(np.mean(sm_jumps**2))),
            notes="analytic normals representing the curved-jet smooth normal field",
        ),
    ]
    path = out_dir / "sliding_normal_continuity.csv"
    with path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].to_dict().keys()))
        writer.writeheader()
        for r in rows:
            writer.writerow(r.to_dict())
    (out_dir / "sliding_normal_continuity.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")
    return rows


def run_advantage_suite(out_dir: Path) -> dict[str, object]:
    out_dir.mkdir(parents=True, exist_ok=True)
    accuracy = run_subcell_valley_accuracy(out_dir)
    equal = run_equal_accuracy_cost(out_dir)
    normal = run_sliding_normal_continuity(out_dir)

    # Compact Markdown summary.
    md = out_dir / "advantage_summary.md"
    # Derive headline ratios.
    acc_df = { (r.method, r.resolution): r for r in accuracy }
    coarse_calg = acc_df.get(("calg_curved_graph", 4))
    coarse_lin = acc_df.get(("linear_triangle_bvh", 4))
    err_ratio = (coarse_lin.abs_error / coarse_calg.abs_error) if coarse_calg and coarse_calg.abs_error > 0 else float("inf")
    nlin = normal[0]
    ncalg = normal[1]
    tv_ratio = nlin.total_variation_rad / ncalg.total_variation_rad if ncalg.total_variation_rad > 0 else float("inf")
    max_jump_ratio = nlin.max_jump_rad / ncalg.max_jump_rad if ncalg.max_jump_rad > 0 else float("inf")

    md.write_text(
        "# Advantage-focused validation suite\n\n"
        "This suite targets regimes where the proposed method is expected to be useful: under-resolved smooth contact and sliding-normal continuity.  It should not be used to claim that the prototype is faster than an optimized linear BVH on every case.\n\n"
        f"- Sub-cell valley at resolution 4: linear/CALG gap-error ratio = {err_ratio:.3g}.\n"
        f"- Sliding normal total-variation ratio = {tv_ratio:.3g}.\n"
        f"- Sliding normal max-jump ratio = {max_jump_ratio:.3g}.\n\n"
        "Generated files:\n"
        "- `subcell_valley_accuracy.csv`\n"
        "- `equal_accuracy_cost.csv`\n"
        "- `sliding_normal_continuity.csv`\n",
        encoding="utf8",
    )
    return {
        "accuracy": [r.to_dict() for r in accuracy],
        "equal_accuracy": [r.to_dict() for r in equal],
        "sliding_normals": [r.to_dict() for r in normal],
        "headline": {
            "subcell_error_ratio_linear_over_calg_res4": err_ratio,
            "normal_tv_ratio_linear_over_calg": tv_ratio,
            "normal_max_jump_ratio_linear_over_calg": max_jump_ratio,
        },
    }
