from __future__ import annotations

from dataclasses import dataclass, asdict
from time import perf_counter
from typing import Callable

import numpy as np

from calg.modeling.mesh import TriangleMesh
from calg.modeling.mesh_ops import subdivide_midpoint
from calg.core.primitives import build_primitives
from calg.core.bvh import BVH
from calg.core.triangle_distance import triangle_triangle_closest
from calg.solver.detector import ContactDetector, DetectionResult


@dataclass
class BaselineResult:
    method: str
    min_gap: float
    contacts: int
    candidate_pairs: int
    elapsed_s: float
    notes: str = ""

    def to_dict(self) -> dict:
        d = asdict(self)
        d["min_gap"] = float(self.min_gap)
        d["elapsed_s"] = float(self.elapsed_s)
        return d


def run_calg(mesh_a: TriangleMesh, mesh_b: TriangleMesh, d_hat: float, mu_min: float = 0.20) -> BaselineResult:
    detector = ContactDetector(d_hat=d_hat, mu_min=mu_min, two_sided=True, enable_interval_fallback=True, interval_depth=5)
    t0 = perf_counter()
    res = detector.detect(mesh_a, mesh_b)
    elapsed = perf_counter() - t0
    return BaselineResult(
        method="calg_curved_graph",
        min_gap=res.min_gap,
        contacts=len(res.contacts),
        candidate_pairs=res.stats.candidate_pairs,
        elapsed_s=elapsed,
        notes=f"fast_path_ratio={res.stats.fast_path_ratio:.6g}; curved_attempted={res.stats.curved_attempted}; interval_attempted={res.stats.interval_attempted}",
    )


def run_linear_triangle(mesh_a: TriangleMesh, mesh_b: TriangleMesh, d_hat: float, leaf_size: int = 8) -> BaselineResult:
    """Linear triangle BVH baseline using exact PL triangle-triangle distances."""
    t0 = perf_counter()
    prims_a = build_primitives(mesh_a, contact_margin=d_hat, error_safety=0.0)
    prims_b = build_primitives(mesh_b, contact_margin=d_hat, error_safety=0.0)
    bvh_a = BVH([p.aabb for p in prims_a], leaf_size=leaf_size)
    bvh_b = BVH([p.aabb for p in prims_b], leaf_size=leaf_size)
    pairs = bvh_a.query_pairs(bvh_b)
    min_gap = float("inf")
    contacts = 0
    for ia, ib in pairs:
        d = triangle_triangle_closest(prims_a[ia].vertices, prims_b[ib].vertices).distance
        if d < min_gap:
            min_gap = float(d)
        if d <= d_hat:
            contacts += 1
    elapsed = perf_counter() - t0
    return BaselineResult("linear_triangle_bvh", min_gap, contacts, len(pairs), elapsed)


def run_global_subdivision_linear(mesh_a: TriangleMesh, mesh_b: TriangleMesh, d_hat: float, levels: int = 1) -> BaselineResult:
    """Linear baseline after global midpoint subdivision.

    This is a computational-cost baseline, not a geometry-improving remesher for
    purely piecewise-linear inputs.  For analytic surfaces, the experiment
    scripts additionally regenerate finer meshes to study true mesh refinement.
    """
    a = mesh_a
    b = mesh_b
    t0 = perf_counter()
    for _ in range(max(0, int(levels))):
        a = subdivide_midpoint(a, name=f"{a.name}_sub")
        b = subdivide_midpoint(b, name=f"{b.name}_sub")
    prep = perf_counter() - t0
    r = run_linear_triangle(a, b, d_hat=d_hat)
    r.method = f"global_subdivision_linear_L{levels}"
    r.elapsed_s += prep
    r.notes = f"faces_a={len(a.faces)}; faces_b={len(b.faces)}"
    return r


def _points_for_trimesh_baseline(mesh: TriangleMesh) -> np.ndarray:
    tri = mesh.vertices[mesh.faces]
    centroids = tri.mean(axis=1)
    # Add edge midpoints to reduce bias without making the baseline too slow.
    mids = np.concatenate([
        0.5 * (tri[:, 0, :] + tri[:, 1, :]),
        0.5 * (tri[:, 1, :] + tri[:, 2, :]),
        0.5 * (tri[:, 2, :] + tri[:, 0, :]),
    ], axis=0)
    pts = np.vstack([mesh.vertices, centroids, mids])
    return pts


def run_trimesh_nearest(mesh_a: TriangleMesh, mesh_b: TriangleMesh, d_hat: float) -> BaselineResult:
    """Open-source baseline using trimesh.proximity.closest_point_naive.

    It samples vertices, face centroids, and edge midpoints on both surfaces and
    measures nearest distance to the opposite triangle mesh.  It is not a
    conservative contact solver; it is included as a reproducible open-source
    geometry baseline for the numerical comparison table.
    """
    try:
        import trimesh  # type: ignore
        from trimesh.proximity import closest_point_naive  # type: ignore
    except Exception as exc:  # pragma: no cover - optional dependency
        return BaselineResult("trimesh_nearest_sampled", float("nan"), 0, 0, 0.0, f"skipped:{type(exc).__name__}")

    t0 = perf_counter()
    tm_a = trimesh.Trimesh(vertices=mesh_a.vertices, faces=mesh_a.faces, process=False)
    tm_b = trimesh.Trimesh(vertices=mesh_b.vertices, faces=mesh_b.faces, process=False)
    pts_a = _points_for_trimesh_baseline(mesh_a)
    pts_b = _points_for_trimesh_baseline(mesh_b)
    _, da, _ = closest_point_naive(tm_b, pts_a)
    _, db, _ = closest_point_naive(tm_a, pts_b)
    dists = np.concatenate([np.asarray(da, float), np.asarray(db, float)])
    min_gap = float(np.min(dists)) if dists.size else float("inf")
    contacts = int(np.count_nonzero(dists <= d_hat))
    elapsed = perf_counter() - t0
    return BaselineResult("trimesh_nearest_sampled", min_gap, contacts, int(len(pts_a) + len(pts_b)), elapsed, "open_source_baseline=trimesh.closest_point_naive")


def run_all_baselines(mesh_a: TriangleMesh, mesh_b: TriangleMesh, d_hat: float, include_subdivision: bool = True) -> list[BaselineResult]:
    out = [
        run_calg(mesh_a, mesh_b, d_hat),
        run_linear_triangle(mesh_a, mesh_b, d_hat),
        run_trimesh_nearest(mesh_a, mesh_b, d_hat),
    ]
    if include_subdivision:
        out.append(run_global_subdivision_linear(mesh_a, mesh_b, d_hat, levels=1))
    return out
