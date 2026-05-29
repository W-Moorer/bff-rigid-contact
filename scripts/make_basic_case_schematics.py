#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from calg.experiments.cases import (
    ValidationCase,
    make_cylinder_plane_case,
    make_paraboloid_plane_case,
    make_plane_plane_case,
    make_sphere_plane_case,
    make_wavy_sheet_plane_case,
)
from calg.modeling.mesh import TriangleMesh


CASE_BUILDERS = [
    ("01_plane_plane_gap", make_plane_plane_case, {"n": 4}),
    ("02_sphere_plane_gap", make_sphere_plane_case, {"n_lat": 8, "n_lon": 16, "plane_n": 8}),
    ("03_paraboloid_plane_gap", make_paraboloid_plane_case, {"n": 10}),
    ("04_open_wavy_sheet_plane", make_wavy_sheet_plane_case, {"n": 10}),
    ("05_cylinder_plane_line_contact", make_cylinder_plane_case, {"n_theta": 18, "n_z": 5}),
]


def _mesh_collection(mesh: TriangleMesh, face_color: str, edge_color: str, alpha: float) -> Poly3DCollection:
    tris = mesh.vertices[mesh.faces]
    coll = Poly3DCollection(tris, facecolors=face_color, edgecolors=edge_color, linewidths=0.35, alpha=alpha)
    coll.set_antialiased(True)
    return coll


def _axis_limits(meshes: list[TriangleMesh]) -> tuple[np.ndarray, np.ndarray]:
    pts = np.vstack([m.vertices for m in meshes])
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    span = hi - lo
    span[span < 1e-8] = 1.0
    pad = 0.08 * span
    return lo - pad, hi + pad


def _set_equalish_axes(ax, meshes: list[TriangleMesh]) -> None:
    lo, hi = _axis_limits(meshes)
    center = 0.5 * (lo + hi)
    radius = 0.5 * float(max(hi - lo))
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    z_span = max(float(hi[2] - lo[2]), 0.18 * radius)
    ax.set_zlim(center[2] - 0.55 * z_span, center[2] + 0.55 * z_span)
    ax.set_box_aspect((1.0, 1.0, 0.55))


def _gap_segment(case: ValidationCase) -> tuple[np.ndarray, np.ndarray]:
    a = case.mesh_a.vertices
    b = case.mesh_b.vertices
    if case.name == "plane_plane_gap":
        xy = np.array([0.0, 0.0])
        za = float(np.median(case.mesh_a.vertices[:, 2]))
        zb = float(np.median(case.mesh_b.vertices[:, 2]))
        return np.array([xy[0], xy[1], za]), np.array([xy[0], xy[1], zb])

    ia = int(np.argmin(a[:, 2]))
    pa = a[ia].copy()
    z_plane = float(np.median(b[:, 2]))
    pb = np.array([pa[0], pa[1], z_plane], dtype=float)
    return pa, pb


def _plot_case(case: ValidationCase, out_path: Path, title: str) -> None:
    fig = plt.figure(figsize=(8.0, 6.2), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(_mesh_collection(case.mesh_b, "#5B8DEF", "#355FAD", 0.34))
    ax.add_collection3d(_mesh_collection(case.mesh_a, "#F08A4B", "#9A4D24", 0.78))

    pa, pb = _gap_segment(case)
    lower, upper = (pa, pb) if pa[2] <= pb[2] else (pb, pa)
    ax.plot(
        [lower[0], upper[0]],
        [lower[1], upper[1]],
        [lower[2], upper[2]],
        color="#111111",
        linewidth=2.2,
        linestyle="--",
    )
    ax.scatter([lower[0], upper[0]], [lower[1], upper[1]], [lower[2], upper[2]], color="#111111", s=20)

    expected = "" if case.expected_gap is None else f"expected gap = {case.expected_gap:.3g}"
    mid = 0.5 * (lower + upper)
    ax.text(mid[0], mid[1], mid[2], expected, color="#111111", fontsize=8)
    ax.text2D(0.03, 0.94, "Surface A", transform=ax.transAxes, color="#9A4D24", fontsize=10, weight="bold")
    ax.text2D(0.03, 0.895, "Surface B / plane", transform=ax.transAxes, color="#355FAD", fontsize=10, weight="bold")

    _set_equalish_axes(ax, [case.mesh_a, case.mesh_b])
    ax.view_init(elev=24, azim=-48)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(title, pad=14)
    ax.grid(False)
    ax.xaxis.pane.set_alpha(0.0)
    ax.yaxis.pane.set_alpha(0.0)
    ax.zaxis.pane.set_alpha(0.0)
    fig.tight_layout()
    fig.savefig(out_path)
    plt.close(fig)


def generate(out_dir: Path) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for stem, builder, kwargs in CASE_BUILDERS:
        case = builder(**kwargs)
        path = out_dir / f"{stem}.png"
        title = case.name.replace("_", " ")
        _plot_case(case, path, title)
        paths.append(path)
    return paths


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate schematic images for the basic analytic CALG cases.")
    parser.add_argument("--out-dir", default="docs/figures/basic_cases")
    args = parser.parse_args()
    for path in generate(Path(args.out_dir)):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
