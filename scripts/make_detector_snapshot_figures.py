#!/usr/bin/env python
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection, PolyCollection


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IN = ROOT / "results" / "v0_16_detector_snapshots"
DEFAULT_OUT = ROOT / "paper" / "figures" / "detector_snapshots" / "projection_planes"


def _set_style() -> None:
    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "mathtext.fontset": "stix",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "svg.fonttype": "none",
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "font.size": 9,
            "axes.labelsize": 9,
        }
    )


def _numbers(text: str, dtype=float) -> np.ndarray:
    text = (text or "").strip()
    if not text:
        return np.array([], dtype=dtype)
    return np.fromstring(text, sep=" ", dtype=dtype)


def _data_array(parent: ET.Element | None, name: str | None = None) -> np.ndarray:
    if parent is None:
        return np.array([])
    for arr in parent.findall("DataArray"):
        if name is None or arr.attrib.get("Name") == name:
            dtype = int if arr.attrib.get("type", "").startswith("Int") else float
            return _numbers(arr.text or "", dtype=dtype)
    return np.array([])


def _connectivity(conn: np.ndarray, offsets: np.ndarray) -> list[list[int]]:
    out: list[list[int]] = []
    start = 0
    for off in offsets.astype(int):
        out.append(conn[start:off].astype(int).tolist())
        start = int(off)
    return out


def read_vtp(path: Path) -> dict:
    root = ET.parse(path).getroot()
    piece = root.find(".//Piece")
    if piece is None:
        raise ValueError(f"invalid VTP file: {path}")
    points = _data_array(piece.find("Points")).reshape((-1, 3))
    polys = _connectivity(
        _data_array(piece.find("Polys"), "connectivity"),
        _data_array(piece.find("Polys"), "offsets"),
    )
    point_data = piece.find("PointData")
    normals = _data_array(point_data, "normal")
    normals = normals.reshape((-1, 3)) if normals.size else np.empty((0, 3))
    gaps = _data_array(point_data, "gap")
    return {"path": path, "points": points, "polys": polys, "normals": normals, "gaps": gaps}


def _unit(v: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n > 1.0e-14:
        return v / n
    if fallback is None:
        fallback = np.array([0.0, 0.0, 1.0])
    return _unit(fallback)


def _plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = _unit(normal)
    ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.85 else np.array([0.0, 1.0, 0.0])
    e1 = _unit(np.cross(ref, n), np.array([0.0, 1.0, 0.0]))
    e2 = _unit(np.cross(n, e1), np.array([1.0, 0.0, 0.0]))
    return e1, e2, n


def _project(points: np.ndarray, origin: np.ndarray, e1: np.ndarray, e2: np.ndarray) -> np.ndarray:
    rel = points - origin
    return np.column_stack((rel @ e1, rel @ e2))


def _case_label(stem: str) -> str:
    if "guide_slot" in stem:
        return "guide slot"
    if "finite_socket" in stem:
        return "ball-and-socket pendulum"
    if "bearing_inner" in stem:
        return "bearing inner raceway"
    if "bearing_outer" in stem:
        return "bearing outer raceway"
    return stem


def _draw_projected_mesh(ax, data: dict, origin: np.ndarray, e1: np.ndarray, e2: np.ndarray, color: str, label: str) -> None:
    uv = _project(data["points"], origin, e1, e2)
    polys = [uv[poly] for poly in data["polys"] if len(poly) == 3]
    if not polys:
        return
    ax.add_collection(PolyCollection(polys, facecolors=color, edgecolors=color, linewidths=0.8, alpha=0.12, label=label))
    edges = []
    for poly in polys:
        edges.extend([[poly[0], poly[1]], [poly[1], poly[2]], [poly[2], poly[0]]])
    ax.add_collection(LineCollection(edges, colors=color, linewidths=0.85, alpha=0.80))


def _prefixes(in_dir: Path) -> list[Path]:
    terrain_paths = sorted(in_dir.glob("*_detector_snapshot_terrain_patch.vtp"))
    return [p.with_name(p.name.removesuffix("_terrain_patch.vtp")) for p in terrain_paths]


def save_projection_plane(prefix: Path, out_dir: Path) -> list[Path]:
    terrain = read_vtp(prefix.with_name(prefix.name + "_terrain_patch.vtp"))
    body = read_vtp(prefix.with_name(prefix.name + "_body_patch.vtp"))
    contacts = read_vtp(prefix.with_name(prefix.name + "_sdf_contact_points.vtp"))
    if len(contacts["points"]) == 0:
        raise ValueError(f"no SDF contact points found for {prefix.name}")

    origin = contacts["points"].mean(axis=0)
    normal = contacts["normals"].mean(axis=0) if len(contacts["normals"]) else np.array([0.0, 0.0, 1.0])
    e1, e2, n = _plane_basis(normal)
    contact_uv = _project(contacts["points"], origin, e1, e2)

    fig, ax = plt.subplots(figsize=(3.15, 2.55), constrained_layout=True)
    _draw_projected_mesh(ax, terrain, origin, e1, e2, "#d67b1f", "fixed patch")
    _draw_projected_mesh(ax, body, origin, e1, e2, "#1f5c99", "moving patch")

    ax.scatter(contact_uv[:, 0], contact_uv[:, 1], s=24, c="#111111", edgecolors="#1c9a4b", linewidths=1.2, zorder=5)
    ax.scatter([0.0], [0.0], s=18, c="#1c9a4b", marker="x", linewidths=1.1, zorder=6)

    all_uv = [_project(terrain["points"], origin, e1, e2), _project(body["points"], origin, e1, e2), contact_uv]
    stacked = np.vstack([u for u in all_uv if len(u)])
    lo = stacked.min(axis=0)
    hi = stacked.max(axis=0)
    center = 0.5 * (lo + hi)
    span = max(float((hi - lo).max()), 1.0e-6)
    pad = 0.18 * span
    ax.set_xlim(center[0] - 0.5 * span - pad, center[0] + 0.5 * span + pad)
    ax.set_ylim(center[1] - 0.5 * span - pad, center[1] + 0.5 * span + pad)
    ax.set_aspect("equal", adjustable="box")

    axis_len = 0.22 * span
    origin_2d = np.array([center[0] - 0.43 * span, center[1] - 0.43 * span])
    ax.annotate("", xy=origin_2d + np.array([axis_len, 0.0]), xytext=origin_2d, arrowprops={"arrowstyle": "->", "lw": 0.9})
    ax.annotate("", xy=origin_2d + np.array([0.0, axis_len]), xytext=origin_2d, arrowprops={"arrowstyle": "->", "lw": 0.9})
    ax.text(*(origin_2d + np.array([axis_len * 1.05, 0.0])), r"$\xi_1$", ha="left", va="center")
    ax.text(*(origin_2d + np.array([0.0, axis_len * 1.05])), r"$\xi_2$", ha="center", va="bottom")
    ax.text(0.02, 0.97, _case_label(prefix.name), transform=ax.transAxes, ha="left", va="top")
    ax.text(
        0.02,
        0.06,
        r"$\Pi: (x-x_c)\cdot n=0,\quad h=(x-x_c)\cdot n$",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
    )
    ax.set_xlabel(r"local coordinate $\xi_1$ (m)")
    ax.set_ylabel(r"local coordinate $\xi_2$ (m)")
    ax.tick_params(direction="out", length=2.5, width=0.7)
    for spine in ax.spines.values():
        spine.set_linewidth(0.7)

    out_dir.mkdir(parents=True, exist_ok=True)
    outputs = []
    for suffix, kwargs in {".png": {"dpi": 600}, ".pdf": {}, ".svg": {}}.items():
        out = out_dir / f"{prefix.name}_projection_plane{suffix}"
        fig.savefig(out, bbox_inches="tight", pad_inches=0.02, **kwargs)
        outputs.append(out)
    plt.close(fig)
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(description="Draw 2D height-field projection planes for SDF contact snapshots.")
    parser.add_argument("--in-dir", type=Path, default=DEFAULT_IN)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    _set_style()
    outputs: list[Path] = []
    for prefix in _prefixes(args.in_dir):
        outputs.extend(save_projection_plane(prefix, args.out_dir))
    print("\n".join(str(p) for p in outputs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
