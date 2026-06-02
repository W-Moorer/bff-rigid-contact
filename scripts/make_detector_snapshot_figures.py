#!/usr/bin/env python
from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IN = ROOT / "results" / "v0_16_detector_snapshots"
DEFAULT_OUT = ROOT / "paper" / "figures" / "detector_snapshots"


PART_STYLE = {
    1: ("#c9ced6", 0.22, 0.22, "terrain patch"),
    2: ("#7fb6d6", 0.28, 0.22, "moving patch"),
    3: ("#f2a93b", 0.82, 0.62, "candidate terrain"),
    4: ("#111111", 0.58, 0.62, "candidate body"),
}

LINE_STYLE = {
    5: ("#d07a00", 0.70, 0.55),  # terrain AABB
    6: ("#111111", 0.70, 0.55),  # body AABB
    7: ("#525866", 0.45, 0.42),  # initial closest segment
    8: ("#c43c39", 0.90, 0.80),  # graph contact segment
    9: ("#1f8f4d", 1.35, 0.95),  # representative normal
}


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
        }
    )


def _numbers(text: str, dtype=float) -> np.ndarray:
    text = (text or "").strip()
    if not text:
        return np.array([], dtype=dtype)
    return np.fromstring(text, sep=" ", dtype=dtype)


def _data_array(piece: ET.Element, section: str, name: str | None = None) -> np.ndarray:
    parent = piece.find(section)
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
    n_lines = int(piece.attrib.get("NumberOfLines", "0"))
    n_polys = int(piece.attrib.get("NumberOfPolys", "0"))
    pts = _data_array(piece, "Points").reshape((-1, 3))
    line_conn = _data_array(piece, "Lines", "connectivity")
    line_offsets = _data_array(piece, "Lines", "offsets")
    poly_conn = _data_array(piece, "Polys", "connectivity")
    poly_offsets = _data_array(piece, "Polys", "offsets")
    part_id = _data_array(piece, "CellData", "part_id",).astype(int)
    lines = _connectivity(line_conn, line_offsets)
    polys = _connectivity(poly_conn, poly_offsets)
    if len(part_id) != n_lines + n_polys:
        raise ValueError(f"cell-data length mismatch in {path}")
    return {
        "path": path,
        "points": pts,
        "lines": lines,
        "polys": polys,
        "line_parts": part_id[:n_lines],
        "poly_parts": part_id[n_lines:],
    }


def _axis_equal(ax, points: np.ndarray) -> None:
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    center = 0.5 * (lo + hi)
    radius = 0.58 * max(float((hi - lo).max()), 1.0e-9)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect((1, 1, 1))


def _view_for(stem: str) -> tuple[float, float]:
    if "guide_slot" in stem:
        return 24, -62
    if "ball_joint" in stem:
        return 18, -48
    if "bearing" in stem:
        return 32, -58
    return 25, -55


def draw_snapshot(ax, datasets: list[dict]) -> None:
    pts = np.vstack([data["points"] for data in datasets if len(data["points"])])

    for data in datasets:
        dpts = data["points"]
        polys = data["polys"]
        lines = data["lines"]
        poly_parts = data["poly_parts"]
        line_parts = data["line_parts"]

        for part in (1, 2, 3, 4):
            faces = [dpts[poly] for poly, pid in zip(polys, poly_parts) if int(pid) == part]
            if not faces:
                continue
            color, alpha, linewidth, _ = PART_STYLE[part]
            coll = Poly3DCollection(faces, facecolor=color, edgecolor=color, linewidth=linewidth, alpha=alpha)
            ax.add_collection3d(coll)

        for part in (5, 6):
            segs = [dpts[line] for line, pid in zip(lines, line_parts) if int(pid) == part]
            if not segs:
                continue
            color, linewidth, alpha = LINE_STYLE[part]
            ax.add_collection3d(Line3DCollection(segs, colors=color, linewidths=linewidth, alpha=alpha))

    _axis_equal(ax, pts)
    elev, azim = _view_for(datasets[0]["path"].stem)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def group_paths(prefix: Path) -> list[Path]:
    return [
        prefix.with_name(prefix.name + suffix)
        for suffix in ("_terrain_patch.vtp", "_body_patch.vtp", "_terrain_aabb.vtp", "_body_aabb.vtp")
    ]


def save_one(prefix: Path, out_dir: Path) -> list[Path]:
    datasets = [read_vtp(path) for path in group_paths(prefix)]
    fig = plt.figure(figsize=(3.10, 2.62), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    draw_snapshot(ax, datasets)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = prefix.name + "_overlay"
    outputs = []
    for suffix, kwargs in {
        ".png": {"dpi": 600},
        ".pdf": {},
        ".svg": {},
    }.items():
        out = out_dir / f"{stem}{suffix}"
        fig.savefig(out, transparent=False, bbox_inches="tight", pad_inches=0.01, **kwargs)
        outputs.append(out)
    plt.close(fig)
    return outputs


def save_overview(prefixes: list[Path], out_dir: Path) -> list[Path]:
    fig = plt.figure(figsize=(7.20, 5.10), constrained_layout=True)
    for i, prefix in enumerate(prefixes, 1):
        ax = fig.add_subplot(2, 2, i, projection="3d")
        draw_snapshot(ax, [read_vtp(path) for path in group_paths(prefix)])
    outputs = []
    for suffix, kwargs in {
        ".png": {"dpi": 600},
        ".pdf": {},
        ".svg": {},
    }.items():
        out = out_dir / f"04_1_detector_snapshot_overview{suffix}"
        fig.savefig(out, transparent=False, bbox_inches="tight", pad_inches=0.02, **kwargs)
        outputs.append(out)
    plt.close(fig)
    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(description="Render CALG coarse-detector snapshot VTP files for the manuscript.")
    parser.add_argument("--in-dir", type=Path, default=DEFAULT_IN)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    _set_style()
    terrain_paths = sorted(args.in_dir.glob("*_detector_snapshot_terrain_patch.vtp"))
    prefixes = [p.with_name(p.name.removesuffix("_terrain_patch.vtp")) for p in terrain_paths]
    if not prefixes:
        raise SystemExit(f"no split detector snapshot VTP files found in {args.in_dir}")
    outputs: list[Path] = []
    for prefix in prefixes:
        outputs.extend(save_one(prefix, args.out_dir))
    overview_paths: list[Path] = []
    for token in ("guide_slot", "ball_joint", "bearing_inner", "bearing_outer"):
        match = next((p for p in prefixes if token in p.name), None)
        if match is not None:
            overview_paths.append(match)
    outputs.extend(save_overview(overview_paths, args.out_dir))
    print("\n".join(str(p) for p in outputs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
