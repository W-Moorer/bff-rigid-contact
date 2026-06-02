from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Polygon
import numpy as np


mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset": "dejavuserif",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
        "font.size": 8.5,
        "axes.linewidth": 0.6,
    }
)


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "figures" / "method_pipeline_components"

BLUE = "#2F6FAE"
BLUE_LIGHT = "#D9E8F7"
GREEN = "#3C8D5A"
GREEN_LIGHT = "#DDEFE4"
ORANGE = "#D58A2A"
ORANGE_LIGHT = "#F4DFBD"
GRAY = "#686868"
GRAY_LIGHT = "#DADADA"
BLACK = "#1A1A1A"
RED = "#B83A3A"
RED_LIGHT = "#F3D7D7"


def normalize(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


VIEW_DIR = normalize(np.array([0.80, -1.0, 0.82]))
SCREEN_X = normalize(np.cross(np.array([0.0, 0.0, 1.0]), VIEW_DIR))
SCREEN_Y = normalize(np.cross(VIEW_DIR, SCREEN_X))


def project(points):
    pts = np.asarray(points, dtype=float)
    return np.column_stack([pts @ SCREEN_X, pts @ SCREEN_Y])


def zero_surface_height(x, y):
    return 0.18 * np.sin(1.55 * x - 0.25) + 0.10 * y - 0.05


def signed_gap(p):
    p = np.asarray(p, dtype=float)
    return p[..., 2] - zero_surface_height(p[..., 0], p[..., 1])


def draw_arrow(ax, p0, p1, color=BLACK, lw=1.1, ms=9, zorder=10):
    arr = FancyArrowPatch(
        p0,
        p1,
        arrowstyle="-|>",
        mutation_scale=ms,
        linewidth=lw,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=zorder,
    )
    ax.add_patch(arr)
    return arr


def set_limits(ax, pts, pad_frac=0.08):
    pts = np.asarray(pts, dtype=float)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(maxs - mins)
    pad = pad_frac * span
    ax.set_xlim(center[0] - 0.5 * span - pad, center[0] + 0.5 * span + pad)
    ax.set_ylim(center[1] - 0.5 * span - pad, center[1] + 0.5 * span + pad)
    ax.set_aspect("equal")
    ax.axis("off")


def cube_corners(bounds):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    return np.array(
        [
            [xmin, ymin, zmin],
            [xmax, ymin, zmin],
            [xmax, ymax, zmin],
            [xmin, ymax, zmin],
            [xmin, ymin, zmax],
            [xmax, ymin, zmax],
            [xmax, ymax, zmax],
            [xmin, ymax, zmax],
        ]
    )


CUBE_EDGES = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
]


def draw_wire_cube(ax, bounds, color=GRAY, lw=0.65, ls="solid", zorder=3):
    c = cube_corners(bounds)
    q = project(c)
    for a, b in CUBE_EDGES:
        ax.plot([q[a, 0], q[b, 0]], [q[a, 1], q[b, 1]], color=color, lw=lw, ls=ls, zorder=zorder)
    return c


def draw_lattice(ax, values):
    for x in values:
        for y in values:
            p = project(np.column_stack([np.full_like(values, x), np.full_like(values, y), values]))
            ax.plot(p[:, 0], p[:, 1], color=GRAY_LIGHT, lw=0.35, zorder=1)
    for x in values:
        for z in values:
            p = project(np.column_stack([np.full_like(values, x), values, np.full_like(values, z)]))
            ax.plot(p[:, 0], p[:, 1], color=GRAY_LIGHT, lw=0.35, zorder=1)
    for y in values:
        for z in values:
            p = project(np.column_stack([values, np.full_like(values, y), np.full_like(values, z)]))
            ax.plot(p[:, 0], p[:, 1], color=GRAY_LIGHT, lw=0.35, zorder=1)


def draw_zero_surface(ax):
    xs = np.linspace(-1.0, 1.0, 22)
    ys = np.linspace(-1.0, 1.0, 22)
    quads = []
    depths = []
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            cell_xy = np.array(
                [
                    [xs[i], ys[j]],
                    [xs[i + 1], ys[j]],
                    [xs[i + 1], ys[j + 1]],
                    [xs[i], ys[j + 1]],
                ]
            )
            z = zero_surface_height(cell_xy[:, 0], cell_xy[:, 1])
            cell = np.column_stack([cell_xy, z])
            quads.append(cell)
            depths.append((cell @ VIEW_DIR).mean())
    for idx in np.argsort(depths):
        poly = Polygon(
            project(quads[idx]),
            closed=True,
            facecolor=BLUE_LIGHT,
            edgecolor=BLUE,
            lw=0.16,
            alpha=0.58,
            zorder=2,
        )
        ax.add_patch(poly)


def draw_sdf_nodes(ax, values, query):
    nodes = np.array([[x, y, z] for x in values for y in values for z in values])
    depths = nodes @ VIEW_DIR
    for p in nodes[np.argsort(depths)]:
        q = project(p[None, :])[0]
        g = signed_gap(p)
        color = BLUE if g >= 0.0 else RED
        ax.scatter(q[0], q[1], s=10, color=color, alpha=0.68, edgecolors="white", linewidths=0.25, zorder=4)

    stencil_values = np.array([-0.5, 0.0, 0.5, 1.0])
    stencil_nodes = np.array(
        [[x, y, z] for x in stencil_values for y in stencil_values for z in stencil_values]
    )
    q_stencil = project(stencil_nodes)
    ax.scatter(
        q_stencil[:, 0],
        q_stencil[:, 1],
        s=18,
        facecolors="none",
        edgecolors=ORANGE,
        linewidths=0.75,
        alpha=0.95,
        zorder=5,
    )

    draw_wire_cube(ax, (-0.5, 1.0, -0.5, 1.0, -0.5, 1.0), color=ORANGE, lw=1.0, ls=(0, (4, 2)), zorder=5)
    q = project(query[None, :])[0]
    ax.scatter(q[0], q[1], s=30, color=BLACK, edgecolors="white", linewidths=0.5, zorder=8)
    ax.text(q[0] + 0.06, q[1] + 0.03, r"$q$", color=BLACK, fontsize=9, zorder=10)


def draw_local_axes(ax):
    origin = project(np.array([[-1.1, -1.1, -1.1]]))[0]
    ex = project(np.array([[-1.1, -1.1, -1.1], [-0.62, -1.1, -1.1]]))
    ey = project(np.array([[-1.1, -1.1, -1.1], [-1.1, -0.62, -1.1]]))
    ez = project(np.array([[-1.1, -1.1, -1.1], [-1.1, -1.1, -0.62]]))
    draw_arrow(ax, ex[0], ex[1], color=GRAY, lw=0.8, ms=7, zorder=8)
    draw_arrow(ax, ey[0], ey[1], color=GRAY, lw=0.8, ms=7, zorder=8)
    draw_arrow(ax, ez[0], ez[1], color=GRAY, lw=0.8, ms=7, zorder=8)
    ax.text(ex[1, 0] + 0.02, ex[1, 1], r"$\xi_1$", color=GRAY, fontsize=8)
    ax.text(ey[1, 0], ey[1, 1] + 0.02, r"$\xi_2$", color=GRAY, fontsize=8)
    ax.text(ez[1, 0], ez[1, 1] + 0.02, r"$\eta$", color=GRAY, fontsize=8)
    ax.scatter(origin[0], origin[1], s=5, color=GRAY, zorder=8)


def draw_field_panel(ax):
    values = np.linspace(-1.0, 1.0, 5)
    query = np.array([0.18, -0.15, 0.42])
    draw_lattice(ax, values)
    draw_zero_surface(ax)
    draw_wire_cube(ax, (-1, 1, -1, 1, -1, 1), color=GRAY, lw=0.8, zorder=3)
    draw_sdf_nodes(ax, values, query)
    draw_local_axes(ax)

    extent_pts = [cube_corners((-1, 1, -1, 1, -1, 1)), query[None, :]]
    projected = np.vstack([project(p) for p in extent_pts])
    set_limits(ax, projected, pad_frac=0.16)
    ax.text(
        0.02,
        1.06,
        "local tricubic signed-gap field",
        transform=ax.transAxes,
        ha="left",
        va="bottom",
        fontsize=9.2,
        color=BLACK,
        clip_on=False,
    )
    ax.text(
        0.60,
        -0.08,
        r"$4\times4\times4$ stencil",
        transform=ax.transAxes,
        ha="left",
        va="center",
        fontsize=8.6,
        color=ORANGE,
        clip_on=False,
    )
    ax.text(
        0.05,
        -0.08,
        r"$g=0$ surface",
        transform=ax.transAxes,
        ha="left",
        va="center",
        fontsize=9,
        color=BLUE,
        clip_on=False,
    )


def draw_response_panel(ax):
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect("equal")
    ax.axis("off")

    x = np.linspace(0.12, 0.88, 300)
    y0 = 0.50 + 0.045 * np.sin(2.2 * np.pi * (x - 0.12) / 0.76 - 0.35)
    ax.fill_between(x, y0, 0.88, color=BLUE_LIGHT, alpha=0.55, zorder=1)
    ax.fill_between(x, 0.16, y0, color=RED_LIGHT, alpha=0.40, zorder=1)
    ax.plot(x, y0, color=BLUE, lw=1.5, zorder=4)

    q = np.array([0.58, 0.64])
    foot_x = q[0] - 0.035
    foot_y = np.interp(foot_x, x, y0)
    foot = np.array([foot_x, foot_y])
    normal = normalize(np.array([0.18, 0.42]))
    draw_arrow(ax, foot, foot + 0.19 * normal, color=GREEN, lw=1.35, ms=10, zorder=8)
    ax.plot([q[0], foot[0]], [q[1], foot[1]], color=BLACK, lw=1.0, ls=(0, (3, 2)), zorder=6)
    ax.scatter([q[0]], [q[1]], s=34, color=BLACK, edgecolors="white", linewidths=0.5, zorder=9)
    ax.scatter([foot[0]], [foot[1]], s=22, color=BLUE, edgecolors="white", linewidths=0.4, zorder=8)

    ax.text(0.08, 0.92, "continuous response query", ha="left", va="top", fontsize=9.2, color=BLACK)
    ax.text(q[0] + 0.035, q[1] + 0.015, r"$q$", fontsize=9, color=BLACK)
    ax.text(0.65, 0.71, r"$g(q)$", fontsize=9, color=BLACK)
    ax.text(foot[0] + 0.12, foot[1] + 0.08, r"$n_c=\nabla g/\|\nabla g\|$", fontsize=8.6, color=GREEN)
    ax.text(0.17, 0.79, r"$g>0$ open gap", fontsize=8.4, color=BLUE)
    ax.text(0.17, 0.25, r"$g<0$ active contact", fontsize=8.4, color=RED)
    ax.text(
        0.50,
        0.07,
        r"$g(q)=\sum_{i,j,k} c_{ijk}\xi_1^i\xi_2^j\eta^k$",
        fontsize=8.3,
        color=BLACK,
        ha="center",
    )


def save(fig, name):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "svg", "png"):
        kwargs = {"bbox_inches": "tight", "pad_inches": 0.03}
        if ext == "png":
            kwargs["dpi"] = 600
        fig.savefig(OUT_DIR / f"{name}.{ext}", **kwargs)
    plt.close(fig)


def main():
    fig = plt.figure(figsize=(6.7, 3.0))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.18, 1.0], wspace=0.08)
    draw_field_panel(fig.add_subplot(gs[0, 0]))
    draw_response_panel(fig.add_subplot(gs[0, 1]))
    save(fig, "10_tricubic_sdf_signed_gap_schematic")


if __name__ == "__main__":
    main()
