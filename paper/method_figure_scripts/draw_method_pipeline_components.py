from pathlib import Path as FsPath
import math

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import (
    Arc,
    Circle,
    Ellipse,
    FancyArrowPatch,
    PathPatch,
    Polygon,
    Rectangle,
)
from matplotlib.path import Path as MplPath
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


ROOT = FsPath(__file__).resolve().parents[1]
OUT_DIR = ROOT / "figures" / "method_pipeline_components"

BLUE = "#2F6FAE"
BLUE_LIGHT = "#CFE0F2"
GREEN = "#3C8D5A"
GREEN_LIGHT = "#DDEFE4"
ORANGE = "#D58A2A"
ORANGE_LIGHT = "#F4DFBD"
GRAY = "#4A4A4A"
GRAY_LIGHT = "#E8E8E8"
BLACK = "#1A1A1A"
RED = "#B83A3A"
PURPLE = "#6B5EA8"


def setup_canvas(width=3.3, height=2.3):
    fig, ax = plt.subplots(figsize=(width, height))
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_aspect("equal")
    ax.axis("off")
    return fig, ax


def save(fig, name):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "svg", "png"):
        kwargs = {"bbox_inches": "tight", "pad_inches": 0.03}
        if ext == "png":
            kwargs["dpi"] = 600
        fig.savefig(OUT_DIR / f"{name}.{ext}", **kwargs)
    plt.close(fig)


def arrow(ax, start, end, color=BLACK, lw=1.2, ms=9, style="-|>", alpha=1.0, zorder=5):
    arr = FancyArrowPatch(
        start,
        end,
        arrowstyle=style,
        mutation_scale=ms,
        linewidth=lw,
        color=color,
        shrinkA=0,
        shrinkB=0,
        alpha=alpha,
        zorder=zorder,
    )
    ax.add_patch(arr)
    return arr


def label(ax, xy, text, size=8.5, color=BLACK, ha="center", va="center", weight="normal"):
    ax.text(*xy, text, fontsize=size, color=color, ha=ha, va=va, weight=weight)


def draw_curved_patch(ax, x0, y0, w, h, color=BLUE, fill=BLUE_LIGHT, mesh=False, tri=False, alpha=0.85):
    def tr(u, v):
        x = x0 + w * (u + 0.22 * v)
        y = y0 + h * (0.18 * u + v + 0.08 * math.sin(2 * math.pi * u) * math.sin(math.pi * v))
        return x, y

    boundary = np.array([tr(0, 0), tr(1, 0), tr(1, 1), tr(0, 1)])
    ax.add_patch(Polygon(boundary, closed=True, facecolor=fill, edgecolor=color, lw=1.2, alpha=alpha))

    for s in np.linspace(0.18, 0.82, 4):
        pts = np.array([tr(t, s) for t in np.linspace(0, 1, 80)])
        ax.plot(pts[:, 0], pts[:, 1], color=color, lw=0.55, alpha=0.75)
        pts = np.array([tr(s, t) for t in np.linspace(0, 1, 80)])
        ax.plot(pts[:, 0], pts[:, 1], color=color, lw=0.55, alpha=0.75)

    if mesh:
        grid = np.linspace(0, 1, 5)
        for u in grid:
            pts = np.array([tr(u, v) for v in grid])
            ax.plot(pts[:, 0], pts[:, 1], color=color, lw=0.45, alpha=0.85)
        for v in grid:
            pts = np.array([tr(u, v) for u in grid])
            ax.plot(pts[:, 0], pts[:, 1], color=color, lw=0.45, alpha=0.85)
        if tri:
            for i in range(4):
                for j in range(4):
                    p1 = tr(grid[i], grid[j])
                    p2 = tr(grid[i + 1], grid[j + 1])
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color, lw=0.35, alpha=0.75)

    return tr


def draw_cad_patch(ax, x0, y0, w, h):
    verts = np.array(
        [
        (x0 + 0.05 * w, y0 + 0.15 * h),
        (x0 + 0.38 * w, y0 + 0.02 * h),
        (x0 + 0.92 * w, y0 + 0.22 * h),
        (x0 + 0.84 * w, y0 + 0.82 * h),
        (x0 + 0.35 * w, y0 + 0.97 * h),
        (x0 + 0.00 * w, y0 + 0.62 * h),
        ]
    )
    ax.add_patch(Polygon(verts, closed=True, facecolor="#DCE9F7", edgecolor=BLUE, lw=1.2, alpha=0.95))
    ax.add_patch(Ellipse((x0 + 0.52 * w, y0 + 0.55 * h), 0.23 * w, 0.17 * h, angle=18, facecolor="white", edgecolor=BLUE, lw=0.9))
    for t in np.linspace(0.25, 0.75, 3):
        ax.plot([x0 + 0.15 * w, x0 + 0.85 * w], [y0 + t * h, y0 + (t + 0.08) * h], color=BLUE, lw=0.5, alpha=0.65)


def draw_rigid_mesh(ax, x0, y0, w, h):
    front = np.array(
        [
            [x0 + 0.18 * w, y0 + 0.18 * h],
            [x0 + 0.82 * w, y0 + 0.12 * h],
            [x0 + 0.82 * w, y0 + 0.62 * h],
            [x0 + 0.22 * w, y0 + 0.72 * h],
        ]
    )
    back = front + np.array([0.12 * w, 0.18 * h])
    faces = [
        (back, "#E9E9E9"),
        (np.array([front[0], front[1], back[1], back[0]]), "#DADADA"),
        (np.array([front[1], front[2], back[2], back[1]]), "#CFCFCF"),
        (np.array([front[2], front[3], back[3], back[2]]), "#E1E1E1"),
        (front, "#F2F2F2"),
    ]
    for pts, fc in faces:
        ax.add_patch(Polygon(pts, closed=True, facecolor=fc, edgecolor=GRAY, lw=0.9))
    for pts in (front, back):
        ax.plot([pts[0, 0], pts[2, 0]], [pts[0, 1], pts[2, 1]], color=GRAY, lw=0.45)
        ax.plot([pts[1, 0], pts[3, 0]], [pts[1, 1], pts[3, 1]], color=GRAY, lw=0.45)


def draw_provider():
    fig, ax = setup_canvas(6.6, 1.75)
    positions = [(0.08, 0.35), (0.31, 0.35), (0.54, 0.35), (0.77, 0.35)]
    labels = ["Analytic\npatch", "CAD\npatch", "Mesh\npatch", "Rigid\nmesh"]

    draw_curved_patch(ax, positions[0][0], positions[0][1], 0.13, 0.26, color=BLUE, fill=BLUE_LIGHT)
    draw_cad_patch(ax, positions[1][0], positions[1][1], 0.14, 0.26)
    draw_curved_patch(ax, positions[2][0], positions[2][1], 0.13, 0.26, color=GREEN, fill=GREEN_LIGHT, mesh=True, tri=True)
    draw_rigid_mesh(ax, positions[3][0], positions[3][1], 0.14, 0.28)

    for (x, _), txt in zip(positions, labels):
        label(ax, (x + 0.07, 0.17), txt, size=7.0)
    label(ax, (0.5, 0.86), "Contact element provider", size=10, weight="bold")
    save(fig, "01_contact_element_provider")


def draw_curved_jets_bvh():
    fig, ax = setup_canvas(3.4, 2.3)
    tr = draw_curved_patch(ax, 0.18, 0.28, 0.48, 0.38, color=BLUE, fill=BLUE_LIGHT, mesh=True, tri=False)
    pts = [tr(0.25, 0.35), tr(0.52, 0.52), tr(0.74, 0.38)]
    for p in pts:
        arrow(ax, p, (p[0] + 0.02, p[1] + 0.15), color=BLUE, lw=1.1, ms=8)
        cone = Polygon(
            [(p[0], p[1]), (p[0] - 0.045, p[1] + 0.11), (p[0] + 0.085, p[1] + 0.12)],
            closed=True,
            facecolor=BLUE_LIGHT,
            edgecolor=BLUE,
            lw=0.45,
            alpha=0.35,
        )
        ax.add_patch(cone)
    ax.add_patch(Rectangle((0.12, 0.22), 0.72, 0.58, facecolor="none", edgecolor=ORANGE, lw=1.2, linestyle=(0, (4, 2))))
    ax.add_patch(Rectangle((0.16, 0.26), 0.61, 0.48, facecolor="none", edgecolor=GRAY, lw=0.8, alpha=0.7))
    label(ax, (0.48, 0.89), "Curved jets + inflated BVH", size=10, weight="bold")
    label(ax, (0.83, 0.72), "inflated\nbox", size=7.5, color=ORANGE, ha="left")
    label(ax, (0.29, 0.75), "normal\ncones", size=7.5, color=BLUE)
    save(fig, "02_curved_jets_inflated_bvh")


def draw_candidate_pair():
    fig, ax = setup_canvas(3.2, 2.15)
    tr_a = draw_curved_patch(ax, 0.17, 0.18, 0.45, 0.30, color=BLUE, fill=BLUE_LIGHT, mesh=True)
    tr_b = draw_curved_patch(ax, 0.42, 0.49, 0.38, 0.26, color=GREEN, fill=GREEN_LIGHT, mesh=True)
    p_a = tr_a(0.70, 0.70)
    p_b = tr_b(0.23, 0.20)
    ax.plot([p_a[0], p_b[0]], [p_a[1], p_b[1]], color=RED, lw=1.0, linestyle=(0, (3, 2)))
    ax.add_patch(Circle(p_a, 0.012, facecolor=RED, edgecolor="white", lw=0.4, zorder=8))
    ax.add_patch(Circle(p_b, 0.012, facecolor=RED, edgecolor="white", lw=0.4, zorder=8))
    label(ax, (p_a[0] - 0.03, p_a[1] - 0.05), r"$x_A^0$", size=8.5, color=RED)
    label(ax, (p_b[0] + 0.04, p_b[1] + 0.04), r"$x_B^0$", size=8.5, color=RED)
    label(ax, (0.50, 0.90), "Candidate primitive pair", size=10, weight="bold")
    save(fig, "03_candidate_primitive_pair")


def draw_graph_gap_solve():
    fig, ax = setup_canvas(3.6, 2.15)
    x = np.linspace(0.12, 0.88, 250)
    h_a = 0.33 + 0.045 * np.sin(2 * np.pi * (x - 0.12) / 0.76)
    h_b = 0.60 + 0.055 * np.sin(2 * np.pi * (x - 0.12) / 0.76 + 0.55)
    ax.fill_between(x, h_a - 0.015, h_a + 0.015, color=BLUE_LIGHT, alpha=0.9)
    ax.plot(x, h_a, color=BLUE, lw=1.5)
    ax.fill_between(x, h_b - 0.015, h_b + 0.015, color=GREEN_LIGHT, alpha=0.9)
    ax.plot(x, h_b, color=GREEN, lw=1.5)
    idx = np.argmin(h_b - h_a)
    xi = x[idx]
    ax.plot([xi, xi], [h_a[idx], h_b[idx]], color=BLACK, lw=1.0)
    arrow(ax, (xi + 0.045, h_a[idx] + 0.02), (xi + 0.045, h_b[idx] - 0.02), color=BLACK, lw=0.9, ms=7)
    arrow(ax, (xi + 0.045, h_b[idx] - 0.02), (xi + 0.045, h_a[idx] + 0.02), color=BLACK, lw=0.9, ms=7)
    ax.plot([0.10, 0.90], [0.20, 0.20], color=GRAY, lw=0.8)
    label(ax, (0.50, 0.90), "2D local graph gap solve", size=10, weight="bold")
    label(ax, (0.23, 0.38), r"$h_A(\xi)$", color=BLUE, size=8.5)
    label(ax, (0.25, 0.67), r"$h_B(\xi)$", color=GREEN, size=8.5)
    label(ax, (xi + 0.105, 0.47), r"$g_{AB}(\xi^*)$", color=BLACK, size=8.5, ha="left")
    label(ax, (0.50, 0.14), "contact plane", color=GRAY, size=8)
    save(fig, "04_2d_graph_gap_solve")


def draw_patch_patch_fallback():
    fig, ax = setup_canvas(3.4, 2.15)
    tr_a = draw_curved_patch(ax, 0.12, 0.20, 0.48, 0.34, color=ORANGE, fill=ORANGE_LIGHT, mesh=True)
    tr_b = draw_curved_patch(ax, 0.44, 0.42, 0.40, 0.34, color=PURPLE, fill="#E7E3F3", mesh=True)
    p = tr_a(0.82, 0.76)
    q = tr_b(0.18, 0.22)
    ax.plot([p[0], q[0]], [p[1], q[1]], color=RED, lw=1.1, linestyle=(0, (3, 2)))
    ax.add_patch(Circle(p, 0.012, facecolor=RED, edgecolor="white", lw=0.4, zorder=8))
    ax.add_patch(Circle(q, 0.012, facecolor=RED, edgecolor="white", lw=0.4, zorder=8))
    ax.add_patch(Arc((0.51, 0.57), 0.36, 0.25, theta1=195, theta2=330, color=ORANGE, lw=1.2))
    arrow(ax, (0.67, 0.50), (0.70, 0.48), color=ORANGE, lw=1.0, ms=7)
    label(ax, (0.50, 0.90), "Curved patch-patch fallback", size=10, weight="bold")
    label(ax, (0.50, 0.20), r"$\min_{p,q}\,\|X_A(p)-X_B(q)\|^2$", size=7.4)
    label(ax, (p[0] - 0.03, p[1] - 0.04), r"$p$", color=RED, size=8.5)
    label(ax, (q[0] + 0.035, q[1] + 0.035), r"$q$", color=RED, size=8.5)
    label(ax, (0.31, 0.74), "non-graphable\nor uncertain", color=ORANGE, size=7.5)
    save(fig, "05_curved_patch_patch_fallback")


def draw_se3_response_field():
    fig, ax = setup_canvas(3.7, 2.35)
    origin = np.array([0.24, 0.27])
    ex = np.array([0.42, 0.02])
    ey = np.array([0.10, 0.34])
    ez = np.array([-0.13, 0.15])
    nodes = []
    for i in range(4):
        for j in range(4):
            for k in range(2):
                p = origin + ex * (i / 3) + ey * (j / 3) + ez * k
                nodes.append(p)
                ax.add_patch(Circle(p, 0.0075, facecolor=BLUE if k == 0 else GREEN, edgecolor="white", lw=0.25, alpha=0.85))
    edges = [
        (origin, origin + ex),
        (origin, origin + ey),
        (origin, origin + ez),
        (origin + ex, origin + ex + ey),
        (origin + ey, origin + ex + ey),
        (origin + ez, origin + ex + ez),
        (origin + ez, origin + ey + ez),
        (origin + ex + ez, origin + ex + ey + ez),
        (origin + ey + ez, origin + ex + ey + ez),
        (origin + ex + ey, origin + ex + ey + ez),
    ]
    for a, b in edges:
        ax.plot([a[0], b[0]], [a[1], b[1]], color=GRAY, lw=0.65, alpha=0.55)
    q = origin + 0.58 * ex + 0.47 * ey + 0.55 * ez
    ax.add_patch(Circle(q, 0.018, facecolor=BLACK, edgecolor="white", lw=0.5, zorder=8))
    arrow(ax, (0.78, 0.43), (0.62, 0.49), color=BLACK, lw=1.0, ms=8)
    label(ax, (0.50, 0.91), r"Local $SE(3)$ signed-gap response field", size=10, weight="bold")
    label(ax, (0.77, 0.48), r"$q=(x,\theta)\in SE(3)$", size=8.5, ha="left")
    label(ax, (0.57, 0.18), "tensor-product\ncubic interpolation", size=8.0, color=BLUE)
    label(ax, (0.36, 0.76), r"samples $\phi_{\mathbf{i}}$", size=8.0, color=GRAY)
    save(fig, "06_se3_signed_gap_response_field")


def draw_friction_response():
    fig, ax = setup_canvas(3.4, 2.15)
    x = np.linspace(0.12, 0.88, 200)
    y = 0.35 + 0.045 * np.sin(2 * np.pi * (x - 0.12) / 0.76)
    ax.fill_between(x, 0.12, y, color=BLUE_LIGHT, alpha=0.8)
    ax.plot(x, y, color=BLUE, lw=1.5)
    body = Polygon(
        [(0.43, 0.56), (0.65, 0.60), (0.68, 0.76), (0.46, 0.72)],
        closed=True,
        facecolor="#EFEFEF",
        edgecolor=BLACK,
        lw=1.0,
    )
    ax.add_patch(body)
    contact = (0.55, 0.39)
    ax.add_patch(Circle(contact, 0.012, facecolor=BLACK, edgecolor="white", lw=0.35, zorder=8))
    arrow(ax, contact, (0.55, 0.56), color=BLACK, lw=1.1, ms=8)
    arrow(ax, contact, (0.72, 0.43), color=ORANGE, lw=1.3, ms=9)
    arrow(ax, (0.34, 0.62), (0.25, 0.58), color=GREEN, lw=1.3, ms=9)
    xx = np.linspace(0.20, 0.76, 90)
    yy = 0.825 + 0.018 * np.sin(2 * np.pi * (xx - 0.20) / 0.56)
    ax.plot(xx, yy, color=BLACK, lw=1.0)
    arrow(ax, (0.73, 0.825), (0.76, 0.825), color=BLACK, lw=1.0, ms=7)
    label(ax, (0.50, 0.94), "Frictional rigid-body response", size=10, weight="bold")
    label(ax, (0.59, 0.50), r"$\tilde n$", size=8.5, ha="left")
    label(ax, (0.74, 0.46), r"$F_t$", color=ORANGE, size=8.5, ha="left")
    label(ax, (0.25, 0.52), r"$\phi(q)$", color=GREEN, size=8.5)
    label(ax, (0.51, 0.18), "smooth force response", size=8.0, color=BLACK)
    save(fig, "07_frictional_rigid_body_response")


def main():
    draw_provider()
    draw_curved_jets_bvh()
    draw_candidate_pair()
    draw_graph_gap_solve()
    draw_patch_patch_fallback()
    draw_se3_response_field()
    draw_friction_response()
    print(f"Wrote method pipeline component figures to {OUT_DIR}")


if __name__ == "__main__":
    main()
