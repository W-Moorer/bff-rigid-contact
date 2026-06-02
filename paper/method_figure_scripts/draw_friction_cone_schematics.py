from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Polygon
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
BLUE_LIGHT = "#CFE0F2"
GREEN = "#3C8D5A"
GREEN_LIGHT = "#DDEFE4"
ORANGE = "#D58A2A"
GRAY = "#4A4A4A"
BLACK = "#1A1A1A"


def normalize(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


VIEW_DIR = normalize(np.array([0.72, -1.0, 1.30]))
SCREEN_X = normalize(np.cross(np.array([0.0, 0.0, 1.0]), VIEW_DIR))
SCREEN_Y = normalize(np.cross(VIEW_DIR, SCREEN_X))


def project(points):
    pts = np.asarray(points, dtype=float)
    return np.column_stack([pts @ SCREEN_X, pts @ SCREEN_Y])


def save(fig, name):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for ext in ("pdf", "svg", "png"):
        kwargs = {"bbox_inches": "tight", "pad_inches": 0.02}
        if ext == "png":
            kwargs["dpi"] = 600
        fig.savefig(OUT_DIR / f"{name}.{ext}", **kwargs)
    plt.close(fig)


def setup_canvas(projected_points):
    fig, ax = plt.subplots(figsize=(3.2, 2.35))
    ax.set_aspect("equal")
    ax.axis("off")
    pts = np.vstack(projected_points)
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    center = 0.5 * (mins + maxs)
    span = max(maxs - mins)
    pad = 0.08 * span
    ax.set_xlim(center[0] - 0.5 * span - pad, center[0] + 0.5 * span + pad)
    ax.set_ylim(center[1] - 0.5 * span - pad, center[1] + 0.5 * span + pad)
    fig.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)
    return fig, ax


def triangle_grid(n=16):
    pts = []
    ids = {}
    for i in range(n + 1):
        for j in range(n + 1 - i):
            ids[(i, j)] = len(pts)
            pts.append((i / n, j / n))

    faces = []
    for i in range(n):
        for j in range(n - i):
            faces.append([ids[(i, j)], ids[(i + 1, j)], ids[(i, j + 1)]])
            if j < n - i - 1:
                faces.append([ids[(i + 1, j)], ids[(i + 1, j + 1)], ids[(i, j + 1)]])
    return np.asarray(pts), np.asarray(faces)


def curved_height(x, y):
    return 0.12 + 0.18 * x * x - 0.07 * y + 0.055 * np.sin(2.6 * x + 1.4 * y)


def curved_grad(x, y):
    common = 0.055 * np.cos(2.6 * x + 1.4 * y)
    return np.array([0.36 * x + 2.6 * common, -0.07 + 1.4 * common])


CURVED_V0 = np.array([-0.78, -0.54])
CURVED_V1 = np.array([0.82, -0.36])
CURVED_V2 = np.array([-0.25, 0.84])


def curved_point_from_bary(a, b):
    xy = (1.0 - a - b) * CURVED_V0 + a * CURVED_V1 + b * CURVED_V2
    return np.array([xy[0], xy[1], curved_height(xy[0], xy[1])])


def curved_patch_boundary(n=72):
    edge_01 = [curved_point_from_bary(a, 0.0) for a in np.linspace(0.0, 1.0, n, endpoint=False)]
    edge_12 = [curved_point_from_bary(1.0 - s, s) for s in np.linspace(0.0, 1.0, n, endpoint=False)]
    edge_20 = [curved_point_from_bary(0.0, 1.0 - s) for s in np.linspace(0.0, 1.0, n, endpoint=False)]
    return np.asarray(edge_01 + edge_12 + edge_20)


def make_curved_patch():
    bary, faces = triangle_grid()
    xy = (
        (1.0 - bary[:, :1] - bary[:, 1:2]) * CURVED_V0
        + bary[:, :1] * CURVED_V1
        + bary[:, 1:2] * CURVED_V2
    )
    z = curved_height(xy[:, 0], xy[:, 1])
    pts = np.column_stack([xy, z])

    cp_bary = np.array([0.34, 0.30])
    cp_xy = (
        (1.0 - cp_bary[0] - cp_bary[1]) * CURVED_V0
        + cp_bary[0] * CURVED_V1
        + cp_bary[1] * CURVED_V2
    )
    cp = np.array([cp_xy[0], cp_xy[1], curved_height(cp_xy[0], cp_xy[1])])
    grad = curved_grad(cp_xy[0], cp_xy[1])
    normal = normalize(np.array([-grad[0], -grad[1], 1.0]))
    return pts, faces, cp, normal


def make_planar_patch():
    tri = np.array(
        [
            [-0.82, -0.50, 0.06],
            [0.78, -0.42, -0.01],
            [-0.20, 0.82, 0.33],
        ]
    )
    normal = normalize(np.cross(tri[1] - tri[0], tri[2] - tri[0]))
    if normal[2] < 0:
        normal *= -1.0
    cp = 0.36 * tri[0] + 0.34 * tri[1] + 0.30 * tri[2]
    return tri, cp, normal


def aabb_geometry(pts):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    corners = np.array(
        [
            [mins[0], mins[1], mins[2]],
            [maxs[0], mins[1], mins[2]],
            [maxs[0], maxs[1], mins[2]],
            [mins[0], maxs[1], mins[2]],
            [mins[0], mins[1], maxs[2]],
            [maxs[0], mins[1], maxs[2]],
            [maxs[0], maxs[1], maxs[2]],
            [mins[0], maxs[1], maxs[2]],
        ]
    )
    edges = [
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
    return corners, edges


def tangent_basis(normal):
    ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(ref, normal)) > 0.92:
        ref = np.array([1.0, 0.0, 0.0])
    t1 = normalize(np.cross(normal, ref))
    t2 = normalize(np.cross(normal, t1))
    return t1, t2


def cone_geometry(apex, normal, mu=0.45, height=0.48, n_theta=80):
    t1, t2 = tangent_basis(normal)
    theta = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=True)
    base = apex + height * normal + mu * height * (
        np.cos(theta)[:, None] * t1 + np.sin(theta)[:, None] * t2
    )
    rings = []
    for s in np.linspace(0.18, 1.0, 7):
        rings.append(apex + s * height * normal + s * mu * height * (
            np.cos(theta)[:, None] * t1 + np.sin(theta)[:, None] * t2
        ))
    meridians = []
    for angle in np.linspace(0.0, 2.0 * np.pi, 10, endpoint=False):
        end = apex + height * normal + mu * height * (np.cos(angle) * t1 + np.sin(angle) * t2)
        meridians.append(np.vstack([apex, end]))
    return base, rings, meridians, t1


def draw_aabb(ax, corners, edges):
    pc = project(corners)
    for a, b in edges:
        ax.plot(
            [pc[a, 0], pc[b, 0]],
            [pc[a, 1], pc[b, 1]],
            color=ORANGE,
            lw=1.15,
            ls=(0, (4, 2)),
            solid_capstyle="round",
            zorder=5,
        )


def draw_curved_triangle_patch(ax, boundary):
    poly = Polygon(
        project(boundary),
        closed=True,
        facecolor=BLUE_LIGHT,
        edgecolor=BLUE,
        lw=1.4,
        alpha=0.88,
        joinstyle="round",
        zorder=3,
    )
    ax.add_patch(poly)


def draw_planar_triangle(ax, tri):
    poly = Polygon(
        project(tri),
        closed=True,
        facecolor=BLUE_LIGHT,
        edgecolor=BLUE,
        lw=1.4,
        alpha=0.88,
        joinstyle="round",
        zorder=3,
    )
    ax.add_patch(poly)


def draw_arrow(ax, p0, p1, color, lw=1.3, zorder=8):
    q0, q1 = project(np.vstack([p0, p1]))
    arr = FancyArrowPatch(
        q0,
        q1,
        arrowstyle="-|>",
        mutation_scale=9,
        linewidth=lw,
        color=color,
        shrinkA=0,
        shrinkB=0,
        zorder=zorder,
    )
    ax.add_patch(arr)


def draw_cone(ax, apex, normal):
    base, rings, meridians, tangent = cone_geometry(apex, normal)
    apex_2d = project(apex[None, :])[0]
    base_2d = project(base)
    for i in range(len(base_2d) - 1):
        tri = Polygon(
            np.vstack([apex_2d, base_2d[i], base_2d[i + 1]]),
            closed=True,
            facecolor=GREEN_LIGHT,
            edgecolor="none",
            alpha=0.18,
            zorder=6,
        )
        ax.add_patch(tri)
    for ring in rings:
        rr = project(ring)
        ax.plot(rr[:, 0], rr[:, 1], color=GREEN, lw=0.55, alpha=0.92, zorder=7)
    for meridian in meridians:
        mm = project(meridian)
        ax.plot(mm[:, 0], mm[:, 1], color=GREEN, lw=0.65, alpha=0.88, zorder=7)
    ax.plot(base_2d[:, 0], base_2d[:, 1], color=GREEN, lw=1.05, zorder=8)
    draw_arrow(ax, apex, apex + 0.48 * normal, GREEN, lw=1.35, zorder=9)
    draw_arrow(ax, apex, apex + 0.30 * tangent, GRAY, lw=1.05, zorder=9)
    return [base, *rings, *meridians]


def draw_contact_point(ax, cp):
    q = project(cp[None, :])[0]
    ax.add_patch(Circle(q, radius=0.026, facecolor=BLACK, edgecolor="white", lw=0.45, zorder=10))


def draw_curved_triangle_friction_cone():
    pts, faces, cp, normal = make_curved_patch()
    boundary = curved_patch_boundary()
    box, edges = aabb_geometry(pts)
    cone_parts = cone_geometry(cp + 0.008 * normal, normal)[:3]
    projected_extent = [project(boundary), project(box), project(cp[None, :])]
    projected_extent.extend(project(part) for part in cone_parts[1])
    projected_extent.append(project(cone_parts[0]))
    fig, ax = setup_canvas(projected_extent)
    draw_curved_triangle_patch(ax, boundary)
    draw_aabb(ax, box, edges)
    draw_cone(ax, cp + 0.008 * normal, normal)
    draw_contact_point(ax, cp)
    save(fig, "08_curved_triangle_patch_friction_cone")


def draw_planar_triangle_friction_cone():
    tri, cp, normal = make_planar_patch()
    box, edges = aabb_geometry(tri)
    cone_parts = cone_geometry(cp + 0.008 * normal, normal)[:3]
    projected_extent = [project(tri), project(box), project(cp[None, :])]
    projected_extent.extend(project(part) for part in cone_parts[1])
    projected_extent.append(project(cone_parts[0]))
    fig, ax = setup_canvas(projected_extent)
    draw_planar_triangle(ax, tri)
    draw_aabb(ax, box, edges)
    draw_cone(ax, cp + 0.008 * normal, normal)
    draw_contact_point(ax, cp)
    save(fig, "09_planar_triangle_patch_friction_cone")


def main():
    draw_curved_triangle_friction_cone()
    draw_planar_triangle_friction_cone()


if __name__ == "__main__":
    main()
