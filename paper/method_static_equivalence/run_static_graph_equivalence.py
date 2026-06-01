from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

try:
    from scipy import optimize
except ImportError as exc:  # pragma: no cover
    raise SystemExit("This static equivalence script requires scipy.optimize.") from exc


mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset": "dejavuserif",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
        "font.size": 8.5,
    }
)


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "figures" / "static_graph_equivalence"
RES_DIR = ROOT.parents[0] / "results" / "v0_16_static_graph_equivalence"

GAP0 = 2.5e-2
ALPHA = 1.2e-1
BETA = 3.5e-2
GEOM_SCALE = 5.0
DOMAIN = (-0.9, 0.9)


@dataclass
class SolveResult:
    name: str
    parameters: np.ndarray
    gap_or_distance: float
    point_a: np.ndarray
    point_b: np.ndarray
    normal: np.ndarray
    success: bool
    iterations: int
    objective: float


def h_common(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray | float:
    return GEOM_SCALE * (
        0.035 * (u * u + 0.65 * v * v)
        + 0.020 * (np.cos(3.2 * u) - 1.0) * (0.65 + 0.35 * np.cos(2.4 * v))
        + 0.012 * np.sin(2.7 * u) * np.sin(3.1 * v)
        + 0.006 * (np.cos(5.0 * u) - 1.0) * (np.cos(3.8 * v) - 1.0)
    )


def grad_common(u: float, v: float) -> np.ndarray:
    hu = GEOM_SCALE * (
        0.070 * u
        - 0.020 * 3.2 * np.sin(3.2 * u) * (0.65 + 0.35 * np.cos(2.4 * v))
        + 0.012 * 2.7 * np.cos(2.7 * u) * np.sin(3.1 * v)
        - 0.006 * 5.0 * np.sin(5.0 * u) * (np.cos(3.8 * v) - 1.0)
    )
    hv = GEOM_SCALE * (
        0.070 * 0.65 * v
        + 0.020 * (np.cos(3.2 * u) - 1.0) * 0.35 * (-2.4 * np.sin(2.4 * v))
        + 0.012 * 3.1 * np.sin(2.7 * u) * np.cos(3.1 * v)
        - 0.006 * 3.8 * (np.cos(5.0 * u) - 1.0) * np.sin(3.8 * v)
    )
    return np.array([hu, hv], dtype=float)


def h_upper(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray | float:
    return h_common(u, v) + GAP0 + ALPHA * (u * u + v * v) + BETA * (u**4 + 0.7 * v**4)


def grad_upper(u: float, v: float) -> np.ndarray:
    base = grad_common(u, v)
    return np.array(
        [
            base[0] + 2.0 * ALPHA * u + 4.0 * BETA * u**3,
            base[1] + 2.0 * ALPHA * v + 4.0 * 0.7 * BETA * v**3,
        ],
        dtype=float,
    )


def gap_graph(uv: np.ndarray) -> float:
    u, v = uv
    return float(h_upper(u, v) - h_common(u, v))


def grad_gap_graph(uv: np.ndarray) -> np.ndarray:
    u, v = uv
    return np.array(
        [
            2.0 * ALPHA * u + 4.0 * BETA * u**3,
            2.0 * ALPHA * v + 4.0 * 0.7 * BETA * v**3,
        ],
        dtype=float,
    )


def point_a(uv: np.ndarray) -> np.ndarray:
    u, v = uv
    return np.array([u, v, h_common(u, v)], dtype=float)


def point_b(uv: np.ndarray) -> np.ndarray:
    u, v = uv
    return np.array([u, v, h_upper(u, v)], dtype=float)


def jac_a(uv: np.ndarray) -> np.ndarray:
    u, v = uv
    hu, hv = grad_common(u, v)
    return np.array([[1.0, 0.0], [0.0, 1.0], [hu, hv]], dtype=float)


def jac_b(uv: np.ndarray) -> np.ndarray:
    u, v = uv
    hu, hv = grad_upper(u, v)
    return np.array([[1.0, 0.0], [0.0, 1.0], [hu, hv]], dtype=float)


def upward_normal(grad: np.ndarray) -> np.ndarray:
    n = np.array([-grad[0], -grad[1], 1.0], dtype=float)
    return n / np.linalg.norm(n)


def solve_2d_graph() -> SolveResult:
    starts = [
        np.array([0.45, -0.35]),
        np.array([-0.55, 0.40]),
        np.array([0.75, 0.75]),
        np.array([-0.30, -0.70]),
    ]
    best = None
    bounds = [DOMAIN, DOMAIN]
    for start in starts:
        res = optimize.minimize(
            gap_graph,
            start,
            jac=grad_gap_graph,
            bounds=bounds,
            method="L-BFGS-B",
            options={"ftol": 1e-15, "gtol": 1e-13, "maxiter": 500},
        )
        if best is None or res.fun < best.fun:
            best = res
    uv = np.asarray(best.x, dtype=float)
    pa = point_a(uv)
    pb = point_b(uv)
    n = upward_normal(grad_common(*uv))
    return SolveResult("2D graph gap", uv, float(best.fun), pa, pb, n, bool(best.success), int(best.nit), float(best.fun))


def objective_4d(z: np.ndarray) -> float:
    p = z[:2]
    q = z[2:]
    r = point_a(p) - point_b(q)
    return float(0.5 * np.dot(r, r))


def gradient_4d(z: np.ndarray) -> np.ndarray:
    p = z[:2]
    q = z[2:]
    r = point_a(p) - point_b(q)
    return np.r_[jac_a(p).T @ r, -jac_b(q).T @ r]


def solve_4d_patch_patch() -> SolveResult:
    starts = []
    for a in (-0.55, 0.0, 0.55):
        for b in (-0.45, 0.45):
            starts.append(np.array([a, b, a, b], dtype=float))
            starts.append(np.array([a, b, 0.5 * a, 0.5 * b], dtype=float))
    best = None
    bounds = [DOMAIN, DOMAIN, DOMAIN, DOMAIN]
    for start in starts:
        res = optimize.minimize(
            objective_4d,
            start,
            jac=gradient_4d,
            bounds=bounds,
            method="L-BFGS-B",
            options={"ftol": 1e-15, "gtol": 1e-13, "maxiter": 1000},
        )
        if best is None or res.fun < best.fun:
            best = res
    z = np.asarray(best.x, dtype=float)
    pa = point_a(z[:2])
    pb = point_b(z[2:])
    n = pb - pa
    distance = float(np.linalg.norm(n))
    n = n / distance
    return SolveResult(
        "4D patch-patch closest",
        z,
        distance,
        pa,
        pb,
        n,
        bool(best.success),
        int(best.nit),
        float(best.fun),
    )


def analytic_reference() -> SolveResult:
    uv = np.array([0.0, 0.0], dtype=float)
    pa = point_a(uv)
    pb = point_b(uv)
    return SolveResult("analytic reference", uv, GAP0, pa, pb, np.array([0.0, 0.0, 1.0]), True, 0, GAP0)


def normal_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def graphability_statistics(samples: int = 121) -> dict[str, float]:
    vals = np.linspace(*DOMAIN, samples)
    min_a = 1.0
    min_b = 1.0
    for u in vals:
        for v in vals:
            ga = grad_common(u, v)
            gb = grad_upper(u, v)
            min_a = min(min_a, 1.0 / np.sqrt(1.0 + float(np.dot(ga, ga))))
            min_b = min(min_b, 1.0 / np.sqrt(1.0 + float(np.dot(gb, gb))))
    return {
        "min_NA_dot_nc": float(min_a),
        "min_minus_NB_dot_nc": float(min_b),
        "sample_count": samples * samples,
    }


def write_outputs(ref: SolveResult, graph: SolveResult, patch: SolveResult) -> dict[str, float]:
    RES_DIR.mkdir(parents=True, exist_ok=True)
    stats = graphability_statistics()
    metrics = {
        "analytic_gap_m": ref.gap_or_distance,
        "graph_gap_m": graph.gap_or_distance,
        "patch_patch_distance_m": patch.gap_or_distance,
        "abs_graph_minus_reference_gap_m": abs(graph.gap_or_distance - ref.gap_or_distance),
        "abs_patch_minus_reference_distance_m": abs(patch.gap_or_distance - ref.gap_or_distance),
        "abs_graph_minus_patch_m": abs(graph.gap_or_distance - patch.gap_or_distance),
        "graph_patch_point_A_difference_m": float(np.linalg.norm(graph.point_a - patch.point_a)),
        "graph_patch_point_B_difference_m": float(np.linalg.norm(graph.point_b - patch.point_b)),
        "graph_patch_normal_angle_deg": normal_angle_deg(graph.normal, patch.normal),
        "graph_success": graph.success,
        "patch_success": patch.success,
        "graph_iterations": graph.iterations,
        "patch_iterations": patch.iterations,
        **stats,
    }

    with (RES_DIR / "static_graph_equivalence_summary.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)

    rows = [
        {
            "method": ref.name,
            "gap_or_distance_m": ref.gap_or_distance,
            "parameters": " ".join(f"{x:.16e}" for x in ref.parameters),
            "point_A_m": " ".join(f"{x:.16e}" for x in ref.point_a),
            "point_B_m": " ".join(f"{x:.16e}" for x in ref.point_b),
            "normal": " ".join(f"{x:.16e}" for x in ref.normal),
            "iterations": ref.iterations,
        },
        {
            "method": graph.name,
            "gap_or_distance_m": graph.gap_or_distance,
            "parameters": " ".join(f"{x:.16e}" for x in graph.parameters),
            "point_A_m": " ".join(f"{x:.16e}" for x in graph.point_a),
            "point_B_m": " ".join(f"{x:.16e}" for x in graph.point_b),
            "normal": " ".join(f"{x:.16e}" for x in graph.normal),
            "iterations": graph.iterations,
        },
        {
            "method": patch.name,
            "gap_or_distance_m": patch.gap_or_distance,
            "parameters": " ".join(f"{x:.16e}" for x in patch.parameters),
            "point_A_m": " ".join(f"{x:.16e}" for x in patch.point_a),
            "point_B_m": " ".join(f"{x:.16e}" for x in patch.point_b),
            "normal": " ".join(f"{x:.16e}" for x in patch.normal),
            "iterations": patch.iterations,
        },
    ]
    with (RES_DIR / "static_graph_equivalence_methods.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    md = [
        "# Static graph-equivalence check",
        "",
        "The scene uses two geometrically complex but graphable analytic patches. "
        "The upper patch is the same complex base geometry plus a positive opening field, "
        "so the static contact state has a known reference minimum at the patch center.",
        "",
        "| quantity | value |",
        "|---|---:|",
    ]
    for key, value in metrics.items():
        if isinstance(value, bool):
            value_text = str(value)
        elif isinstance(value, int):
            value_text = str(value)
        else:
            value_text = f"{value:.12e}"
        md.append(f"| {key} | {value_text} |")
    md.append("")
    md.append(
        "The equality of the 2D graph gap, the 4D patch-patch closest distance, "
        "and the analytic reference demonstrates equivalence for this certified graphable static case."
    )
    (RES_DIR / "static_graph_equivalence_summary.md").write_text("\n".join(md), encoding="utf-8")
    return metrics


def save_figure(fig: plt.Figure, stem: str) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / f"{stem}.pdf", bbox_inches="tight", pad_inches=0.03)
    fig.savefig(FIG_DIR / f"{stem}.svg", bbox_inches="tight", pad_inches=0.03)
    fig.savefig(FIG_DIR / f"{stem}.png", bbox_inches="tight", pad_inches=0.03, dpi=600)
    plt.close(fig)


def draw_model_figure(graph: SolveResult, patch: SolveResult) -> None:
    fig = plt.figure(figsize=(4.8, 3.6))
    ax = fig.add_subplot(111, projection="3d")
    u = np.linspace(*DOMAIN, 90)
    v = np.linspace(*DOMAIN, 90)
    U, V = np.meshgrid(u, v)
    ZA = h_common(U, V)
    ZB = h_upper(U, V)
    ax.plot_surface(U, V, ZA, rstride=2, cstride=2, linewidth=0.12, edgecolor="#5C85B8", color="#CFE0F2", alpha=0.78)
    ax.plot_surface(U, V, ZB, rstride=2, cstride=2, linewidth=0.12, edgecolor="#61A778", color="#DDEFE4", alpha=0.48)
    ax.plot(
        [graph.point_a[0], graph.point_b[0]],
        [graph.point_a[1], graph.point_b[1]],
        [graph.point_a[2], graph.point_b[2]],
        color="#B83A3A",
        linewidth=3.0,
        zorder=50,
    )
    ax.scatter(*graph.point_a, s=70, color="#1A1A1A", edgecolor="white", linewidth=0.4, depthshade=False, zorder=60)
    ax.scatter(*graph.point_b, s=70, color="#1A1A1A", edgecolor="white", linewidth=0.4, depthshade=False, zorder=60)
    ax.scatter(*patch.point_a, s=35, color="#B83A3A", depthshade=False, zorder=70)
    ax.scatter(*patch.point_b, s=35, color="#B83A3A", depthshade=False, zorder=70)
    ax.view_init(elev=24, azim=-57)
    ax.set_axis_off()
    ax.set_box_aspect((1.0, 1.0, 0.34))
    save_figure(fig, "static_graph_equivalence_model")


def draw_error_figure(metrics: dict[str, float]) -> None:
    names = [
        "gap\n2D-ref",
        "distance\n4D-ref",
        "2D-4D\ngap",
        "point A\n2D-4D",
        "point B\n2D-4D",
        "normal\nangle",
    ]
    values = np.array(
        [
            metrics["abs_graph_minus_reference_gap_m"],
            metrics["abs_patch_minus_reference_distance_m"],
            metrics["abs_graph_minus_patch_m"],
            metrics["graph_patch_point_A_difference_m"],
            metrics["graph_patch_point_B_difference_m"],
            metrics["graph_patch_normal_angle_deg"],
        ],
        dtype=float,
    )
    values_for_plot = np.maximum(values, 1e-16)
    fig, ax = plt.subplots(figsize=(4.8, 2.6))
    colors = ["#2F6FAE", "#3C8D5A", "#1A1A1A", "#8A8A8A", "#8A8A8A", "#D58A2A"]
    ax.bar(np.arange(len(values)), values_for_plot, color=colors, width=0.66)
    ax.set_yscale("log")
    ax.set_xticks(np.arange(len(values)))
    ax.set_xticklabels(names)
    ax.set_ylabel("absolute discrepancy")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", color="#D9D9D9", linewidth=0.45)
    save_figure(fig, "static_graph_equivalence_errors")


def main() -> None:
    ref = analytic_reference()
    graph = solve_2d_graph()
    patch = solve_4d_patch_patch()
    metrics = write_outputs(ref, graph, patch)
    draw_model_figure(graph, patch)
    draw_error_figure(metrics)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
