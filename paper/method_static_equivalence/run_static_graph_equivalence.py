from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np


mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
        "mathtext.fontset": "dejavuserif",
        "pdf.fonttype": 42,
        "ps.fonttype": 42,
        "svg.fonttype": "none",
        "font.size": 9.0,
        "axes.labelsize": 9.0,
        "xtick.labelsize": 8.5,
        "ytick.labelsize": 8.5,
        "legend.fontsize": 8.5,
    }
)


ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "figures" / "static_patch_contact_equivalence"
RES_DIR = ROOT.parents[0] / "results" / "v0_17_static_patch_contact_equivalence"

DOMAIN = (-0.9, 0.9)
GEOM_SCALE = 5.0
U0 = 0.16
V0 = -0.12
ELLIPSE_A = 0.30
ELLIPSE_B = 0.18
PENETRATION = 4.0e-4
CLEARANCE = 2.0e-3
NORMAL_STIFFNESS = 2.0e6


@dataclass
class RegionIntegral:
    name: str
    area: float
    int_gap: float
    mean_gap: float
    pressure_integral: float
    mean_pressure: float
    pressure_center: np.ndarray
    average_normal: np.ndarray
    radial_order: int
    theta_order: int


def h_common(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray | float:
    return GEOM_SCALE * (
        0.035 * (u * u + 0.65 * v * v)
        + 0.020 * (np.cos(3.2 * u) - 1.0) * (0.65 + 0.35 * np.cos(2.4 * v))
        + 0.012 * np.sin(2.7 * u) * np.sin(3.1 * v)
        + 0.006 * (np.cos(5.0 * u) - 1.0) * (np.cos(3.8 * v) - 1.0)
    )


def grad_common(u: np.ndarray | float, v: np.ndarray | float) -> tuple[np.ndarray | float, np.ndarray | float]:
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
    return hu, hv


def ellipse_r2(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray | float:
    return ((u - U0) / ELLIPSE_A) ** 2 + ((v - V0) / ELLIPSE_B) ** 2


def signed_gap(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray | float:
    s = ellipse_r2(u, v) - 1.0
    return np.where(s <= 0.0, -PENETRATION * s * s, CLEARANCE * s * s)


def grad_signed_gap(u: np.ndarray | float, v: np.ndarray | float) -> tuple[np.ndarray | float, np.ndarray | float]:
    s = ellipse_r2(u, v) - 1.0
    scale = np.where(s <= 0.0, -PENETRATION, CLEARANCE)
    ds_du = 2.0 * (u - U0) / (ELLIPSE_A * ELLIPSE_A)
    ds_dv = 2.0 * (v - V0) / (ELLIPSE_B * ELLIPSE_B)
    return 2.0 * scale * s * ds_du, 2.0 * scale * s * ds_dv


def h_upper(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray | float:
    return h_common(u, v) + signed_gap(u, v)


def grad_upper(u: np.ndarray | float, v: np.ndarray | float) -> tuple[np.ndarray | float, np.ndarray | float]:
    hu, hv = grad_common(u, v)
    gu, gv = grad_signed_gap(u, v)
    return hu + gu, hv + gv


def point_a(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray:
    return np.stack([u, v, h_common(u, v)], axis=-1)


def point_b(u: np.ndarray | float, v: np.ndarray | float) -> np.ndarray:
    return np.stack([u, v, h_upper(u, v)], axis=-1)


def normal_from_grad(hu: np.ndarray, hv: np.ndarray) -> np.ndarray:
    n = np.stack([-hu, -hv, np.ones_like(hu)], axis=-1)
    return n / np.linalg.norm(n, axis=-1, keepdims=True)


def active_quadrature(radial_order: int, theta_order: int, name: str) -> RegionIntegral:
    r_nodes, r_weights = np.polynomial.legendre.leggauss(radial_order)
    r = 0.5 * (r_nodes + 1.0)
    wr = 0.5 * r_weights
    theta = np.linspace(0.0, 2.0 * np.pi, theta_order, endpoint=False)
    wt = 2.0 * np.pi / theta_order

    rr, tt = np.meshgrid(r, theta, indexing="ij")
    weights = wr[:, None] * wt * ELLIPSE_A * ELLIPSE_B * rr
    u = U0 + ELLIPSE_A * rr * np.cos(tt)
    v = V0 + ELLIPSE_B * rr * np.sin(tt)

    g = signed_gap(u, v)
    pressure = NORMAL_STIFFNESS * np.maximum(0.0, -g)
    pa = point_a(u, v)
    pb = point_b(u, v)
    mid = 0.5 * (pa + pb)
    hu, hv = grad_common(u, v)
    normal = normal_from_grad(hu, hv)

    area = float(np.sum(weights))
    int_gap = float(np.sum(g * weights))
    pressure_integral = float(np.sum(pressure * weights))
    pressure_weight = pressure * weights
    if pressure_integral <= 0.0:
        pressure_center = np.array([np.nan, np.nan, np.nan], dtype=float)
        average_normal = np.array([np.nan, np.nan, np.nan], dtype=float)
    else:
        pressure_center = np.sum(mid * pressure_weight[..., None], axis=(0, 1)) / pressure_integral
        normal_integral = np.sum(normal * pressure_weight[..., None], axis=(0, 1))
        average_normal = normal_integral / np.linalg.norm(normal_integral)

    return RegionIntegral(
        name=name,
        area=area,
        int_gap=int_gap,
        mean_gap=int_gap / area,
        pressure_integral=pressure_integral,
        mean_pressure=pressure_integral / area,
        pressure_center=pressure_center,
        average_normal=average_normal,
        radial_order=radial_order,
        theta_order=theta_order,
    )


def analytic_reference() -> dict[str, float]:
    area = np.pi * ELLIPSE_A * ELLIPSE_B
    int_gap = -PENETRATION * area / 3.0
    pressure_integral = NORMAL_STIFFNESS * PENETRATION * area / 3.0
    return {
        "active_area_m2": float(area),
        "int_gap_m3": float(int_gap),
        "mean_gap_m": float(int_gap / area),
        "normal_force_N": float(pressure_integral),
        "mean_pressure_Pa": float(pressure_integral / area),
    }


def normal_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def graphability_statistics(radial_samples: int = 81, theta_samples: int = 240) -> dict[str, float]:
    radii = np.linspace(0.0, 1.05, radial_samples)
    theta = np.linspace(0.0, 2.0 * np.pi, theta_samples, endpoint=False)
    min_a = 1.0
    min_b = 1.0
    for r in radii:
        for t in theta:
            u = U0 + ELLIPSE_A * r * np.cos(t)
            v = V0 + ELLIPSE_B * r * np.sin(t)
            ha_u, ha_v = grad_common(u, v)
            hb_u, hb_v = grad_upper(u, v)
            min_a = min(min_a, 1.0 / np.sqrt(1.0 + float(ha_u * ha_u + ha_v * ha_v)))
            min_b = min(min_b, 1.0 / np.sqrt(1.0 + float(hb_u * hb_u + hb_v * hb_v)))
    return {
        "active_band_min_NA_dot_nc": float(min_a),
        "active_band_min_minus_NB_dot_nc": float(min_b),
        "active_band_sample_count": radial_samples * theta_samples,
    }


def write_outputs(reference: RegionIntegral, calg: RegionIntegral) -> dict[str, float]:
    RES_DIR.mkdir(parents=True, exist_ok=True)
    exact = analytic_reference()
    metrics = {
        "exact_active_area_m2": exact["active_area_m2"],
        "calg_active_area_m2": calg.area,
        "active_area_abs_error_m2": abs(calg.area - exact["active_area_m2"]),
        "active_area_rel_error": abs(calg.area - exact["active_area_m2"]) / exact["active_area_m2"],
        "exact_int_gap_m3": exact["int_gap_m3"],
        "calg_int_gap_m3": calg.int_gap,
        "int_gap_abs_error_m3": abs(calg.int_gap - exact["int_gap_m3"]),
        "int_gap_rel_error": abs(calg.int_gap - exact["int_gap_m3"]) / abs(exact["int_gap_m3"]),
        "exact_mean_gap_m": exact["mean_gap_m"],
        "calg_mean_gap_m": calg.mean_gap,
        "mean_gap_abs_error_m": abs(calg.mean_gap - exact["mean_gap_m"]),
        "exact_normal_force_N": exact["normal_force_N"],
        "calg_normal_force_N": calg.pressure_integral,
        "normal_force_abs_error_N": abs(calg.pressure_integral - exact["normal_force_N"]),
        "normal_force_rel_error": abs(calg.pressure_integral - exact["normal_force_N"]) / exact["normal_force_N"],
        "pressure_center_error_m": float(np.linalg.norm(calg.pressure_center - reference.pressure_center)),
        "average_normal_angle_error_deg": normal_angle_deg(calg.average_normal, reference.average_normal),
        "reference_pressure_center_x_m": float(reference.pressure_center[0]),
        "reference_pressure_center_y_m": float(reference.pressure_center[1]),
        "reference_pressure_center_z_m": float(reference.pressure_center[2]),
        "calg_pressure_center_x_m": float(calg.pressure_center[0]),
        "calg_pressure_center_y_m": float(calg.pressure_center[1]),
        "calg_pressure_center_z_m": float(calg.pressure_center[2]),
        "reference_average_normal_x": float(reference.average_normal[0]),
        "reference_average_normal_y": float(reference.average_normal[1]),
        "reference_average_normal_z": float(reference.average_normal[2]),
        "calg_average_normal_x": float(calg.average_normal[0]),
        "calg_average_normal_y": float(calg.average_normal[1]),
        "calg_average_normal_z": float(calg.average_normal[2]),
        "reference_radial_order": reference.radial_order,
        "reference_theta_order": reference.theta_order,
        "calg_radial_order": calg.radial_order,
        "calg_theta_order": calg.theta_order,
        **graphability_statistics(),
    }

    with (RES_DIR / "static_patch_contact_equivalence_summary.json").open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)

    rows = []
    for item in (reference, calg):
        rows.append(
            {
                "method": item.name,
                "active_area_m2": item.area,
                "int_gap_m3": item.int_gap,
                "mean_gap_m": item.mean_gap,
                "normal_force_N": item.pressure_integral,
                "mean_pressure_Pa": item.mean_pressure,
                "pressure_center_m": " ".join(f"{x:.16e}" for x in item.pressure_center),
                "average_normal": " ".join(f"{x:.16e}" for x in item.average_normal),
                "radial_order": item.radial_order,
                "theta_order": item.theta_order,
            }
        )
    with (RES_DIR / "static_patch_contact_equivalence_methods.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    md = [
        "# Static patch-contact equivalence check",
        "",
        "The scene uses two graphable but geometrically complex analytic patches. "
        "Their signed graph gap is prescribed so that the active contact set is a small ellipse, "
        "allowing area, gap, pressure-center, and average-normal quantities to be compared.",
        "",
        "| quantity | value |",
        "|---|---:|",
    ]
    for key, value in metrics.items():
        if isinstance(value, int):
            value_text = str(value)
        else:
            value_text = f"{value:.12e}"
        md.append(f"| {key} | {value_text} |")
    md.append("")
    md.append(
        "The CALG graph-region quadrature reproduces the analytic active area, "
        "gap integral, mean gap, and normal force, while the pressure-weighted center "
        "and average normal agree with the high-order reference quadrature."
    )
    (RES_DIR / "static_patch_contact_equivalence_summary.md").write_text("\n".join(md), encoding="utf-8")
    return metrics


def save_figure(fig: plt.Figure, stem: str) -> None:
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / f"{stem}.pdf", bbox_inches="tight", pad_inches=0.03)
    fig.savefig(FIG_DIR / f"{stem}.svg", bbox_inches="tight", pad_inches=0.03)
    fig.savefig(FIG_DIR / f"{stem}.png", bbox_inches="tight", pad_inches=0.03, dpi=600)
    plt.close(fig)


def draw_model_figure() -> None:
    fig = plt.figure(figsize=(4.8, 3.7))
    ax = fig.add_subplot(111, projection="3d")
    u = np.linspace(*DOMAIN, 120)
    v = np.linspace(*DOMAIN, 120)
    uu, vv = np.meshgrid(u, v)
    za = h_common(uu, vv)
    zb = h_upper(uu, vv)
    ax.plot_surface(uu, vv, za, rstride=3, cstride=3, linewidth=0.10, edgecolor="#5C85B8", color="#CFE0F2", alpha=0.74)
    ax.plot_surface(uu, vv, zb, rstride=3, cstride=3, linewidth=0.10, edgecolor="#61A778", color="#DDEFE4", alpha=0.42)

    r = np.linspace(0.0, 1.0, 40)
    theta = np.linspace(0.0, 2.0 * np.pi, 160)
    rr, tt = np.meshgrid(r, theta, indexing="ij")
    up = U0 + ELLIPSE_A * rr * np.cos(tt)
    vp = V0 + ELLIPSE_B * rr * np.sin(tt)
    zp = h_common(up, vp) + 1.5e-3
    ax.plot_surface(up, vp, zp, linewidth=0.0, color="#C73E3A", alpha=0.82)

    edge_u = U0 + ELLIPSE_A * np.cos(theta)
    edge_v = V0 + ELLIPSE_B * np.sin(theta)
    edge_z = h_common(edge_u, edge_v) + 2.0e-3
    ax.plot(edge_u, edge_v, edge_z, color="#7A1F1F", linewidth=1.4)
    ax.view_init(elev=23, azim=-55)
    ax.set_axis_off()
    ax.set_box_aspect((1.0, 1.0, 0.34))
    save_figure(fig, "static_patch_contact_equivalence_model")


def draw_gap_field_figure() -> None:
    fig, ax = plt.subplots(figsize=(3.2, 2.8))
    u = np.linspace(-0.26, 0.58, 260)
    v = np.linspace(-0.44, 0.20, 220)
    uu, vv = np.meshgrid(u, v)
    g = signed_gap(uu, vv)
    levels = np.linspace(-PENETRATION * 1e3, PENETRATION * 1e3, 25)
    im = ax.contourf(uu, vv, g * 1e3, levels=levels, cmap="coolwarm", extend="both")
    theta = np.linspace(0.0, 2.0 * np.pi, 240)
    ax.plot(U0 + ELLIPSE_A * np.cos(theta), V0 + ELLIPSE_B * np.sin(theta), color="black", linewidth=0.9)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel(r"$\xi_1$ (m)")
    ax.set_ylabel(r"$\xi_2$ (m)")
    cb = fig.colorbar(im, ax=ax, shrink=0.82, pad=0.02)
    cb.set_label(r"$g_{AB}$ (mm)")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    save_figure(fig, "static_patch_contact_equivalence_gap_field")


def draw_error_figure(metrics: dict[str, float]) -> None:
    names = [
        "area",
        r"$\int g\,d\xi$",
        "mean gap",
        "normal force",
        "center",
        "normal angle",
    ]
    values = np.array(
        [
            metrics["active_area_rel_error"],
            metrics["int_gap_rel_error"],
            metrics["mean_gap_abs_error_m"] / abs(metrics["exact_mean_gap_m"]),
            metrics["normal_force_rel_error"],
            metrics["pressure_center_error_m"],
            metrics["average_normal_angle_error_deg"],
        ],
        dtype=float,
    )
    values_for_plot = np.maximum(values, 1e-16)
    fig, ax = plt.subplots(figsize=(4.8, 2.7))
    colors = ["#2F6FAE", "#3C8D5A", "#1A1A1A", "#8A8A8A", "#B83A3A", "#D58A2A"]
    ax.bar(np.arange(len(values)), values_for_plot, color=colors, width=0.66)
    ax.set_yscale("log")
    ax.set_xticks(np.arange(len(values)))
    ax.set_xticklabels(names)
    ax.set_ylabel("discrepancy")
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(axis="y", color="#D9D9D9", linewidth=0.45)
    save_figure(fig, "static_patch_contact_equivalence_errors")


def main() -> None:
    reference = active_quadrature(180, 720, "high-order reference")
    calg = active_quadrature(36, 144, "CALG graph-region quadrature")
    metrics = write_outputs(reference, calg)
    draw_model_figure()
    draw_gap_field_figure()
    draw_error_figure(metrics)
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
