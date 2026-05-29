from __future__ import annotations

import csv
import json
from pathlib import Path
import numpy as np

from calg.core.aabb import AABB
from calg.gpu import query_aabb_pairs_vectorized, aabb_arrays_from_boxes, gpu_available
from calg.modeling.generators import make_plane_grid
from calg.solver import ContactDetector, TimeDependentInclusionCCD
from calg.dynamics import MassSpringBody, ContactPairSpec, ImplicitScene, ImplicitDynamicsSolver


def run_gpu_broadphase_check():
    boxes_a = [AABB(np.array([0, 0, 0.0]), np.array([1, 1, 0.1])), AABB(np.array([3, 0, 0]), np.array([4, 1, 1]))]
    boxes_b = [AABB(np.array([0.5, 0.5, 0.0]), np.array([1.5, 1.5, 0.1])), AABB(np.array([5, 0, 0]), np.array([6, 1, 1]))]
    loa, hia = aabb_arrays_from_boxes(boxes_a)
    lob, hib = aabb_arrays_from_boxes(boxes_b)
    pairs, info = query_aabb_pairs_vectorized(loa, hia, lob, hib, prefer_gpu=True)
    return {
        "case": "gpu_vectorized_broadphase",
        "backend": info.name,
        "gpu_used": bool(info.is_gpu),
        "reason": info.reason,
        "pairs": int(len(pairs)),
        "passed": bool(len(pairs) == 1 and tuple(pairs[0]) == (0, 0)),
    }


def run_tdi_ccd_checks():
    det = ContactDetector(d_hat=0.10, mu_min=0.1, enable_interval_fallback=True)
    a0 = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a0")
    a1 = a0.copy(name="a1")
    b0 = make_plane_grid(size=1.0, n=1, z=0.30, normal=(0, 0, -1), name="b0")
    b1 = make_plane_grid(size=1.0, n=1, z=0.02, normal=(0, 0, -1), name="b1")
    ccd = TimeDependentInclusionCCD(d_hat=0.10, detector=det, time_tol=2e-3, max_depth=24)
    r = ccd.detect(a0, a1, b0, b1)
    expected = (0.30 - 0.10) / (0.30 - 0.02)
    return {
        "case": "tdi_ccd_moving_plane",
        "backend": "cpu_conservative_inclusion",
        "gpu_used": False,
        "impact": bool(r.impact),
        "certified_no_impact": bool(r.certified_no_impact),
        "uncertain": bool(r.uncertain),
        "toi_lower": r.toi_lower,
        "toi_upper": r.toi_upper,
        "expected_toi": expected,
        "nodes_visited": int(r.nodes_visited),
        "pairs_checked": int(r.pairs_checked),
        "passed": bool(r.impact and r.toi_lower is not None and r.toi_upper is not None and r.toi_lower <= expected <= r.toi_upper + 0.01),
        "reason": r.reason,
    }


def run_implicit_dynamics_check(out_dir: Path):
    mover_mesh = make_plane_grid(size=1.0, n=1, z=0.25, normal=(0, 0, 1), name="mover")
    fixed_mesh = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="fixed")
    mover = MassSpringBody(mover_mesh, velocity=np.tile(np.array([0.0, 0.0, -1.0]), (len(mover_mesh.vertices), 1)), spring_stiffness=1.0, name="mover")
    fixed = MassSpringBody(fixed_mesh, fixed=np.ones(len(fixed_mesh.vertices), dtype=bool), spring_stiffness=0.0, name="fixed")
    pair = ContactPairSpec(0, 1, d_hat=0.08, stiffness=20.0, detector=ContactDetector(d_hat=0.08, mu_min=0.1, enable_interval_fallback=True))
    scene = ImplicitScene(bodies=[mover, fixed], contact_pairs=[pair])
    solver = ImplicitDynamicsSolver(max_iter=10, grad_tol=1e-5)
    res = solver.step(scene, dt=0.30, use_ccd_gate=True)
    det_final = ContactDetector(d_hat=0.08, mu_min=0.1, enable_interval_fallback=True).detect(scene.bodies[0].mesh, scene.bodies[1].mesh)
    explicit_gap = float(scene.bodies[0].mesh.vertices[:, 2].mean() - scene.bodies[1].mesh.vertices[:, 2].mean())
    row = {
        "case": "implicit_global_step_with_tdi_gate",
        "backend": "cpu_global_implicit",
        "gpu_used": False,
        "converged": bool(res.converged),
        "iterations": int(res.iterations),
        "initial_energy": float(res.initial_energy),
        "final_energy": float(res.final_energy),
        "max_gradient_norm": float(res.max_gradient_norm),
        "ccd_clamped": bool(res.ccd_clamped),
        "final_min_gap": float(det_final.min_gap),
        "explicit_mean_z_gap": explicit_gap,
        "d_hat": 0.08,
        "passed": bool(explicit_gap >= 0.075),
        "reason": res.reason,
    }
    return row


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="results/v0_3_validation")
    args = parser.parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = [run_gpu_broadphase_check(), run_tdi_ccd_checks(), run_implicit_dynamics_check(out_dir)]
    keys = sorted(set().union(*(r.keys() for r in rows)))
    csv_path = out_dir / "v0_3_validation_summary.csv"
    with csv_path.open("w", newline="", encoding="utf8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    json_path = out_dir / "v0_3_validation_details.json"
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf8")
    md = ["# CALG v0.3 validation summary", "", f"GPU available on this host: `{gpu_available()}`", "", "| case | passed | key result |", "|---|---:|---|"]
    for r in rows:
        if r["case"] == "gpu_vectorized_broadphase":
            key = f"backend={r['backend']}, pairs={r['pairs']}"
        elif r["case"] == "tdi_ccd_moving_plane":
            key = f"toi=[{r['toi_lower']}, {r['toi_upper']}], expected={r['expected_toi']:.6f}, reason={r['reason']}"
        else:
            key = f"explicit_gap={r['explicit_mean_z_gap']:.6f}, ccd_clamped={r['ccd_clamped']}"
        md.append(f"| {r['case']} | {r['passed']} | {key} |")
    md_path = out_dir / "v0_3_validation_summary.md"
    md_path.write_text("\n".join(md) + "\n", encoding="utf8")
    print(md_path)


if __name__ == "__main__":
    main()
