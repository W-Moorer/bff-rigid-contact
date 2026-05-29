#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from calg.modeling.generators import make_plane_grid
from calg.core.primitives import build_primitives
from calg.solver.curved_newton import solve_curved_patch_pair
from calg.solver.interval_fallback import bezier_interval_fallback
from calg.solver import ContactDetector, ContinuousContactDetector, ContactEnergyModel, evaluate_contact_response, assemble_nodal_forces


def main() -> int:
    out = Path("results/advanced_validation")
    out.mkdir(parents=True, exist_ok=True)
    rows = []

    # 1. Curved 4D Newton and interval fallback on the same primitive pair.
    a = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=1, z=0.08, normal=(0, 0, -1), name="b")
    pa = build_primitives(a, contact_margin=0.12)[0]
    pb = build_primitives(b, contact_margin=0.12)[0]
    newton = solve_curved_patch_pair(pa, pb, d_hat=0.12)
    interval = bezier_interval_fallback(pa, pb, d_hat=0.12, max_depth=3)
    rows.append({"case": "curved4d_newton_plane_pair", "metric": "gap", "value": newton.gap, "expected": 0.08, "passed": abs(newton.gap - 0.08) < 1e-8})
    rows.append({"case": "interval_bezier_plane_pair", "metric": "gap", "value": interval.gap, "expected": 0.08, "passed": abs(interval.gap - 0.08) < 1e-6})

    # 2. CCD threshold TOI.
    a0 = make_plane_grid(size=1.0, n=2, z=0.0, normal=(0, 0, 1), name="a0")
    a1 = a0.copy(name="a1")
    b0 = make_plane_grid(size=1.0, n=2, z=0.20, normal=(0, 0, -1), name="b0")
    b1 = make_plane_grid(size=1.0, n=2, z=0.05, normal=(0, 0, -1), name="b1")
    ccd = ContinuousContactDetector(d_hat=0.10, detector=ContactDetector(d_hat=0.10, mu_min=0.1), samples=8, time_tol=1e-3)
    ccd_result = ccd.detect(a0, a1, b0, b1)
    expected_toi = (0.20 - 0.10) / (0.20 - 0.05)
    rows.append({"case": "threshold_ccd_moving_plane", "metric": "toi", "value": ccd_result.toi if ccd_result.toi is not None else float("nan"), "expected": expected_toi, "passed": ccd_result.impact and abs(ccd_result.toi - expected_toi) < 0.02})

    # 3. IPC/GCP-style barrier response plus Coulomb friction and nodal assembly.
    det = ContactDetector(d_hat=0.12, mu_min=0.1)
    result = det.detect(a0, make_plane_grid(size=1.0, n=2, z=0.08, normal=(0, 0, -1), name="b_static"))
    contacts = result.contacts[:3]
    model = ContactEnergyModel(d_hat=0.12, stiffness=10.0, model="ipc_barrier", friction_mu=0.3)
    response = evaluate_contact_response(contacts, model, [np.array([1.0, 0.0, 0.0]) for _ in contacts])
    fa, fb = assemble_nodal_forces(a0, make_plane_grid(size=1.0, n=2, z=0.08, normal=(0, 0, -1), name="b_static"), contacts, response)
    rows.append({"case": "ipc_barrier_response", "metric": "energy", "value": response.total_energy, "expected": ">0", "passed": response.total_energy > 0})
    rows.append({"case": "force_assembly_balance", "metric": "net_force_norm", "value": float(np.linalg.norm(fa.sum(axis=0) + fb.sum(axis=0))), "expected": 0.0, "passed": float(np.linalg.norm(fa.sum(axis=0) + fb.sum(axis=0))) < 1e-8})

    with (out / "advanced_validation_summary.csv").open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=["case", "metric", "value", "expected", "passed"])
        writer.writeheader()
        writer.writerows(rows)
    with (out / "advanced_validation_details.json").open("w", encoding="utf8") as f:
        json.dump({"rows": rows, "ccd_reason": ccd_result.reason}, f, indent=2)
    with (out / "advanced_validation_summary.md").open("w", encoding="utf8") as f:
        f.write("# CALG advanced validation summary\n\n")
        f.write("| case | metric | value | expected | passed |\n|---|---|---:|---:|---|\n")
        for r in rows:
            f.write(f"| {r['case']} | {r['metric']} | {r['value']} | {r['expected']} | {r['passed']} |\n")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
