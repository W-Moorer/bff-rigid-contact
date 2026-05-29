from __future__ import annotations

import csv
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from time import perf_counter

import numpy as np

from calg.modeling.generators import make_ruffled_sphere, make_plane_grid, make_lobed_rotor_patch, make_open_wavy_sheet
from calg.solver.detector import ContactDetector


@dataclass
class RobustnessRow:
    case_id: int
    geometry: str
    faces_a: int
    faces_b: int
    d_hat: float
    candidate_pairs: int
    contacts: int
    min_gap: float
    fast_path_ratio: float
    curved_attempted: int
    interval_attempted: int
    elapsed_s: float
    status: str
    notes: str

    def to_dict(self) -> dict:
        return asdict(self)


def _make_case(i: int, rng: np.random.Generator):
    kind = i % 3
    if kind == 0:
        gap = float(rng.uniform(0.035, 0.11))
        amp = float(rng.uniform(0.02, 0.08))
        a = make_ruffled_sphere(radius=0.45, center=(0, 0, 0.45 + gap), n_lat=6, n_lon=12, amp=amp, phase=float(rng.uniform(0, 6.28)), name=f"ruffled_{i}")
        b = make_plane_grid(size=1.4, n=5, z=0.0, normal=(0, 0, 1), name="plane")
        return "ruffled_sphere_plane", a, b, gap + 0.04
    if kind == 1:
        gap = float(rng.uniform(0.025, 0.09))
        a = make_lobed_rotor_patch(lobes=int(rng.integers(4, 8)), radius=0.38, length=1.0, amp=float(rng.uniform(0.03, 0.08)), n_theta=8, n_z=3, center=(0, 0, 0.38 + gap), axis="x", name=f"rotor_{i}")
        b = make_plane_grid(size=1.7, n=6, z=0.0, normal=(0, 0, 1), name="plane")
        return "lobed_rotor_plane", a, b, min(gap + 0.035, 0.10)
    gap = float(rng.uniform(0.02, 0.08))
    a = make_open_wavy_sheet(size=1.2, n=5, amplitude=float(rng.uniform(0.015, 0.06)), z0=gap + 0.06, name=f"wavy_{i}")
    b = make_plane_grid(size=1.4, n=5, z=0.0, normal=(0, 0, 1), name="plane")
    # add small lateral shift to avoid identical grid coincidences
    a = a.transformed(t=np.array([float(rng.uniform(-0.05, 0.05)), float(rng.uniform(-0.05, 0.05)), 0.0]), name=a.name)
    return "wavy_sheet_plane", a, b, min(gap + 0.045, 0.10)


def run_robustness_study(out_dir: str | Path, num_cases: int = 21, seed: int = 7) -> list[RobustnessRow]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    detector = ContactDetector(d_hat=0.1, mu_min=0.15, two_sided=True, enable_interval_fallback=False, interval_depth=2)
    rows: list[RobustnessRow] = []
    for i in range(num_cases):
        geometry, a, b, d_hat = _make_case(i, rng)
        detector.d_hat = d_hat
        t0 = perf_counter()
        try:
            res = detector.detect(a, b)
            elapsed = perf_counter() - t0
            rows.append(
                RobustnessRow(
                    case_id=i,
                    geometry=geometry,
                    faces_a=int(len(a.faces)),
                    faces_b=int(len(b.faces)),
                    d_hat=float(d_hat),
                    candidate_pairs=int(res.stats.candidate_pairs),
                    contacts=int(len(res.contacts)),
                    min_gap=float(res.min_gap),
                    fast_path_ratio=float(res.stats.fast_path_ratio),
                    curved_attempted=int(res.stats.curved_attempted),
                    interval_attempted=int(res.stats.interval_attempted),
                    elapsed_s=float(elapsed),
                    status="passed",
                    notes="",
                )
            )
        except Exception as exc:
            elapsed = perf_counter() - t0
            rows.append(
                RobustnessRow(i, geometry, int(len(a.faces)), int(len(b.faces)), float(d_hat), 0, 0, float("inf"), 0.0, 0, 0, elapsed, "failed", f"{type(exc).__name__}: {exc}")
            )
    csv_path = out_dir / "robustness_statistics.csv"
    with csv_path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].to_dict().keys()))
        writer.writeheader()
        for r in rows:
            writer.writerow(r.to_dict())
    (out_dir / "robustness_statistics.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")
    passed = sum(r.status == "passed" for r in rows)
    md = out_dir / "robustness_statistics.md"
    with md.open("w", encoding="utf8") as f:
        f.write("# Large-scale complex-surface robustness statistics\n\n")
        f.write(f"Cases: {len(rows)}, passed: {passed}, failed: {len(rows)-passed}.\n\n")
        f.write("| id | geometry | faces A/B | contacts | candidates | min gap | fast path | time (s) | status |\n")
        f.write("|---:|---|---:|---:|---:|---:|---:|---:|---|\n")
        for r in rows:
            f.write(f"| {r.case_id} | {r.geometry} | {r.faces_a}/{r.faces_b} | {r.contacts} | {r.candidate_pairs} | {r.min_gap:.6g} | {r.fast_path_ratio:.3f} | {r.elapsed_s:.4g} | {r.status} |\n")
    return rows
