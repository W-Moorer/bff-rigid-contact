from __future__ import annotations

import csv
import json
from dataclasses import dataclass, asdict
from pathlib import Path
from time import perf_counter

import numpy as np

from calg.gpu.broadphase import query_aabb_pairs_vectorized
from calg.gpu.backend import get_array_module


@dataclass
class GPUBenchmarkRow:
    n_a: int
    n_b: int
    tile_size: int
    backend: str
    is_true_cuda: bool
    backend_reason: str
    pairs: int
    elapsed_s: float
    throughput_mtests_s: float
    notes: str

    def to_dict(self) -> dict:
        return asdict(self)


def _random_boxes(n: int, rng: np.random.Generator, spread: float = 1.0, size: float = 0.025):
    centers = rng.uniform(-spread, spread, size=(n, 3))
    half = rng.uniform(0.25 * size, size, size=(n, 3))
    return centers - half, centers + half


def _sync_backend():
    try:
        import cupy as cp  # type: ignore
        cp.cuda.Stream.null.synchronize()
    except Exception:
        pass


def run_gpu_benchmark(out_dir: str | Path, sizes: list[int] | None = None, repeats: int = 3, seed: int = 11) -> list[GPUBenchmarkRow]:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    sizes = [512, 1024, 2048, 4096] if sizes is None else sizes
    rng = np.random.default_rng(seed)
    rows: list[GPUBenchmarkRow] = []

    # Prepare backend info once.  The kernel itself still controls fallback.
    _, info = get_array_module(prefer_gpu=True)
    for n in sizes:
        lo_a, hi_a = _random_boxes(n, rng)
        # Shift B slightly to keep a non-trivial but not full overlap rate.
        lo_b, hi_b = _random_boxes(n, rng)
        lo_b[:, 0] += 0.01
        hi_b[:, 0] += 0.01
        best = float("inf")
        best_pairs = 0
        for rep in range(max(1, repeats)):
            _sync_backend()
            t0 = perf_counter()
            pairs, used = query_aabb_pairs_vectorized(lo_a, hi_a, lo_b, hi_b, prefer_gpu=True, tile_size=min(2048, n))
            _sync_backend()
            elapsed = perf_counter() - t0
            if elapsed < best:
                best = elapsed
                best_pairs = int(len(pairs))
                info = used
        ntests = float(n) * float(n)
        throughput = (ntests / best) / 1.0e6 if best > 0 else float("inf")
        rows.append(
            GPUBenchmarkRow(
                n_a=int(n),
                n_b=int(n),
                tile_size=int(min(2048, n)),
                backend=info.name,
                is_true_cuda=bool(info.is_gpu),
                backend_reason=info.reason,
                pairs=best_pairs,
                elapsed_s=float(best),
                throughput_mtests_s=float(throughput),
                notes="true CUDA GPU timing" if info.is_gpu else "CPU/NumPy fallback; run this script on a CUDA workstation for true GPU acceleration data",
            )
        )
    csv_path = out_dir / "gpu_broadphase_benchmark.csv"
    with csv_path.open("w", newline="", encoding="utf8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].to_dict().keys()))
        writer.writeheader()
        for row in rows:
            writer.writerow(row.to_dict())
    (out_dir / "gpu_broadphase_benchmark.json").write_text(json.dumps([r.to_dict() for r in rows], indent=2), encoding="utf8")
    md = out_dir / "gpu_broadphase_benchmark.md"
    with md.open("w", encoding="utf8") as f:
        true_cuda = any(r.is_true_cuda for r in rows)
        f.write("# GPU broad-phase benchmark\n\n")
        if true_cuda:
            f.write("This run contains true CUDA timing data.\n\n")
        else:
            f.write("This run did not detect a CUDA backend. The numbers below are NumPy fallback timings and must not be reported as GPU acceleration. Re-run `scripts/run_gpu_benchmark.py` on the CUDA workstation to generate true GPU data.\n\n")
        f.write("| nA | nB | backend | CUDA? | pairs | time (s) | M tests/s | notes |\n")
        f.write("|---:|---:|---|---|---:|---:|---:|---|\n")
        for r in rows:
            f.write(f"| {r.n_a} | {r.n_b} | {r.backend} | {r.is_true_cuda} | {r.pairs} | {r.elapsed_s:.6g} | {r.throughput_mtests_s:.3f} | {r.notes} |\n")
    return rows
