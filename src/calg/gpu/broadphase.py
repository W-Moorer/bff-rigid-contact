from __future__ import annotations

from typing import Sequence
import numpy as np

from calg.core.aabb import AABB
from .backend import GPUBackendInfo, get_array_module


def aabb_arrays_from_boxes(boxes: Sequence[AABB]) -> tuple[np.ndarray, np.ndarray]:
    lo = np.asarray([b.lo for b in boxes], dtype=float)
    hi = np.asarray([b.hi for b in boxes], dtype=float)
    return lo, hi


def query_aabb_pairs_vectorized(
    lo_a: np.ndarray,
    hi_a: np.ndarray,
    lo_b: np.ndarray,
    hi_b: np.ndarray,
    prefer_gpu: bool = True,
    tile_size: int = 2048,
) -> tuple[np.ndarray, GPUBackendInfo]:
    """Vectorized all-pairs AABB overlap query with optional CUDA acceleration.

    This is not a replacement for the tree BVH in the core solver.  It is a
    GPU-friendly broad-phase kernel for dense or moderately sized batches where
    broadcasting/tiled overlap tests are efficient.  The return value is always
    a CPU NumPy integer array of shape (k, 2), plus backend metadata.
    """

    lo_a = np.asarray(lo_a, dtype=float)
    hi_a = np.asarray(hi_a, dtype=float)
    lo_b = np.asarray(lo_b, dtype=float)
    hi_b = np.asarray(hi_b, dtype=float)
    if lo_a.shape != hi_a.shape or lo_b.shape != hi_b.shape or lo_a.shape[1] != 3 or lo_b.shape[1] != 3:
        raise ValueError("AABB arrays must have shape (n,3) and matching lo/hi shapes")

    xp, info = get_array_module(prefer_gpu=prefer_gpu)
    pairs_chunks: list[np.ndarray] = []
    nb = lo_b.shape[0]

    # Transfer B once for GPU backends; NumPy simply views/copies as needed.
    xb_lo = xp.asarray(lo_b)
    xb_hi = xp.asarray(hi_b)
    for start in range(0, lo_a.shape[0], max(1, int(tile_size))):
        end = min(start + max(1, int(tile_size)), lo_a.shape[0])
        xa_lo = xp.asarray(lo_a[start:end])
        xa_hi = xp.asarray(hi_a[start:end])
        overlap = xp.all((xa_hi[:, None, :] >= xb_lo[None, :, :]) & (xb_hi[None, :, :] >= xa_lo[:, None, :]), axis=2)
        idx = xp.argwhere(overlap)
        if getattr(idx, "size", 0) == 0:
            continue
        if info.is_gpu:
            idx_np = idx.get()
        else:
            idx_np = np.asarray(idx)
        idx_np[:, 0] += start
        pairs_chunks.append(idx_np.astype(np.int64, copy=False))

    if not pairs_chunks:
        return np.empty((0, 2), dtype=np.int64), info
    pairs = np.vstack(pairs_chunks)
    if pairs.shape[0] > lo_a.shape[0] * nb:
        raise RuntimeError("internal broad-phase pair count overflow")
    return pairs, info
