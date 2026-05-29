from __future__ import annotations

from dataclasses import dataclass
from typing import Any
import importlib.util

import numpy as np


@dataclass(frozen=True)
class GPUBackendInfo:
    """Information about the array backend used by a GPU-capable kernel.

    The project keeps all GPU features optional.  If CuPy is not installed, or
    if a CUDA device is unavailable, kernels fall back to NumPy and return this
    fact explicitly.  This makes validation reproducible on CPU-only machines
    while preserving a drop-in GPU execution path for CUDA workstations.
    """

    name: str
    is_gpu: bool
    reason: str


def gpu_available() -> bool:
    if importlib.util.find_spec("cupy") is None:
        return False
    try:
        import cupy as cp  # type: ignore

        return int(cp.cuda.runtime.getDeviceCount()) > 0
    except Exception:
        return False


def get_array_module(prefer_gpu: bool = True) -> tuple[Any, GPUBackendInfo]:
    """Return CuPy when available, otherwise NumPy.

    The function intentionally catches all CuPy import/device errors so that the
    public API never fails merely because the validation host has no GPU.
    """

    if prefer_gpu and importlib.util.find_spec("cupy") is not None:
        try:
            import cupy as cp  # type: ignore

            ndev = int(cp.cuda.runtime.getDeviceCount())
            if ndev > 0:
                return cp, GPUBackendInfo("cupy", True, f"cuda_devices={ndev}")
            return np, GPUBackendInfo("numpy", False, "cupy_installed_but_no_cuda_device")
        except Exception as exc:  # pragma: no cover - hardware dependent
            return np, GPUBackendInfo("numpy", False, f"cupy_unavailable:{type(exc).__name__}")
    return np, GPUBackendInfo("numpy", False, "gpu_not_requested_or_unavailable")
