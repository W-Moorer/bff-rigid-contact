from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from calg.modeling.mesh import TriangleMesh
from .detector import ContactDetector, DetectionResult


@dataclass
class CCDResult:
    impact: bool
    toi: float | None
    gap_at_toi: float
    iterations: int
    bracket: tuple[float, float] | None
    contacts: DetectionResult | None
    reason: str


def interpolate_mesh(mesh0: TriangleMesh, mesh1: TriangleMesh, t: float, name: str | None = None) -> TriangleMesh:
    if mesh0.vertices.shape != mesh1.vertices.shape or mesh0.faces.shape != mesh1.faces.shape:
        raise ValueError("CCD interpolation requires matching topology for mesh0 and mesh1")
    v = (1.0 - t) * mesh0.vertices + t * mesh1.vertices
    n0 = mesh0.vertex_normals
    n1 = mesh1.vertex_normals
    normals = None
    if n0 is not None and n1 is not None and n0.shape == n1.shape:
        normals = (1.0 - t) * n0 + t * n1
    return TriangleMesh(v, mesh0.faces.copy(), normals, mesh0.name if name is None else name)


class ContinuousContactDetector:
    """Practical threshold CCD front-end for linearly moving meshes.

    The detector brackets the first time at which the static high-order contact
    detector reports min_gap <= d_hat, then refines the time of impact by
    bisection.  It is designed as the project-level CCD API and can later be
    replaced by pairwise time-dependent inclusion kernels without changing the
    public interface.
    """

    def __init__(
        self,
        d_hat: float,
        detector: ContactDetector | None = None,
        samples: int = 24,
        time_tol: float = 1e-5,
        max_bisect_iter: int = 40,
    ):
        self.d_hat = float(d_hat)
        self.detector = detector if detector is not None else ContactDetector(d_hat=d_hat)
        self.samples = max(2, int(samples))
        self.time_tol = float(time_tol)
        self.max_bisect_iter = int(max_bisect_iter)

    def _eval(self, a0: TriangleMesh, a1: TriangleMesh, b0: TriangleMesh, b1: TriangleMesh, t: float) -> DetectionResult:
        ma = interpolate_mesh(a0, a1, t, name=f"{a0.name}_t{t:.6f}")
        mb = interpolate_mesh(b0, b1, t, name=f"{b0.name}_t{t:.6f}")
        return self.detector.detect(ma, mb)

    def detect(self, mesh_a0: TriangleMesh, mesh_a1: TriangleMesh, mesh_b0: TriangleMesh, mesh_b1: TriangleMesh) -> CCDResult:
        # Check t=0 first.
        r0 = self._eval(mesh_a0, mesh_a1, mesh_b0, mesh_b1, 0.0)
        if r0.min_gap <= self.d_hat:
            return CCDResult(True, 0.0, r0.min_gap, 0, (0.0, 0.0), r0, "initial_contact")

        prev_t = 0.0
        prev_gap = r0.min_gap
        bracket: tuple[float, float] | None = None
        best_result = r0
        # Uniform scan to find the first crossing.  The public interface is
        # independent of this bracketing strategy.
        for i in range(1, self.samples + 1):
            t = i / self.samples
            ri = self._eval(mesh_a0, mesh_a1, mesh_b0, mesh_b1, t)
            best_result = ri
            if ri.min_gap <= self.d_hat:
                bracket = (prev_t, t)
                break
            prev_t = t
            prev_gap = ri.min_gap
        if bracket is None:
            return CCDResult(False, None, best_result.min_gap, self.samples, None, best_result, "no_threshold_crossing_in_scan")

        lo, hi = bracket
        iterations = 0
        last = best_result
        while iterations < self.max_bisect_iter and hi - lo > self.time_tol:
            mid = 0.5 * (lo + hi)
            rm = self._eval(mesh_a0, mesh_a1, mesh_b0, mesh_b1, mid)
            last = rm
            iterations += 1
            if rm.min_gap <= self.d_hat:
                hi = mid
            else:
                lo = mid
        toi = hi
        result = self._eval(mesh_a0, mesh_a1, mesh_b0, mesh_b1, toi)
        return CCDResult(True, float(toi), result.min_gap, iterations, (lo, hi), result, "threshold_toi_bisection")
