from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import numpy as np

from calg.core.aabb import AABB, aabb_distance
from calg.core.bvh import BVH
from calg.core.primitives import build_primitives
from calg.modeling.mesh import TriangleMesh
from .ccd import interpolate_mesh
from .detector import ContactDetector, DetectionResult


@dataclass
class TDIPairCertificate:
    face_a: int
    face_b: int
    lower_bound: float
    interval: tuple[float, float]
    certified_no_contact: bool
    reason: str


@dataclass
class TimeDependentInclusionCCDResult:
    """Conservative global CCD result.

    certified_no_impact=True means all swept candidate pairs were rejected by a
    conservative inclusion bound over the whole step.  impact=True means a
    threshold crossing was found or an unresolved interval had to be treated as
    a blocking event.  In the latter case the result is conservative but may be
    a false positive; it is never used to accept an unsafe full step.
    """

    certified_no_impact: bool
    impact: bool
    uncertain: bool
    toi_lower: float | None
    toi_upper: float | None
    gap_at_upper: float
    pairs_checked: int
    nodes_visited: int
    certificates: list[TDIPairCertificate]
    contacts_at_toi: DetectionResult | None
    reason: str

    @property
    def safe_fraction(self) -> float:
        if self.certified_no_impact:
            return 1.0
        if self.toi_lower is None:
            return 0.0
        return max(0.0, min(1.0, float(self.toi_lower)))


def _swept_face_aabb(mesh0: TriangleMesh, mesh1: TriangleMesh, face_index: int, t0: float, t1: float, margin: float) -> AABB:
    f = mesh0.faces[face_index]
    p0 = (1.0 - t0) * mesh0.vertices[f] + t0 * mesh1.vertices[f]
    p1 = (1.0 - t1) * mesh0.vertices[f] + t1 * mesh1.vertices[f]
    pts = np.vstack([p0, p1])
    return AABB.from_points(pts, margin=margin)


def _swept_boxes(mesh0: TriangleMesh, mesh1: TriangleMesh, margin: float) -> list[AABB]:
    return [_swept_face_aabb(mesh0, mesh1, i, 0.0, 1.0, margin) for i in range(len(mesh0.faces))]


class TimeDependentInclusionCCD:
    """Conservative time-dependent inclusion CCD gate for linearly moving meshes.

    The kernel uses a swept AABB inclusion over both space and time for each
    primitive pair.  A pair/interval is certified safe only when its swept boxes
    are farther than the contact threshold.  Otherwise the time interval is
    bisected until a static detector confirms threshold contact or the time
    tolerance is reached.  Unresolved terminal intervals are returned as
    conservative blocking events rather than as no-contact results.

    This gives a practical global no-penetration *acceptance gate*: a full time
    step is accepted only when every swept pair is certified safe.  It is stricter
    than the sampling-based CCD in ccd.py, at the cost of possible false positives
    near highly ambiguous or very close configurations.
    """

    def __init__(
        self,
        d_hat: float,
        detector: ContactDetector | None = None,
        time_tol: float = 1e-5,
        max_depth: int = 30,
        max_nodes: int = 200000,
        margin: float = 1e-12,
    ):
        self.d_hat = float(d_hat)
        self.detector = detector if detector is not None else ContactDetector(d_hat=d_hat, enable_interval_fallback=True)
        self.time_tol = float(time_tol)
        self.max_depth = int(max_depth)
        self.max_nodes = int(max_nodes)
        self.margin = float(margin)

    def _static(self, a0: TriangleMesh, a1: TriangleMesh, b0: TriangleMesh, b1: TriangleMesh, t: float) -> DetectionResult:
        return self.detector.detect(interpolate_mesh(a0, a1, t, name=f"{a0.name}_tdi"), interpolate_mesh(b0, b1, t, name=f"{b0.name}_tdi"))

    def detect(self, mesh_a0: TriangleMesh, mesh_a1: TriangleMesh, mesh_b0: TriangleMesh, mesh_b1: TriangleMesh) -> TimeDependentInclusionCCDResult:
        if mesh_a0.vertices.shape != mesh_a1.vertices.shape or mesh_b0.vertices.shape != mesh_b1.vertices.shape:
            raise ValueError("TDI CCD requires matching start/end topology for each moving mesh")
        # Early endpoint checks provide true contact samples and make TOI less conservative.
        r0 = self._static(mesh_a0, mesh_a1, mesh_b0, mesh_b1, 0.0)
        if r0.min_gap <= self.d_hat:
            return TimeDependentInclusionCCDResult(False, True, False, 0.0, 0.0, r0.min_gap, 0, 0, [], r0, "initial_contact")

        # Global swept candidate set over the whole step.
        margin = self.margin + self.d_hat
        boxes_a = _swept_boxes(mesh_a0, mesh_a1, margin)
        boxes_b = _swept_boxes(mesh_b0, mesh_b1, margin)
        pairs = BVH(boxes_a).query_pairs(BVH(boxes_b))
        if not pairs:
            return TimeDependentInclusionCCDResult(True, False, False, None, None, float("inf"), 0, 0, [], None, "no_swept_aabb_pairs")

        # Per-face geometric error inflation from current primitive estimator.
        prim_a0 = build_primitives(mesh_a0, contact_margin=0.0)
        prim_b0 = build_primitives(mesh_b0, contact_margin=0.0)
        certs: list[TDIPairCertificate] = []
        counter = itertools.count()
        heap: list[tuple[float, int, int, int, int, float, float]] = []
        for ia, ib in pairs:
            eps = prim_a0[ia].error_bound + prim_b0[ib].error_bound + self.margin
            ba = _swept_face_aabb(mesh_a0, mesh_a1, ia, 0.0, 1.0, eps)
            bb = _swept_face_aabb(mesh_b0, mesh_b1, ib, 0.0, 1.0, eps)
            lb = aabb_distance(ba, bb)
            if lb > self.d_hat:
                certs.append(TDIPairCertificate(int(ia), int(ib), float(lb), (0.0, 1.0), True, "swept_box_separated"))
                continue
            heapq.heappush(heap, (0.0, next(counter), 0, int(ia), int(ib), 0.0, 1.0))

        if not heap:
            return TimeDependentInclusionCCDResult(True, False, False, None, None, float("inf"), len(pairs), 0, certs, None, "all_pairs_certified_by_swept_boxes")

        nodes = 0
        best_block: tuple[float, float, DetectionResult | None, str] | None = None
        while heap and nodes < self.max_nodes:
            _, _, depth, ia, ib, t0, t1 = heapq.heappop(heap)
            nodes += 1
            eps = prim_a0[ia].error_bound + prim_b0[ib].error_bound + self.margin
            ba = _swept_face_aabb(mesh_a0, mesh_a1, ia, t0, t1, eps)
            bb = _swept_face_aabb(mesh_b0, mesh_b1, ib, t0, t1, eps)
            lb = aabb_distance(ba, bb)
            if lb > self.d_hat:
                certs.append(TDIPairCertificate(ia, ib, float(lb), (float(t0), float(t1)), True, "interval_box_separated"))
                continue

            # Ambiguous interval.  We do not stop merely because the right
            # endpoint is in contact; the first impact may be much earlier.
            # Instead, subdivide until the time width is certified/toleranced.
            if depth >= self.max_depth or (t1 - t0) <= self.time_tol:
                r1 = self._static(mesh_a0, mesh_a1, mesh_b0, mesh_b1, t1)
                if r1.min_gap <= self.d_hat:
                    best_block = (t0, t1, r1, "endpoint_threshold_contact")
                else:
                    # Terminal unresolved interval: block the step conservatively.
                    best_block = (t0, t1, r1, "terminal_uncertain_interval")
                break

            tm = 0.5 * (t0 + t1)
            # Push left first via priority key to obtain earliest possible event.
            heapq.heappush(heap, (t0, next(counter), depth + 1, ia, ib, t0, tm))
            heapq.heappush(heap, (tm, next(counter), depth + 1, ia, ib, tm, t1))

        if best_block is not None:
            lo, hi, res, reason = best_block
            return TimeDependentInclusionCCDResult(
                certified_no_impact=False,
                impact=True,
                uncertain=(reason == "terminal_uncertain_interval"),
                toi_lower=float(lo),
                toi_upper=float(hi),
                gap_at_upper=float(res.min_gap if res is not None else float("inf")),
                pairs_checked=len(pairs),
                nodes_visited=nodes,
                certificates=certs,
                contacts_at_toi=res,
                reason=reason,
            )

        if heap:
            return TimeDependentInclusionCCDResult(False, True, True, 0.0, 1.0, float("inf"), len(pairs), nodes, certs, None, "max_nodes_reached_uncertain")
        return TimeDependentInclusionCCDResult(True, False, False, None, None, float("inf"), len(pairs), nodes, certs, None, "all_intervals_certified_safe")
