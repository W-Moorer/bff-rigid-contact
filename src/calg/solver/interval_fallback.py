from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import numpy as np

from calg.core.aabb import AABB, aabb_distance, aabb_diagonal
from calg.core.curved_patch import QuadraticPatch3D, subdivide_triangle_2d
from calg.core.math_utils import EPS, normalize
from calg.core.primitives import CurvedJetPrimitive
from calg.core.triangle_distance import triangle_triangle_closest


@dataclass
class IntervalFallbackResult:
    valid: bool
    contact: bool
    certified_no_contact: bool
    point_a: np.ndarray
    point_b: np.ndarray
    normal: np.ndarray
    gap: float
    bary_a: np.ndarray
    bary_b: np.ndarray
    nodes_visited: int
    lower_bound: float
    feature: str
    reason: str


def _sample_subpair(pa: QuadraticPatch3D, pb: QuadraticPatch3D, ta: np.ndarray, tb: np.ndarray):
    tri_a = pa.evaluated_triangle(ta)
    tri_b = pb.evaluated_triangle(tb)
    closest = triangle_triangle_closest(tri_a, tri_b)
    # For barycentric assembly, project the closest sample back to the original
    # patch domains.  This is approximate but stable for a fallback sample.
    xi_a = pa.project_to_domain(closest.point_a)
    xi_b = pb.project_to_domain(closest.point_b)
    return closest, xi_a, xi_b


def bezier_interval_fallback(
    prim_a: CurvedJetPrimitive,
    prim_b: CurvedJetPrimitive,
    d_hat: float,
    max_depth: int = 7,
    max_nodes: int = 4000,
    box_tol: float = 1e-5,
) -> IntervalFallbackResult:
    """Subdivision fallback using exact quadratic Bezier control-hull AABBs.

    Each quadratic normal-lifted patch has an exact quadratic Bezier control
    hull over any triangular subdomain.  The hull AABB provides a conservative
    lower bound for branch-and-bound pruning.  This routine is intended as the
    robust fallback after graph-gap and Newton paths, not as the default path.
    """
    pa = QuadraticPatch3D.from_primitive(prim_a)
    pb = QuadraticPatch3D.from_primitive(prim_b)
    margin = prim_a.error_bound + prim_b.error_bound

    tri_a0 = pa.domain.copy()
    tri_b0 = pb.domain.copy()
    box_a0 = pa.aabb_for_subtriangle(tri_a0, margin=0.0)
    box_b0 = pb.aabb_for_subtriangle(tri_b0, margin=0.0)
    lower0 = aabb_distance(box_a0.expanded(prim_a.error_bound), box_b0.expanded(prim_b.error_bound))

    best_gap = float("inf")
    best_pa = pa.eval(pa.centroid_xi())
    best_pb = pb.eval(pb.centroid_xi())
    best_xia = pa.centroid_xi()
    best_xib = pb.centroid_xi()
    best_feature = "interval_init"

    counter = itertools.count()
    heap: list[tuple[float, int, int, np.ndarray, np.ndarray, AABB, AABB]] = []
    heapq.heappush(heap, (lower0, next(counter), 0, tri_a0, tri_b0, box_a0, box_b0))
    nodes = 0
    min_unprocessed = lower0

    while heap and nodes < max_nodes:
        lower, _, depth, ta, tb, ba, bb = heapq.heappop(heap)
        nodes += 1
        min_unprocessed = float(lower)
        # No subpair below this one can improve a certified no-contact result.
        if lower > d_hat + margin and lower >= best_gap - margin:
            continue

        closest, xia, xib = _sample_subpair(pa, pb, ta, tb)
        if closest.distance < best_gap:
            best_gap = float(closest.distance)
            best_pa = closest.point_a.copy()
            best_pb = closest.point_b.copy()
            best_xia = xia
            best_xib = xib
            best_feature = closest.feature

        diag_a = aabb_diagonal(ba)
        diag_b = aabb_diagonal(bb)
        if depth >= max_depth or max(diag_a, diag_b) <= box_tol:
            continue

        if lower > best_gap + margin:
            continue

        if diag_a >= diag_b:
            for child in subdivide_triangle_2d(ta):
                cbox = pa.aabb_for_subtriangle(child)
                lb = aabb_distance(cbox.expanded(prim_a.error_bound), bb.expanded(prim_b.error_bound))
                if lb <= best_gap + margin or lb <= d_hat + margin:
                    heapq.heappush(heap, (lb, next(counter), depth + 1, child, tb, cbox, bb))
        else:
            for child in subdivide_triangle_2d(tb):
                cbox = pb.aabb_for_subtriangle(child)
                lb = aabb_distance(ba.expanded(prim_a.error_bound), cbox.expanded(prim_b.error_bound))
                if lb <= best_gap + margin or lb <= d_hat + margin:
                    heapq.heappush(heap, (lb, next(counter), depth + 1, ta, child, ba, cbox))

    if heap:
        min_unprocessed = min(min_unprocessed, float(heap[0][0]))
    certified_no = bool(best_gap > d_hat + margin and (not heap or min_unprocessed > d_hat + margin))
    contact = bool(best_gap <= d_hat + margin)
    normal = normalize(best_pb - best_pa, normalize(prim_b.centroid - prim_a.centroid, prim_a.face_normal))
    valid = bool(np.isfinite(best_gap))
    reason = "contact_found" if contact else ("certified_no_contact" if certified_no else "uncertain_best_sample")
    return IntervalFallbackResult(
        valid=valid,
        contact=contact,
        certified_no_contact=certified_no,
        point_a=best_pa,
        point_b=best_pb,
        normal=normal,
        gap=float(best_gap),
        bary_a=pa.barycentric(best_xia),
        bary_b=pb.barycentric(best_xib),
        nodes_visited=int(nodes),
        lower_bound=float(min_unprocessed),
        feature=f"interval_{best_feature}",
        reason=reason,
    )
