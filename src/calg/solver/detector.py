from __future__ import annotations

from dataclasses import dataclass, asdict

import numpy as np

from calg.modeling.mesh import TriangleMesh
from calg.core.primitives import build_primitives
from calg.core.bvh import BVH
from calg.core.triangle_distance import triangle_triangle_closest
from calg.core.math_utils import normalize, point_to_barycentric
from .contact_frame import build_contact_frame, graphability_certificate
from .graph_gap import solve_graph_gap
from .curved_newton import solve_curved_patch_pair
from .interval_fallback import bezier_interval_fallback


@dataclass
class ContactSample:
    face_a: int
    face_b: int
    point_a: np.ndarray
    point_b: np.ndarray
    normal: np.ndarray
    gap: float
    method: str
    graph_mu_a: float
    graph_mu_b: float
    feature: str
    overlap_area: float = 0.0
    bary_a: np.ndarray | None = None
    bary_b: np.ndarray | None = None
    solver_iterations: int = 0
    residual: float = 0.0
    certificate: str = ""

    def to_dict(self) -> dict:
        d = asdict(self)
        for key in ("point_a", "point_b", "normal", "bary_a", "bary_b"):
            if d.get(key) is not None:
                d[key] = [float(v) for v in d[key]]
        return d


@dataclass
class DetectionStats:
    candidate_pairs: int
    linear_rejected: int
    graph_attempted: int
    graph_passed: int
    graph_contacts: int
    curved_attempted: int
    curved_contacts: int
    interval_attempted: int
    interval_contacts: int
    fallback_contacts: int
    contacts: int

    @property
    def fast_path_ratio(self) -> float:
        return self.graph_passed / self.candidate_pairs if self.candidate_pairs else 0.0


@dataclass
class DetectionResult:
    contacts: list[ContactSample]
    stats: DetectionStats

    @property
    def min_gap(self) -> float:
        if not self.contacts:
            return float("inf")
        return float(min(c.gap for c in self.contacts))

    def to_rows(self) -> list[dict]:
        return [c.to_dict() for c in self.contacts]


def _bary_for_point(point: np.ndarray, tri: np.ndarray) -> np.ndarray:
    b = point_to_barycentric(point, tri)
    bc = np.maximum(b, 0.0)
    s = float(np.sum(bc))
    return bc / s if s > 1e-14 else np.array([1.0, 0.0, 0.0])


class ContactDetector:
    """Two-body surface contact detector.

    Pipeline:
      1. inflated 3D BVH broad phase;
      2. contact-aligned 2D graph-gap fast path when graphability is certified;
      3. full curved 4D patch-pair Gauss--Newton fallback;
      4. quadratic Bezier control-hull subdivision fallback for uncertain cases.

    The core solver does not require BFF, global UVs, disk topology, or manual
    cuts.  BFF can still be used as an optional external local conditioner.
    """

    def __init__(
        self,
        d_hat: float = 0.01,
        mu_min: float = 0.25,
        two_sided: bool = True,
        leaf_size: int = 8,
        enable_interval_fallback: bool = False,
        interval_depth: int = 6,
        complete_curved_solver: bool = False,
    ):
        self.d_hat = float(d_hat)
        self.mu_min = float(mu_min)
        self.two_sided = bool(two_sided)
        self.leaf_size = int(leaf_size)
        self.enable_interval_fallback = bool(enable_interval_fallback)
        self.interval_depth = int(interval_depth)
        self.complete_curved_solver = bool(complete_curved_solver)

    def _build(self, mesh: TriangleMesh):
        primitives = build_primitives(mesh, contact_margin=self.d_hat)
        return primitives, BVH([p.aabb for p in primitives], leaf_size=self.leaf_size)

    def detect(self, mesh_a: TriangleMesh, mesh_b: TriangleMesh) -> DetectionResult:
        prims_a, bvh_a = self._build(mesh_a)
        prims_b, bvh_b = self._build(mesh_b)
        pairs = bvh_a.query_pairs(bvh_b)

        contacts: list[ContactSample] = []
        linear_rejected = 0
        graph_attempted = 0
        graph_passed = 0
        graph_contacts = 0
        curved_attempted = 0
        curved_contacts = 0
        interval_attempted = 0
        interval_contacts = 0

        for ia, ib in pairs:
            pa = prims_a[ia]
            pb = prims_b[ib]
            eps_pair = pa.error_bound + pb.error_bound
            linear = triangle_triangle_closest(pa.vertices, pb.vertices)
            if linear.distance > self.d_hat + eps_pair:
                linear_rejected += 1
                continue

            frame = build_contact_frame(pa, pb)
            graph = graphability_certificate(pa, pb, frame, mu_min=self.mu_min, two_sided=self.two_sided)
            graph_attempted += 1
            resolved_by_graph = False

            if graph.passed:
                graph_passed += 1
                gg = solve_graph_gap(pa, pb, frame, graph, d_hat=self.d_hat)
                if gg.valid:
                    resolved_by_graph = True
                    if gg.contact:
                        graph_contacts += 1
                        contacts.append(
                            ContactSample(
                                face_a=pa.face_index,
                                face_b=pb.face_index,
                                point_a=gg.point_a,
                                point_b=gg.point_b,
                                normal=gg.normal,
                                gap=float(gg.gap),
                                method="graph2d",
                                graph_mu_a=graph.mu_a,
                                graph_mu_b=graph.mu_b,
                                feature="surface-surface",
                                overlap_area=float(gg.overlap_area),
                                bary_a=_bary_for_point(gg.point_a, pa.vertices),
                                bary_b=_bary_for_point(gg.point_b, pb.vertices),
                                certificate="normal_cone_graphability",
                            )
                        )

            if resolved_by_graph:
                continue

            curved_attempted += 1
            curved = solve_curved_patch_pair(pa, pb, d_hat=self.d_hat, enumerate_active_sets=self.complete_curved_solver)
            if curved.valid and curved.contact:
                curved_contacts += 1
                contacts.append(
                    ContactSample(
                        face_a=pa.face_index,
                        face_b=pb.face_index,
                        point_a=curved.point_a,
                        point_b=curved.point_b,
                        normal=curved.normal,
                        gap=float(curved.gap),
                        method="curved4d_newton",
                        graph_mu_a=graph.mu_a,
                        graph_mu_b=graph.mu_b,
                        feature=curved.feature,
                        overlap_area=0.0,
                        bary_a=curved.bary_a,
                        bary_b=curved.bary_b,
                        solver_iterations=curved.iterations,
                        residual=curved.residual,
                        certificate=curved.reason,
                    )
                )
                continue

            # The Newton candidate may be valid but not converged/decisive near
            # the threshold.  Invoke the conservative Bezier-control fallback for
            # ambiguous cases rather than blindly trusting a local optimum.
            ambiguous = (
                not curved.valid
                or "max_iter" in curved.reason
                or "stalled" in curved.reason
                or (curved.valid and curved.gap <= self.d_hat + eps_pair + 1e-6)
            )
            if self.enable_interval_fallback and ambiguous:
                interval_attempted += 1
                interval = bezier_interval_fallback(pa, pb, d_hat=self.d_hat, max_depth=self.interval_depth)
                if interval.valid and interval.contact:
                    interval_contacts += 1
                    contacts.append(
                        ContactSample(
                            face_a=pa.face_index,
                            face_b=pb.face_index,
                            point_a=interval.point_a,
                            point_b=interval.point_b,
                            normal=interval.normal,
                            gap=float(interval.gap),
                            method="interval_bezier_fallback",
                            graph_mu_a=graph.mu_a,
                            graph_mu_b=graph.mu_b,
                            feature=interval.feature,
                            overlap_area=0.0,
                            bary_a=interval.bary_a,
                            bary_b=interval.bary_b,
                            solver_iterations=interval.nodes_visited,
                            residual=max(0.0, interval.gap - interval.lower_bound),
                            certificate=interval.reason,
                        )
                    )

        fallback_contacts = curved_contacts + interval_contacts
        stats = DetectionStats(
            candidate_pairs=len(pairs),
            linear_rejected=linear_rejected,
            graph_attempted=graph_attempted,
            graph_passed=graph_passed,
            graph_contacts=graph_contacts,
            curved_attempted=curved_attempted,
            curved_contacts=curved_contacts,
            interval_attempted=interval_attempted,
            interval_contacts=interval_contacts,
            fallback_contacts=fallback_contacts,
            contacts=len(contacts),
        )
        return DetectionResult(contacts=contacts, stats=stats)
