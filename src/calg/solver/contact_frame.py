from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from calg.core.math_utils import normalize, orthonormal_basis_from_normal, dot
from calg.core.triangle_distance import triangle_triangle_closest, ClosestPair
from calg.core.primitives import CurvedJetPrimitive


@dataclass
class ContactFrame:
    origin: np.ndarray
    t1: np.ndarray
    t2: np.ndarray
    n: np.ndarray
    closest: ClosestPair

    def project_xyh(self, x: np.ndarray) -> tuple[np.ndarray, float]:
        r = x - self.origin
        return np.array([dot(self.t1, r), dot(self.t2, r)]), dot(self.n, r)

    def lift(self, xi: np.ndarray, h: float) -> np.ndarray:
        return self.origin + xi[0] * self.t1 + xi[1] * self.t2 + h * self.n


def build_contact_frame(pa: CurvedJetPrimitive, pb: CurvedJetPrimitive) -> ContactFrame:
    closest = triangle_triangle_closest(pa.vertices, pb.vertices)
    origin = 0.5 * (closest.point_a + closest.point_b)
    direction = closest.point_b - closest.point_a
    if np.linalg.norm(direction) < 1e-10:
        # Use opposing local normals when the linear closest points coincide.
        na = normalize(pa.vertex_normals.mean(axis=0), pa.face_normal)
        nb = normalize(pb.vertex_normals.mean(axis=0), pb.face_normal)
        direction = na - nb
        if np.linalg.norm(direction) < 1e-10:
            direction = pa.centroid - pb.centroid
    t1, t2, n = orthonormal_basis_from_normal(direction)
    return ContactFrame(origin=origin, t1=t1, t2=t2, n=n, closest=closest)


@dataclass
class Graphability:
    passed: bool
    mu_a: float
    mu_b: float
    sign_a: float
    sign_b: float
    reason: str


def graphability_certificate(
    pa: CurvedJetPrimitive,
    pb: CurvedJetPrimitive,
    frame: ContactFrame,
    mu_min: float = 0.25,
    two_sided: bool = True,
) -> Graphability:
    """Check whether both primitives can be represented as graphs over the contact plane.

    The local graph height is taken along frame.n.  For oriented closed bodies one
    may set two_sided=False to require A's provided normals to align with +n and
    B's normals to align with -n.  For generic open/possibly unoriented input the
    default two_sided=True automatically chooses the normal signs that maximize
    graphability, while preserving the gap direction from the closest point on A
    to the closest point on B.
    """
    n = frame.n
    vals_a = np.array([np.dot(na, n) for na in pa.vertex_normals])
    vals_b = np.array([np.dot(nb, n) for nb in pb.vertex_normals])
    if two_sided:
        min_a_pos = float(vals_a.min())
        min_a_neg = float((-vals_a).min())
        sign_a = 1.0 if min_a_pos >= min_a_neg else -1.0
        mu_a = max(min_a_pos, min_a_neg)
        min_b_pos = float(vals_b.min())
        min_b_neg = float((-vals_b).min())
        sign_b = 1.0 if min_b_pos >= min_b_neg else -1.0
        mu_b = max(min_b_pos, min_b_neg)
    else:
        sign_a = 1.0
        # B's outward normal should oppose the A-to-B contact direction.
        sign_b = -1.0
        mu_a = float(vals_a.min())
        mu_b = float((-vals_b).min())
    passed = bool(mu_a >= mu_min and mu_b >= mu_min)
    reason = "passed" if passed else f"normal-cone projection is ill-conditioned: mu_a={mu_a:.3g}, mu_b={mu_b:.3g}, threshold={mu_min:.3g}"
    return Graphability(passed=passed, mu_a=mu_a, mu_b=mu_b, sign_a=sign_a, sign_b=sign_b, reason=reason)
