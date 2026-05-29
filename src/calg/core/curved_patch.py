from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .aabb import AABB
from .math_utils import (
    EPS,
    cross,
    dot,
    norm,
    normalize,
    orthonormal_basis_from_normal,
    point_to_barycentric,
    barycentric_to_point,
)
from .primitives import CurvedJetPrimitive


def _basis(xi: np.ndarray) -> np.ndarray:
    x, y = float(xi[0]), float(xi[1])
    return np.array([1.0, x, y, x * x, x * y, y * y], dtype=float)


def _basis_dx(xi: np.ndarray) -> np.ndarray:
    x, y = float(xi[0]), float(xi[1])
    return np.array([0.0, 1.0, 0.0, 2.0 * x, y, 0.0], dtype=float)


def _basis_dy(xi: np.ndarray) -> np.ndarray:
    x, y = float(xi[0]), float(xi[1])
    return np.array([0.0, 0.0, 1.0, 0.0, x, 2.0 * y], dtype=float)


def barycentric_2d(p: np.ndarray, tri: np.ndarray) -> np.ndarray:
    """Barycentric coordinates of a 2D point with respect to a 2D triangle."""
    tri3 = np.column_stack([np.asarray(tri, float), np.zeros(3)])
    p3 = np.array([float(p[0]), float(p[1]), 0.0])
    return point_to_barycentric(p3, tri3)


def point_from_barycentric_2d(tri: np.ndarray, bary: np.ndarray) -> np.ndarray:
    return bary[0] * tri[0] + bary[1] * tri[1] + bary[2] * tri[2]


def clamp_to_triangle_2d(p: np.ndarray, tri: np.ndarray) -> np.ndarray:
    """Closest point to p on a 2D triangle."""
    p = np.asarray(p, float)
    tri = np.asarray(tri, float)
    b = barycentric_2d(p, tri)
    if np.all(b >= -1e-12):
        # Keep the point itself; small negative barycentrics are roundoff.
        return point_from_barycentric_2d(tri, np.maximum(b, 0.0) / max(np.sum(np.maximum(b, 0.0)), EPS)) if np.any(b < 0) else p

    best = tri[0].copy()
    best_d = float("inf")
    for i, j in ((0, 1), (1, 2), (2, 0)):
        a, c = tri[i], tri[j]
        e = c - a
        denom = float(e @ e)
        if denom <= EPS:
            q = a.copy()
        else:
            t = float(((p - a) @ e) / denom)
            t = max(0.0, min(1.0, t))
            q = a + t * e
        d = float(np.linalg.norm(p - q))
        if d < best_d:
            best_d = d
            best = q
    return best


def subdivide_triangle_2d(tri: np.ndarray) -> list[np.ndarray]:
    a, b, c = np.asarray(tri, float)
    ab = 0.5 * (a + b)
    bc = 0.5 * (b + c)
    ca = 0.5 * (c + a)
    return [
        np.array([a, ab, ca], dtype=float),
        np.array([ab, b, bc], dtype=float),
        np.array([ca, bc, c], dtype=float),
        np.array([ab, bc, ca], dtype=float),
    ]


@dataclass
class QuadraticPatch3D:
    """Quadratic normal-lifted surface patch over a triangle-local plane.

    The patch is intentionally local: no global atlas, no disk topology, and no
    manually prescribed cut graph are required.  It is fitted from the triangle
    vertices and vertex normals of a CurvedJetPrimitive.  The domain is the
    original triangle projected into the local patch plane.
    """

    origin: np.ndarray
    t1: np.ndarray
    t2: np.ndarray
    n0: np.ndarray
    coeff: np.ndarray
    domain: np.ndarray  # shape (3, 2), local plane coordinates of vertices
    error_bound: float = 0.0
    primitive_id: tuple[str, int] | None = None

    @classmethod
    def from_primitive(
        cls,
        primitive: CurvedJetPrimitive,
        height_weight: float = 20.0,
        gradient_weight: float = 1.0,
    ) -> "QuadraticPatch3D":
        v = primitive.vertices
        origin = v[0].copy()
        n0 = normalize(primitive.face_normal, normalize(np.mean(primitive.vertex_normals, axis=0), np.array([0.0, 0.0, 1.0])))
        e01 = v[1] - v[0]
        if norm(e01) <= EPS:
            t1, t2, n0 = orthonormal_basis_from_normal(n0)
        else:
            t1 = normalize(e01)
            # Ensure t1 lies in the plane perpendicular to n0.
            t1 = normalize(t1 - dot(t1, n0) * n0, t1)
            t2 = normalize(cross(n0, t1), np.array([0.0, 1.0, 0.0]))
            # Recompute t1 to get an orthonormal right-handed basis.
            t1 = normalize(cross(t2, n0), t1)

        xis = []
        hs = []
        for x in v:
            r = x - origin
            xis.append(np.array([dot(t1, r), dot(t2, r)], dtype=float))
            hs.append(dot(n0, r))
        xis = np.array(xis, dtype=float)
        hs = np.array(hs, dtype=float)

        A: list[np.ndarray] = []
        b: list[float] = []
        for xi, h in zip(xis, hs):
            A.append(height_weight * _basis(xi))
            b.append(height_weight * float(h))

        for xi, n in zip(xis, primitive.vertex_normals):
            n = normalize(n, n0)
            # Orient the vertex normal consistently with the patch plane.
            if dot(n, n0) < 0.0:
                n = -n
            nd = dot(n, n0)
            if abs(nd) < 1e-8:
                continue
            gx = -dot(n, t1) / nd
            gy = -dot(n, t2) / nd
            A.append(gradient_weight * _basis_dx(xi))
            b.append(gradient_weight * float(gx))
            A.append(gradient_weight * _basis_dy(xi))
            b.append(gradient_weight * float(gy))

        amat = np.vstack(A)
        rhs = np.asarray(b, dtype=float)
        coeff, *_ = np.linalg.lstsq(amat, rhs, rcond=None)
        return cls(
            origin=origin,
            t1=t1,
            t2=t2,
            n0=n0,
            coeff=coeff,
            domain=xis,
            error_bound=float(primitive.error_bound),
            primitive_id=(primitive.mesh_name, int(primitive.face_index)),
        )

    def height(self, xi: np.ndarray) -> float:
        return float(_basis(np.asarray(xi, float)) @ self.coeff)

    def grad_height(self, xi: np.ndarray) -> np.ndarray:
        xi = np.asarray(xi, float)
        return np.array([_basis_dx(xi) @ self.coeff, _basis_dy(xi) @ self.coeff], dtype=float)

    def hessian_height(self) -> np.ndarray:
        return np.array([[2.0 * self.coeff[3], self.coeff[4]], [self.coeff[4], 2.0 * self.coeff[5]]], dtype=float)

    def eval(self, xi: np.ndarray) -> np.ndarray:
        xi = np.asarray(xi, float)
        return self.origin + xi[0] * self.t1 + xi[1] * self.t2 + self.height(xi) * self.n0

    def derivatives(self, xi: np.ndarray) -> np.ndarray:
        """Return a 3x2 Jacobian dX/d(xi)."""
        g = self.grad_height(xi)
        return np.column_stack([self.t1 + g[0] * self.n0, self.t2 + g[1] * self.n0])

    def normal(self, xi: np.ndarray) -> np.ndarray:
        J = self.derivatives(xi)
        return normalize(np.cross(J[:, 0], J[:, 1]), self.n0)

    def clamp(self, xi: np.ndarray) -> np.ndarray:
        return clamp_to_triangle_2d(np.asarray(xi, float), self.domain)

    def project_to_domain(self, x: np.ndarray) -> np.ndarray:
        r = np.asarray(x, float) - self.origin
        xi = np.array([dot(self.t1, r), dot(self.t2, r)], dtype=float)
        return self.clamp(xi)

    def barycentric(self, xi: np.ndarray) -> np.ndarray:
        b = barycentric_2d(np.asarray(xi, float), self.domain)
        # Clip gently for downstream force assembly.
        bc = np.maximum(b, 0.0)
        s = float(np.sum(bc))
        return bc / s if s > EPS else np.array([1.0, 0.0, 0.0])

    def bezier_control_points_for_subtriangle(self, sub_tri: np.ndarray) -> np.ndarray:
        """Exact quadratic Bezier control hull for this patch over sub_tri.

        For X(x,y)=linear_base(x,y)+q(x,y)n0, the base coordinates are linear
        and q is quadratic.  The six quadratic triangular Bezier control points
        are therefore obtained from the vertex heights and midpoint heights.
        """
        a, b, c = np.asarray(sub_tri, float)
        verts = [a, b, c]
        h = [self.height(p) for p in verts]
        mids = [0.5 * (a + b), 0.5 * (b + c), 0.5 * (c + a)]
        hm = [self.height(p) for p in mids]
        # Edge Bezier controls from midpoint interpolation:
        # q(mid)=1/4 q_i + 1/2 q_ij + 1/4 q_j.
        hij = [2.0 * hm[0] - 0.5 * (h[0] + h[1]), 2.0 * hm[1] - 0.5 * (h[1] + h[2]), 2.0 * hm[2] - 0.5 * (h[2] + h[0])]

        def lift_base(xi: np.ndarray, hv: float) -> np.ndarray:
            return self.origin + xi[0] * self.t1 + xi[1] * self.t2 + hv * self.n0

        controls = [
            lift_base(a, h[0]),
            lift_base(b, h[1]),
            lift_base(c, h[2]),
            lift_base(mids[0], hij[0]),
            lift_base(mids[1], hij[1]),
            lift_base(mids[2], hij[2]),
        ]
        return np.asarray(controls, dtype=float)

    def aabb_for_subtriangle(self, sub_tri: np.ndarray, margin: float = 0.0) -> AABB:
        controls = self.bezier_control_points_for_subtriangle(sub_tri)
        return AABB.from_points(controls, margin=margin + self.error_bound)

    def evaluated_triangle(self, sub_tri: np.ndarray) -> np.ndarray:
        return np.asarray([self.eval(p) for p in np.asarray(sub_tri, float)], dtype=float)

    def centroid_xi(self, sub_tri: np.ndarray | None = None) -> np.ndarray:
        tri = self.domain if sub_tri is None else np.asarray(sub_tri, float)
        return np.mean(tri, axis=0)

    def domain_diameter(self, sub_tri: np.ndarray | None = None) -> float:
        tri = self.domain if sub_tri is None else np.asarray(sub_tri, float)
        return float(max(np.linalg.norm(tri[1] - tri[0]), np.linalg.norm(tri[2] - tri[1]), np.linalg.norm(tri[0] - tri[2])))

# Backward-compatible extension methods used by the v0.5 active-set Newton solver.
def _quad_second_derivatives(self: QuadraticPatch3D) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    Hh = self.hessian_height()
    return Hh[0, 0] * self.n0, Hh[0, 1] * self.n0, Hh[1, 1] * self.n0


def _quad_second_directional(self: QuadraticPatch3D, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    x_xx, x_xy, x_yy = self.second_derivatives()
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return a[0] * b[0] * x_xx + (a[0] * b[1] + a[1] * b[0]) * x_xy + a[1] * b[1] * x_yy


QuadraticPatch3D.second_derivatives = _quad_second_derivatives  # type: ignore[attr-defined]
QuadraticPatch3D.second_directional = _quad_second_directional  # type: ignore[attr-defined]
