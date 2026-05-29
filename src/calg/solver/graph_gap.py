from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from calg.core.math_utils import (
    clip_polygon_by_convex_polygon,
    point_in_convex_polygon,
    polygon_signed_area,
    ensure_ccw,
    normalize,
)
from calg.core.primitives import CurvedJetPrimitive
from .contact_frame import ContactFrame, Graphability


@dataclass
class QuadraticHeight:
    coeff: np.ndarray  # [1, x, y, x^2, xy, y^2]
    projection: np.ndarray  # 3x2 projected triangle vertices
    heights: np.ndarray
    height_residual_rms: float = 0.0
    gradient_residual_rms: float = 0.0
    max_height_residual: float = 0.0
    max_gradient_residual: float = 0.0

    def value(self, xi: np.ndarray) -> float:
        x, y = float(xi[0]), float(xi[1])
        c = self.coeff
        return float(c[0] + c[1] * x + c[2] * y + c[3] * x * x + c[4] * x * y + c[5] * y * y)

    def grad(self, xi: np.ndarray) -> np.ndarray:
        x, y = float(xi[0]), float(xi[1])
        c = self.coeff
        return np.array([c[1] + 2 * c[3] * x + c[4] * y, c[2] + c[4] * x + 2 * c[5] * y], dtype=float)


def _basis(xi: np.ndarray) -> np.ndarray:
    x, y = float(xi[0]), float(xi[1])
    return np.array([1.0, x, y, x * x, x * y, y * y])


def _basis_dx(xi: np.ndarray) -> np.ndarray:
    x, y = float(xi[0]), float(xi[1])
    return np.array([0.0, 1.0, 0.0, 2 * x, y, 0.0])


def _basis_dy(xi: np.ndarray) -> np.ndarray:
    x, y = float(xi[0]), float(xi[1])
    return np.array([0.0, 0.0, 1.0, 0.0, x, 2 * y])


def fit_quadratic_height(
    primitive: CurvedJetPrimitive,
    frame: ContactFrame,
    normal_sign: float,
    height_weight: float = 20.0,
    gradient_weight: float = 1.0,
) -> QuadraticHeight:
    xis = []
    hs = []
    for v in primitive.vertices:
        xi, h = frame.project_xyh(v)
        xis.append(xi)
        hs.append(h)
    xis = np.array(xis, dtype=float)
    hs = np.array(hs, dtype=float)
    A = []
    b = []
    for xi, h in zip(xis, hs):
        A.append(height_weight * _basis(xi))
        b.append(height_weight * h)
    # Convert graph normal into gradients. For graph X=(x,y,h), normal with positive dot n is [-hx,-hy,1].
    for xi, n0 in zip(xis, primitive.vertex_normals):
        n = normal_sign * normalize(n0)
        nd = np.dot(n, frame.n)
        if abs(nd) < 1e-8:
            continue
        gx = -np.dot(n, frame.t1) / nd
        gy = -np.dot(n, frame.t2) / nd
        A.append(gradient_weight * _basis_dx(xi))
        b.append(gradient_weight * gx)
        A.append(gradient_weight * _basis_dy(xi))
        b.append(gradient_weight * gy)
    A = np.vstack(A)
    b = np.asarray(b)
    coeff, *_ = np.linalg.lstsq(A, b, rcond=None)
    q = QuadraticHeight(coeff=coeff, projection=xis, heights=hs)

    height_residuals = []
    gradient_residuals = []
    for xi, h, n0 in zip(xis, hs, primitive.vertex_normals):
        height_residuals.append(q.value(xi) - float(h))
        n = normal_sign * normalize(n0)
        nd = np.dot(n, frame.n)
        if abs(nd) < 1e-8:
            continue
        target_grad = np.array(
            [
                -np.dot(n, frame.t1) / nd,
                -np.dot(n, frame.t2) / nd,
            ],
            dtype=float,
        )
        gradient_residuals.extend((q.grad(xi) - target_grad).tolist())

    hr = np.asarray(height_residuals, dtype=float)
    gr = np.asarray(gradient_residuals, dtype=float)
    q.height_residual_rms = float(np.sqrt(np.mean(hr * hr))) if hr.size else 0.0
    q.gradient_residual_rms = float(np.sqrt(np.mean(gr * gr))) if gr.size else 0.0
    q.max_height_residual = float(np.max(np.abs(hr))) if hr.size else 0.0
    q.max_gradient_residual = float(np.max(np.abs(gr))) if gr.size else 0.0
    return q


@dataclass
class GraphGapResult:
    valid: bool
    contact: bool
    xi: np.ndarray | None
    gap: float
    point_a: np.ndarray | None
    point_b: np.ndarray | None
    normal: np.ndarray | None
    overlap_area: float
    reason: str


def _evaluate_gap(cdiff: np.ndarray, xi: np.ndarray) -> float:
    return float(_basis(xi) @ cdiff)


def _grad_gap(cdiff: np.ndarray, xi: np.ndarray) -> np.ndarray:
    return np.array([_basis_dx(xi) @ cdiff, _basis_dy(xi) @ cdiff])


def _minimize_quadratic_over_convex_polygon(cdiff: np.ndarray, poly: np.ndarray) -> tuple[np.ndarray, float]:
    candidates: list[np.ndarray] = []
    poly = ensure_ccw(poly)
    for p in poly:
        candidates.append(p.copy())
    # Interior stationary point, if unique.
    H = np.array([[2 * cdiff[3], cdiff[4]], [cdiff[4], 2 * cdiff[5]]], dtype=float)
    g0 = np.array([cdiff[1], cdiff[2]], dtype=float)
    if abs(np.linalg.det(H)) > 1e-12:
        xi = -np.linalg.solve(H, g0)
        if point_in_convex_polygon(xi, poly, tol=1e-9):
            candidates.append(xi)
    # Edge stationary points.
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        d = b - a
        # q(t)=g(a+t d); derivative is grad(q) dot d. Since q is quadratic, derivative linear.
        g_a = _grad_gap(cdiff, a)
        H_d = H @ d
        denom = float(d @ H_d)
        numer = float(g_a @ d)
        if abs(denom) > 1e-14:
            t = -numer / denom
            if 0.0 <= t <= 1.0:
                candidates.append(a + t * d)
    if not candidates:
        return np.zeros(2), float("inf")
    best_xi = candidates[0]
    best_val = _evaluate_gap(cdiff, best_xi)
    for xi in candidates[1:]:
        val = _evaluate_gap(cdiff, xi)
        if val < best_val:
            best_xi = xi
            best_val = val
    return best_xi, float(best_val)


def solve_graph_gap(
    pa: CurvedJetPrimitive,
    pb: CurvedJetPrimitive,
    frame: ContactFrame,
    graph: Graphability,
    d_hat: float,
    area_tol: float = 1e-14,
    fit_height_tol: float = 1e-8,
    fit_gradient_tol: float = 1e-6,
) -> GraphGapResult:
    qa = fit_quadratic_height(pa, frame, graph.sign_a)
    qb = fit_quadratic_height(pb, frame, graph.sign_b)
    fit_height_residual = max(qa.max_height_residual, qb.max_height_residual)
    fit_gradient_residual = max(qa.max_gradient_residual, qb.max_gradient_residual)
    if fit_height_residual > fit_height_tol or fit_gradient_residual > fit_gradient_tol:
        reason = (
            "quadratic graph fit residual too high: "
            f"height={fit_height_residual:.3g}, gradient={fit_gradient_residual:.3g}, "
            f"tol=({fit_height_tol:.3g},{fit_gradient_tol:.3g})"
        )
        return GraphGapResult(False, False, None, float("inf"), None, None, None, 0.0, reason)
    poly_a = ensure_ccw(qa.projection)
    poly_b = ensure_ccw(qb.projection)
    if abs(polygon_signed_area(poly_a)) < area_tol or abs(polygon_signed_area(poly_b)) < area_tol:
        return GraphGapResult(False, False, None, float("inf"), None, None, None, 0.0, "degenerate projected triangle")
    overlap = clip_polygon_by_convex_polygon(poly_a, poly_b)
    if len(overlap) < 3 or abs(polygon_signed_area(overlap)) < area_tol:
        return GraphGapResult(False, False, None, float("inf"), None, None, None, 0.0, "no projected overlap")
    cdiff = qb.coeff - qa.coeff
    xi, gap = _minimize_quadratic_over_convex_polygon(cdiff, overlap)
    h_a = qa.value(xi)
    h_b = qb.value(xi)
    point_a = frame.lift(xi, h_a)
    point_b = frame.lift(xi, h_b)
    # Normal from A to B corrected by average height gradient.
    g_avg = 0.5 * (qa.grad(xi) + qb.grad(xi))
    normal = normalize(frame.n - g_avg[0] * frame.t1 - g_avg[1] * frame.t2, frame.n)
    contact = bool(gap <= d_hat)
    return GraphGapResult(True, contact, xi, gap, point_a, point_b, normal, abs(polygon_signed_area(overlap)), "graph gap solved")
