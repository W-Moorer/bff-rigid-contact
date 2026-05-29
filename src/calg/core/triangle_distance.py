from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .math_utils import clamp, dot, norm


@dataclass
class ClosestPair:
    point_a: np.ndarray
    point_b: np.ndarray
    distance: float
    feature: str


def closest_point_on_triangle(p: np.ndarray, tri: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Closest point and barycentric coordinates on triangle.

    Ericson-style region tests.
    """
    a, b, c = tri
    ab = b - a
    ac = c - a
    ap = p - a
    d1 = dot(ab, ap)
    d2 = dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a.copy(), np.array([1.0, 0.0, 0.0])

    bp = p - b
    d3 = dot(ab, bp)
    d4 = dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b.copy(), np.array([0.0, 1.0, 0.0])

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return a + v * ab, np.array([1.0 - v, v, 0.0])

    cp = p - c
    d5 = dot(ab, cp)
    d6 = dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c.copy(), np.array([0.0, 0.0, 1.0])

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return a + w * ac, np.array([1.0 - w, 0.0, w])

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + w * (c - b), np.array([0.0, 1.0 - w, w])

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    u = 1.0 - v - w
    return u * a + v * b + w * c, np.array([u, v, w])


def closest_points_on_segments(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> tuple[np.ndarray, np.ndarray, float, float]:
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = dot(d1, d1)
    e = dot(d2, d2)
    f = dot(d2, r)
    if a <= 1e-15 and e <= 1e-15:
        return p1.copy(), p2.copy(), 0.0, 0.0
    if a <= 1e-15:
        s = 0.0
        t = clamp(f / e, 0.0, 1.0)
    else:
        c = dot(d1, r)
        if e <= 1e-15:
            t = 0.0
            s = clamp(-c / a, 0.0, 1.0)
        else:
            b = dot(d1, d2)
            denom = a * e - b * b
            if denom != 0.0:
                s = clamp((b * f - c * e) / denom, 0.0, 1.0)
            else:
                s = 0.0
            tnom = b * s + f
            if tnom < 0.0:
                t = 0.0
                s = clamp(-c / a, 0.0, 1.0)
            elif tnom > e:
                t = 1.0
                s = clamp((b - c) / a, 0.0, 1.0)
            else:
                t = tnom / e
    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return c1, c2, s, t


def triangle_triangle_closest(tri_a: np.ndarray, tri_b: np.ndarray) -> ClosestPair:
    best = ClosestPair(tri_a[0].copy(), tri_b[0].copy(), float("inf"), "init")

    def update(pa, pb, feature):
        nonlocal best
        d = norm(pa - pb)
        if d < best.distance:
            best = ClosestPair(pa.copy(), pb.copy(), d, feature)

    # vertices to opposite triangle
    for i, p in enumerate(tri_a):
        q, _ = closest_point_on_triangle(p, tri_b)
        update(p, q, f"A_vertex_{i}-B_face")
    for i, p in enumerate(tri_b):
        q, _ = closest_point_on_triangle(p, tri_a)
        update(q, p, f"B_vertex_{i}-A_face")

    edges = ((0, 1), (1, 2), (2, 0))
    for ei, (ia, ja) in enumerate(edges):
        for ej, (ib, jb) in enumerate(edges):
            pa, pb, _, _ = closest_points_on_segments(tri_a[ia], tri_a[ja], tri_b[ib], tri_b[jb])
            update(pa, pb, f"edge_{ei}-edge_{ej}")
    return best
