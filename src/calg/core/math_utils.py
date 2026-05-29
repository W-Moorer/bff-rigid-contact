from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

EPS = 1.0e-12


def as_array(x, dtype=float) -> np.ndarray:
    return np.asarray(x, dtype=dtype)


def norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def normalize(v: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    v = as_array(v, float)
    n = norm(v)
    if n < EPS:
        if fallback is None:
            return np.zeros_like(v, dtype=float)
        return normalize(fallback)
    return v / n


def dot(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.dot(a, b))


def cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.cross(a, b)


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    a = normalize(a)
    b = normalize(b)
    c = max(-1.0, min(1.0, dot(a, b)))
    return float(math.acos(c))


def orthonormal_basis_from_normal(n: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = normalize(n, np.array([0.0, 0.0, 1.0]))
    # Pick the least aligned coordinate axis for numerical stability.
    if abs(n[0]) < abs(n[1]):
        a = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < abs(n[2]) else np.array([0.0, 0.0, 1.0])
    else:
        a = np.array([0.0, 1.0, 0.0]) if abs(n[1]) < abs(n[2]) else np.array([0.0, 0.0, 1.0])
    t1 = normalize(cross(a, n))
    t2 = normalize(cross(n, t1))
    return t1, t2, n


def triangle_area(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    return 0.5 * norm(cross(b - a, c - a))


def barycentric_to_point(tri: np.ndarray, bary: np.ndarray) -> np.ndarray:
    return bary[0] * tri[0] + bary[1] * tri[1] + bary[2] * tri[2]


def point_to_barycentric(p: np.ndarray, tri: np.ndarray) -> np.ndarray:
    a, b, c = tri
    v0 = b - a
    v1 = c - a
    v2 = p - a
    d00 = dot(v0, v0)
    d01 = dot(v0, v1)
    d11 = dot(v1, v1)
    d20 = dot(v2, v0)
    d21 = dot(v2, v1)
    denom = d00 * d11 - d01 * d01
    if abs(denom) < EPS:
        return np.array([1.0, 0.0, 0.0])
    v = (d11 * d20 - d01 * d21) / denom
    w = (d00 * d21 - d01 * d20) / denom
    u = 1.0 - v - w
    return np.array([u, v, w])


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def polygon_signed_area(poly: np.ndarray) -> float:
    if len(poly) < 3:
        return 0.0
    x = poly[:, 0]
    y = poly[:, 1]
    return float(0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1)))


def ensure_ccw(poly: np.ndarray) -> np.ndarray:
    if len(poly) < 3:
        return poly
    if polygon_signed_area(poly) < 0:
        return poly[::-1].copy()
    return poly


def point_in_convex_polygon(p: np.ndarray, poly: np.ndarray, tol: float = 1e-10) -> bool:
    if len(poly) < 3:
        return False
    poly = ensure_ccw(np.asarray(poly, float))
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        edge = b - a
        rel = p - a
        if edge[0] * rel[1] - edge[1] * rel[0] < -tol:
            return False
    return True


def clip_polygon_by_convex_polygon(subject: np.ndarray, clip: np.ndarray, tol: float = 1e-12) -> np.ndarray:
    """Sutherland-Hodgman clipping for two convex 2D polygons."""
    subject = ensure_ccw(np.asarray(subject, float))
    clip = ensure_ccw(np.asarray(clip, float))
    if len(subject) < 3 or len(clip) < 3:
        return np.zeros((0, 2), dtype=float)

    def inside(p, a, b):
        edge = b - a
        rel = p - a
        return edge[0] * rel[1] - edge[1] * rel[0] >= -tol

    def intersection(p1, p2, a, b):
        # line p1 + t (p2-p1) intersects line a + s (b-a)
        r = p2 - p1
        s = b - a
        denom = r[0] * s[1] - r[1] * s[0]
        if abs(denom) < tol:
            return p2.copy()
        qmp = a - p1
        t = (qmp[0] * s[1] - qmp[1] * s[0]) / denom
        return p1 + t * r

    output = subject.copy()
    for i in range(len(clip)):
        a = clip[i]
        b = clip[(i + 1) % len(clip)]
        inp = output
        if len(inp) == 0:
            break
        output_list = []
        prev = inp[-1]
        prev_inside = inside(prev, a, b)
        for curr in inp:
            curr_inside = inside(curr, a, b)
            if curr_inside:
                if not prev_inside:
                    output_list.append(intersection(prev, curr, a, b))
                output_list.append(curr)
            elif prev_inside:
                output_list.append(intersection(prev, curr, a, b))
            prev = curr
            prev_inside = curr_inside
        if output_list:
            output = np.vstack(output_list)
        else:
            output = np.zeros((0, 2), dtype=float)
    return output


@dataclass(frozen=True)
class Stats:
    values: tuple[float, ...]

    @property
    def mean(self) -> float:
        return float(np.mean(self.values)) if self.values else float("nan")

    @property
    def maximum(self) -> float:
        return float(np.max(self.values)) if self.values else float("nan")

    @property
    def minimum(self) -> float:
        return float(np.min(self.values)) if self.values else float("nan")

    @classmethod
    def from_iterable(cls, values: Iterable[float]) -> "Stats":
        return cls(tuple(float(v) for v in values))
