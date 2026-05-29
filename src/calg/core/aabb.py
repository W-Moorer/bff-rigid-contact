from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class AABB:
    lo: np.ndarray
    hi: np.ndarray

    def __post_init__(self) -> None:
        self.lo = np.asarray(self.lo, dtype=float)
        self.hi = np.asarray(self.hi, dtype=float)

    @classmethod
    def from_points(cls, pts: np.ndarray, margin: float = 0.0) -> "AABB":
        pts = np.asarray(pts, dtype=float)
        lo = pts.min(axis=0) - margin
        hi = pts.max(axis=0) + margin
        return cls(lo, hi)

    def expanded(self, margin: float) -> "AABB":
        return AABB(self.lo - margin, self.hi + margin)

    def union(self, other: "AABB") -> "AABB":
        return AABB(np.minimum(self.lo, other.lo), np.maximum(self.hi, other.hi))

    def intersects(self, other: "AABB") -> bool:
        return bool(np.all(self.hi >= other.lo) and np.all(other.hi >= self.lo))

    @property
    def center(self) -> np.ndarray:
        return 0.5 * (self.lo + self.hi)

    @property
    def extents(self) -> np.ndarray:
        return self.hi - self.lo

    @property
    def volume(self) -> float:
        e = np.maximum(self.extents, 0.0)
        return float(e[0] * e[1] * e[2])


def aabb_distance(a: AABB, b: AABB) -> float:
    """Euclidean distance between two axis-aligned boxes; zero if they overlap."""
    delta = np.maximum(0.0, np.maximum(a.lo - b.hi, b.lo - a.hi))
    return float(np.linalg.norm(delta))


def aabb_diagonal(a: AABB) -> float:
    return float(np.linalg.norm(np.maximum(a.hi - a.lo, 0.0)))
