from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .aabb import AABB


@dataclass
class BVHNode:
    bounds: AABB
    indices: list[int]
    left: "BVHNode | None" = None
    right: "BVHNode | None" = None

    @property
    def is_leaf(self) -> bool:
        return self.left is None and self.right is None


class BVH:
    def __init__(self, boxes: Sequence[AABB], leaf_size: int = 8):
        if not boxes:
            raise ValueError("BVH requires at least one AABB")
        self.boxes = list(boxes)
        self.leaf_size = int(leaf_size)
        self.root = self._build(list(range(len(self.boxes))))

    def _bounds_for(self, indices: list[int]) -> AABB:
        b = self.boxes[indices[0]]
        out = AABB(b.lo.copy(), b.hi.copy())
        for idx in indices[1:]:
            out = out.union(self.boxes[idx])
        return out

    def _build(self, indices: list[int]) -> BVHNode:
        bounds = self._bounds_for(indices)
        if len(indices) <= self.leaf_size:
            return BVHNode(bounds=bounds, indices=indices)
        centers = np.array([self.boxes[i].center for i in indices])
        axis = int(np.argmax(centers.max(axis=0) - centers.min(axis=0)))
        indices.sort(key=lambda i: self.boxes[i].center[axis])
        mid = len(indices) // 2
        left = self._build(indices[:mid])
        right = self._build(indices[mid:])
        return BVHNode(bounds=bounds, indices=[], left=left, right=right)

    def query_pairs(self, other: "BVH") -> list[tuple[int, int]]:
        pairs: list[tuple[int, int]] = []
        stack = [(self.root, other.root)]
        while stack:
            a, b = stack.pop()
            if not a.bounds.intersects(b.bounds):
                continue
            if a.is_leaf and b.is_leaf:
                for i in a.indices:
                    for j in b.indices:
                        if self.boxes[i].intersects(other.boxes[j]):
                            pairs.append((i, j))
            elif a.is_leaf:
                if b.left is not None:
                    stack.append((a, b.left))
                if b.right is not None:
                    stack.append((a, b.right))
            elif b.is_leaf:
                if a.left is not None:
                    stack.append((a.left, b))
                if a.right is not None:
                    stack.append((a.right, b))
            else:
                for aa in (a.left, a.right):
                    for bb in (b.left, b.right):
                        if aa is not None and bb is not None:
                            stack.append((aa, bb))
        return pairs
