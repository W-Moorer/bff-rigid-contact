from __future__ import annotations

import numpy as np

from .mesh import TriangleMesh
from calg.core.math_utils import normalize


def subdivide_midpoint(mesh: TriangleMesh, name: str | None = None) -> TriangleMesh:
    """One level of midpoint subdivision for triangle meshes.

    The operation preserves the PL surface exactly.  It is used to evaluate the
    computational cost of global subdivision contact baselines.  Analytic
    convergence experiments regenerate analytical meshes instead of relying on
    midpoint subdivision for geometry improvement.
    """
    vertices = [v.copy() for v in mesh.vertices]
    normals = [n.copy() for n in mesh.vertex_normals]
    edge_mid: dict[tuple[int, int], int] = {}

    def mid(i: int, j: int) -> int:
        key = (i, j) if i < j else (j, i)
        if key in edge_mid:
            return edge_mid[key]
        vi = mesh.vertices[i]
        vj = mesh.vertices[j]
        ni = mesh.vertex_normals[i]
        nj = mesh.vertex_normals[j]
        idx = len(vertices)
        vertices.append(0.5 * (vi + vj))
        normals.append(normalize(0.5 * (ni + nj), ni))
        edge_mid[key] = idx
        return idx

    faces: list[list[int]] = []
    for f in mesh.faces:
        a, b, c = [int(x) for x in f]
        ab = mid(a, b)
        bc = mid(b, c)
        ca = mid(c, a)
        faces.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
    return TriangleMesh(np.asarray(vertices, dtype=float), np.asarray(faces, dtype=np.int64), np.asarray(normals, dtype=float), mesh.name if name is None else name)


def face_count(mesh: TriangleMesh) -> int:
    return int(len(mesh.faces))


def vertex_count(mesh: TriangleMesh) -> int:
    return int(len(mesh.vertices))
