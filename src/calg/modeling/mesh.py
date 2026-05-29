from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from calg.core.math_utils import normalize, triangle_area


@dataclass
class TriangleMesh:
    """Simple triangle surface mesh.

    Attributes
    ----------
    vertices:
        Array of shape (n, 3).
    faces:
        Array of shape (m, 3) with integer vertex indices.
    vertex_normals:
        Optional per-vertex normals. If omitted, area-weighted normals are computed.
    name:
        Human-readable mesh identifier.
    """

    vertices: np.ndarray
    faces: np.ndarray
    vertex_normals: np.ndarray | None = None
    name: str = "mesh"

    def __post_init__(self) -> None:
        self.vertices = np.asarray(self.vertices, dtype=float)
        self.faces = np.asarray(self.faces, dtype=np.int64)
        if self.vertices.ndim != 2 or self.vertices.shape[1] != 3:
            raise ValueError("vertices must have shape (n, 3)")
        if self.faces.ndim != 2 or self.faces.shape[1] != 3:
            raise ValueError("faces must have shape (m, 3)")
        if self.vertex_normals is None:
            self.vertex_normals = self.compute_vertex_normals()
        else:
            self.vertex_normals = np.asarray(self.vertex_normals, dtype=float)
            self.vertex_normals = np.array([normalize(n, np.array([0.0, 0.0, 1.0])) for n in self.vertex_normals])

    def copy(self, name: str | None = None) -> "TriangleMesh":
        return TriangleMesh(
            self.vertices.copy(),
            self.faces.copy(),
            self.vertex_normals.copy() if self.vertex_normals is not None else None,
            self.name if name is None else name,
        )

    def transformed(self, R: np.ndarray | None = None, t: np.ndarray | None = None, name: str | None = None) -> "TriangleMesh":
        R = np.eye(3) if R is None else np.asarray(R, dtype=float)
        t = np.zeros(3) if t is None else np.asarray(t, dtype=float)
        v = (R @ self.vertices.T).T + t
        n = (R @ self.vertex_normals.T).T if self.vertex_normals is not None else None
        return TriangleMesh(v, self.faces.copy(), n, self.name if name is None else name)

    def compute_face_normals(self) -> np.ndarray:
        n = np.zeros((len(self.faces), 3), dtype=float)
        for i, f in enumerate(self.faces):
            a, b, c = self.vertices[f]
            n[i] = normalize(np.cross(b - a, c - a), np.array([0.0, 0.0, 1.0]))
        return n

    def compute_vertex_normals(self) -> np.ndarray:
        normals = np.zeros_like(self.vertices, dtype=float)
        for f in self.faces:
            a, b, c = self.vertices[f]
            fn = np.cross(b - a, c - a)
            area2 = np.linalg.norm(fn)
            if area2 > 1e-15:
                for idx in f:
                    normals[idx] += fn
        return np.array([normalize(n, np.array([0.0, 0.0, 1.0])) for n in normals])

    def face_vertices(self, face_index: int) -> np.ndarray:
        return self.vertices[self.faces[face_index]]

    def face_vertex_normals(self, face_index: int) -> np.ndarray:
        return self.vertex_normals[self.faces[face_index]]

    def bounding_box(self) -> tuple[np.ndarray, np.ndarray]:
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def max_edge_length(self) -> float:
        max_len = 0.0
        for f in self.faces:
            pts = self.vertices[f]
            for i, j in ((0, 1), (1, 2), (2, 0)):
                max_len = max(max_len, float(np.linalg.norm(pts[i] - pts[j])))
        return max_len

    def total_area(self) -> float:
        return float(sum(triangle_area(*self.vertices[f]) for f in self.faces))

    def write_obj(self, path: str | Path, include_normals: bool = True) -> None:
        path = Path(path)
        with path.open("w", encoding="utf8") as f:
            f.write(f"# {self.name}\n")
            for v in self.vertices:
                f.write(f"v {v[0]:.17g} {v[1]:.17g} {v[2]:.17g}\n")
            if include_normals and self.vertex_normals is not None:
                for n in self.vertex_normals:
                    f.write(f"vn {n[0]:.17g} {n[1]:.17g} {n[2]:.17g}\n")
                for face in self.faces:
                    # 1-based indices; use same normal index as vertex.
                    f.write("f " + " ".join(f"{i+1}//{i+1}" for i in face) + "\n")
            else:
                for face in self.faces:
                    f.write("f " + " ".join(str(i + 1) for i in face) + "\n")

    @staticmethod
    def read_obj(path: str | Path, name: str | None = None) -> "TriangleMesh":
        path = Path(path)
        vertices: list[list[float]] = []
        normals: list[list[float]] = []
        faces: list[list[int]] = []
        with path.open("r", encoding="utf8", errors="ignore") as f:
            for line in f:
                if line.startswith("v "):
                    parts = line.split()
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif line.startswith("vn "):
                    parts = line.split()
                    normals.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif line.startswith("f "):
                    inds = []
                    for token in line.split()[1:]:
                        # accept v, v/vt, v//vn, v/vt/vn
                        idx = token.split("/")[0]
                        if not idx:
                            continue
                        vi = int(idx)
                        if vi < 0:
                            vi = len(vertices) + vi + 1
                        inds.append(vi - 1)
                    if len(inds) == 3:
                        faces.append(inds)
                    elif len(inds) > 3:
                        # fan triangulate polygons
                        for i in range(1, len(inds) - 1):
                            faces.append([inds[0], inds[i], inds[i + 1]])
        mesh_name = path.stem if name is None else name
        return TriangleMesh(np.array(vertices), np.array(faces, dtype=np.int64), None, mesh_name)
