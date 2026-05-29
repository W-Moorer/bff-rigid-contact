from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from calg.modeling.mesh import TriangleMesh
from .aabb import AABB
from .math_utils import normalize, norm, dot, angle_between, triangle_area


@dataclass
class NormalCone:
    axis: np.ndarray
    angle: float

    def min_dot_with(self, direction: np.ndarray) -> float:
        direction = normalize(direction)
        center_dot = max(-1.0, min(1.0, dot(self.axis, direction)))
        theta = np.arccos(center_dot)
        # minimum dot over cone to direction is cos(theta + cone_angle), clipped.
        return float(np.cos(min(np.pi, theta + self.angle)))

    def can_align_with(self, direction: np.ndarray, mu: float) -> bool:
        return self.min_dot_with(direction) >= mu


@dataclass
class CurvedJetPrimitive:
    """Per-face curved contact primitive.

    The present implementation stores a triangle, vertex normals, a normal cone,
    curvature/error bounds, and an inflated AABB.  The graph-gap solver then fits
    a local quadratic height jet in the contact-aligned frame on demand.
    """

    mesh_name: str
    face_index: int
    vertices: np.ndarray
    vertex_normals: np.ndarray
    normal_cone: NormalCone
    curvature_bound: float
    error_bound: float
    aabb: AABB

    @property
    def centroid(self) -> np.ndarray:
        return self.vertices.mean(axis=0)

    @property
    def face_normal(self) -> np.ndarray:
        a, b, c = self.vertices
        return normalize(np.cross(b - a, c - a), self.vertex_normals.mean(axis=0))

    @property
    def max_edge_length(self) -> float:
        v = self.vertices
        return max(norm(v[1] - v[0]), norm(v[2] - v[1]), norm(v[0] - v[2]))


def estimate_curvature_from_vertex_normals(vertices: np.ndarray, normals: np.ndarray) -> float:
    max_k = 0.0
    edges = ((0, 1), (1, 2), (2, 0))
    for i, j in edges:
        length = norm(vertices[i] - vertices[j])
        if length > 1e-12:
            max_k = max(max_k, angle_between(normals[i], normals[j]) / length)
    return float(max_k)


def make_normal_cone(normals: np.ndarray) -> NormalCone:
    axis = normalize(np.mean(normals, axis=0), normals[0])
    angle = 0.0
    for n in normals:
        angle = max(angle, angle_between(axis, n))
    return NormalCone(axis=axis, angle=float(angle))


def build_primitives(mesh: TriangleMesh, contact_margin: float = 0.0, error_safety: float = 1.5) -> list[CurvedJetPrimitive]:
    primitives: list[CurvedJetPrimitive] = []
    for fi, face in enumerate(mesh.faces):
        vertices = mesh.vertices[face]
        normals = mesh.vertex_normals[face]
        normals = np.array([normalize(n, np.array([0, 0, 1.0])) for n in normals])
        kappa = estimate_curvature_from_vertex_normals(vertices, normals)
        h = max(np.linalg.norm(vertices[1] - vertices[0]), np.linalg.norm(vertices[2] - vertices[1]), np.linalg.norm(vertices[0] - vertices[2]))
        # A conservative curve-vs-chord sagitta-like estimate.  It is zero on planar areas.
        eps = error_safety * kappa * h * h / 8.0
        cone = make_normal_cone(normals)
        aabb = AABB.from_points(vertices, margin=contact_margin + eps)
        primitives.append(
            CurvedJetPrimitive(
                mesh_name=mesh.name,
                face_index=int(fi),
                vertices=vertices.copy(),
                vertex_normals=normals.copy(),
                normal_cone=cone,
                curvature_bound=kappa,
                error_bound=eps,
                aabb=aabb,
            )
        )
    return primitives
