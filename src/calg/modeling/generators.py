from __future__ import annotations

import math
import numpy as np

from .mesh import TriangleMesh
from calg.core.math_utils import normalize


def make_plane_grid(
    size: float = 2.0,
    n: int = 16,
    z: float = 0.0,
    normal: tuple[float, float, float] = (0.0, 0.0, 1.0),
    name: str = "plane",
) -> TriangleMesh:
    """Create an open square plane grid in the xy plane.

    The face orientation is chosen so that the default normal is +z.  If a
    different normal is requested, the mesh is rotated from +z to that normal.
    """
    coords = np.linspace(-size / 2.0, size / 2.0, n + 1)
    vertices = []
    for y in coords:
        for x in coords:
            vertices.append([x, y, z])
    vertices = np.array(vertices, dtype=float)
    faces = []
    def vid(i, j):
        return j * (n + 1) + i
    for j in range(n):
        for i in range(n):
            v00 = vid(i, j)
            v10 = vid(i + 1, j)
            v01 = vid(i, j + 1)
            v11 = vid(i + 1, j + 1)
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])
    faces = np.array(faces, dtype=np.int64)
    normals = np.tile(np.array([0.0, 0.0, 1.0]), (len(vertices), 1))

    target = normalize(np.array(normal, dtype=float), np.array([0.0, 0.0, 1.0]))
    src = np.array([0.0, 0.0, 1.0])
    if np.linalg.norm(target - src) > 1e-12:
        # Rodrigues rotation from src to target.
        axis = np.cross(src, target)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-12:
            R = np.diag([1.0, -1.0, -1.0])
        else:
            axis /= axis_norm
            c = float(np.dot(src, target))
            s = axis_norm
            K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            R = np.eye(3) + K * s + K @ K * (1.0 - c)
        vertices = (R @ vertices.T).T
        normals = (R @ normals.T).T
    return TriangleMesh(vertices, faces, normals, name)


def make_uv_sphere(radius: float = 1.0, center=(0.0, 0.0, 0.0), n_lat: int = 16, n_lon: int = 32, name: str = "sphere") -> TriangleMesh:
    center = np.asarray(center, dtype=float)
    vertices = []
    normals = []
    # poles
    vertices.append(center + np.array([0.0, 0.0, radius]))
    normals.append(np.array([0.0, 0.0, 1.0]))
    for i in range(1, n_lat):
        theta = math.pi * i / n_lat
        st = math.sin(theta)
        ct = math.cos(theta)
        for j in range(n_lon):
            phi = 2.0 * math.pi * j / n_lon
            n = np.array([st * math.cos(phi), st * math.sin(phi), ct])
            vertices.append(center + radius * n)
            normals.append(n)
    vertices.append(center + np.array([0.0, 0.0, -radius]))
    normals.append(np.array([0.0, 0.0, -1.0]))
    top = 0
    bottom = len(vertices) - 1

    def ring_id(i, j):
        # i = 1..n_lat-1
        return 1 + (i - 1) * n_lon + (j % n_lon)

    faces = []
    # top cap orientation outward
    for j in range(n_lon):
        faces.append([top, ring_id(1, j), ring_id(1, j + 1)])
    # bands
    for i in range(1, n_lat - 1):
        for j in range(n_lon):
            a = ring_id(i, j)
            b = ring_id(i, j + 1)
            c = ring_id(i + 1, j)
            d = ring_id(i + 1, j + 1)
            faces.append([a, c, d])
            faces.append([a, d, b])
    # bottom cap
    for j in range(n_lon):
        faces.append([ring_id(n_lat - 1, j), bottom, ring_id(n_lat - 1, j + 1)])
    return TriangleMesh(np.array(vertices), np.array(faces, dtype=np.int64), np.array(normals), name)


def make_paraboloid_patch(
    size: float = 2.0,
    n: int = 24,
    a: float = 0.25,
    z0: float = 0.1,
    downward: bool = True,
    name: str = "paraboloid",
) -> TriangleMesh:
    """Create z = z0 + a(x^2+y^2). If downward is True, normals point roughly -z."""
    coords = np.linspace(-size / 2.0, size / 2.0, n + 1)
    vertices = []
    normals = []
    for y in coords:
        for x in coords:
            z = z0 + a * (x * x + y * y)
            # surface F = z - z0 - a(x^2+y^2) => normal [-2ax,-2ay,1]
            nrm = np.array([-2 * a * x, -2 * a * y, 1.0])
            if downward:
                nrm = -nrm
            vertices.append([x, y, z])
            normals.append(normalize(nrm))
    faces = []
    def vid(i, j):
        return j * (n + 1) + i
    for j in range(n):
        for i in range(n):
            v00 = vid(i, j)
            v10 = vid(i + 1, j)
            v01 = vid(i, j + 1)
            v11 = vid(i + 1, j + 1)
            if downward:
                faces.append([v00, v11, v10])
                faces.append([v00, v01, v11])
            else:
                faces.append([v00, v10, v11])
                faces.append([v00, v11, v01])
    return TriangleMesh(np.array(vertices), np.array(faces, dtype=np.int64), np.array(normals), name)


def make_open_wavy_sheet(size: float = 2.0, n: int = 24, amplitude: float = 0.05, z0: float = 0.1, name: str = "wavy_sheet") -> TriangleMesh:
    coords = np.linspace(-size / 2.0, size / 2.0, n + 1)
    vertices = []
    for y in coords:
        for x in coords:
            z = z0 + amplitude * math.sin(math.pi * x / size * 2.0) * math.cos(math.pi * y / size * 2.0)
            vertices.append([x, y, z])
    faces = []
    def vid(i, j):
        return j * (n + 1) + i
    for j in range(n):
        for i in range(n):
            v00 = vid(i, j)
            v10 = vid(i + 1, j)
            v01 = vid(i, j + 1)
            v11 = vid(i + 1, j + 1)
            faces.append([v00, v10, v11])
            faces.append([v00, v11, v01])
    return TriangleMesh(np.array(vertices), np.array(faces, dtype=np.int64), None, name)



def make_shifted_paraboloid_patch(
    size: float = 2.0,
    n: int = 8,
    a: float = 0.65,
    z0: float = 0.05,
    x0: float | None = None,
    y0: float | None = None,
    downward: bool = True,
    name: str = "shifted_paraboloid",
) -> TriangleMesh:
    """Create z = z0 + a*((x-x0)^2 + (y-y0)^2) with analytic normals.

    Unlike make_paraboloid_patch, the minimum can be shifted away from mesh
    vertices.  This is a useful under-resolved smooth-contact benchmark: a
    piecewise-linear mesh can only see vertex/edge extrema, while a curved-jet
    patch with vertex normals can reconstruct the sub-cell valley more
    accurately.

    Parameters
    ----------
    x0, y0:
        Apex position.  If omitted, the apex is placed near the center of an
        interior grid cell, avoiding alignment with refinement vertices.
    """
    coords = np.linspace(-size / 2.0, size / 2.0, n + 1)
    h = size / n
    if x0 is None:
        x0 = 0.37 * h
    if y0 is None:
        y0 = -0.29 * h
    vertices = []
    normals = []
    for y in coords:
        for x in coords:
            z = z0 + a * ((x - x0) ** 2 + (y - y0) ** 2)
            # F(x,y,z)=z-z0-a((x-x0)^2+(y-y0)^2); upward normal
            nrm = np.array([-2.0 * a * (x - x0), -2.0 * a * (y - y0), 1.0])
            if downward:
                nrm = -nrm
            vertices.append([x, y, z])
            normals.append(normalize(nrm))
    faces = []
    def vid(i, j):
        return j * (n + 1) + i
    for j in range(n):
        for i in range(n):
            v00 = vid(i, j)
            v10 = vid(i + 1, j)
            v01 = vid(i, j + 1)
            v11 = vid(i + 1, j + 1)
            if downward:
                faces.append([v00, v11, v10])
                faces.append([v00, v01, v11])
            else:
                faces.append([v00, v10, v11])
                faces.append([v00, v11, v01])
    return TriangleMesh(np.array(vertices), np.array(faces, dtype=np.int64), np.array(normals), name)


def analytic_shifted_paraboloid_normal(
    x: float,
    y: float,
    a: float,
    x0: float,
    y0: float,
    downward: bool = True,
) -> np.ndarray:
    """Analytic unit normal for make_shifted_paraboloid_patch."""
    nrm = np.array([-2.0 * a * (x - x0), -2.0 * a * (y - y0), 1.0], dtype=float)
    if downward:
        nrm = -nrm
    return normalize(nrm)

def make_cylinder_patch(radius: float = 0.5, length: float = 2.0, n_theta: int = 32, n_z: int = 12, center=(0, 0, 0), axis: str = "x", name: str = "cylinder_patch") -> TriangleMesh:
    """Open cylindrical surface patch with full circular circumference and open ends."""
    center = np.asarray(center, dtype=float)
    vertices = []
    normals = []
    xs = np.linspace(-length / 2.0, length / 2.0, n_z + 1)
    for x in xs:
        for j in range(n_theta):
            phi = 2 * math.pi * j / n_theta
            n = np.array([0.0, math.cos(phi), math.sin(phi)])
            p = np.array([x, radius * math.cos(phi), radius * math.sin(phi)])
            if axis == "z":
                p = np.array([radius * math.cos(phi), radius * math.sin(phi), x])
                n = np.array([math.cos(phi), math.sin(phi), 0.0])
            vertices.append(center + p)
            normals.append(n)
    faces = []
    def vid(i, j):
        return i * n_theta + (j % n_theta)
    for i in range(n_z):
        for j in range(n_theta):
            a = vid(i, j)
            b = vid(i, j + 1)
            c = vid(i + 1, j)
            d = vid(i + 1, j + 1)
            faces.append([a, c, d])
            faces.append([a, d, b])
    return TriangleMesh(np.array(vertices), np.array(faces, dtype=np.int64), np.array(normals), name)


def make_ruffled_sphere(
    radius: float = 1.0,
    center=(0.0, 0.0, 0.0),
    n_lat: int = 18,
    n_lon: int = 36,
    amp: float = 0.06,
    modes: tuple[int, int, int] = (3, 5, 7),
    phase: float = 0.0,
    name: str = "ruffled_sphere",
) -> TriangleMesh:
    """Closed sphere-like surface with deterministic multi-mode roughness.

    The shape is intentionally smooth but geometrically complex, useful for
    robustness statistics without relying on external CAD assets.
    """
    center = np.asarray(center, dtype=float)
    vertices = []
    normals = []
    # Use UV sphere topology with pole vertices.  Normals are radial estimates;
    # TriangleMesh can still recompute face normals if needed.
    vertices.append(center + np.array([0.0, 0.0, radius * (1.0 + amp * math.sin(phase))]))
    normals.append(np.array([0.0, 0.0, 1.0]))
    for i in range(1, n_lat):
        theta = math.pi * i / n_lat
        st = math.sin(theta)
        ct = math.cos(theta)
        for j in range(n_lon):
            phi = 2.0 * math.pi * j / n_lon
            base = np.array([st * math.cos(phi), st * math.sin(phi), ct])
            rmod = 1.0 + amp * (
                0.55 * math.sin(modes[0] * phi + phase) * math.sin(2.0 * theta)
                + 0.30 * math.cos(modes[1] * phi - 0.5 * phase) * math.sin(3.0 * theta)
                + 0.15 * math.sin(modes[2] * phi + theta)
            )
            vertices.append(center + radius * rmod * base)
            normals.append(normalize(base))
    vertices.append(center + np.array([0.0, 0.0, -radius * (1.0 - amp * math.sin(phase))]))
    normals.append(np.array([0.0, 0.0, -1.0]))
    top = 0
    bottom = len(vertices) - 1

    def ring_id(i, j):
        return 1 + (i - 1) * n_lon + (j % n_lon)

    faces = []
    for j in range(n_lon):
        faces.append([top, ring_id(1, j), ring_id(1, j + 1)])
    for i in range(1, n_lat - 1):
        for j in range(n_lon):
            a = ring_id(i, j)
            b = ring_id(i, j + 1)
            c = ring_id(i + 1, j)
            d = ring_id(i + 1, j + 1)
            faces.append([a, c, d])
            faces.append([a, d, b])
    for j in range(n_lon):
        faces.append([ring_id(n_lat - 1, j), bottom, ring_id(n_lat - 1, j + 1)])
    return TriangleMesh(np.asarray(vertices), np.asarray(faces, dtype=np.int64), np.asarray(normals), name)


def make_lobed_rotor_patch(
    lobes: int = 6,
    radius: float = 0.45,
    length: float = 1.2,
    amp: float = 0.10,
    n_theta: int = 48,
    n_z: int = 12,
    center=(0.0, 0.0, 0.0),
    axis: str = "x",
    name: str = "lobed_rotor_patch",
) -> TriangleMesh:
    """Open industrial-like lobed cylindrical surface.

    It approximates a cam/rotor-type surface and is used as a complex geometry
    stress test before importing true industrial CAD.
    """
    center = np.asarray(center, dtype=float)
    vertices = []
    normals = []
    xs = np.linspace(-length / 2.0, length / 2.0, n_z + 1)
    for x in xs:
        for j in range(n_theta):
            phi = 2.0 * math.pi * j / n_theta
            r = radius * (1.0 + amp * math.cos(lobes * phi) + 0.03 * math.sin(2.0 * math.pi * x / length))
            dr = -radius * amp * lobes * math.sin(lobes * phi)
            # surface in yz plane around x axis: p=(x,r cos phi,r sin phi)
            p = np.array([x, r * math.cos(phi), r * math.sin(phi)])
            # approximate normal from cross(p_x, p_phi)
            px = np.array([1.0, 0.0, 0.0])
            pphi = np.array([0.0, dr * math.cos(phi) - r * math.sin(phi), dr * math.sin(phi) + r * math.cos(phi)])
            n = normalize(np.cross(pphi, px), np.array([0.0, math.cos(phi), math.sin(phi)]))
            if axis == "z":
                p = np.array([r * math.cos(phi), r * math.sin(phi), x])
                n = np.array([n[1], n[2], n[0]])
            vertices.append(center + p)
            normals.append(n)
    faces = []

    def vid(i, j):
        return i * n_theta + (j % n_theta)

    for i in range(n_z):
        for j in range(n_theta):
            a = vid(i, j)
            b = vid(i, j + 1)
            c = vid(i + 1, j)
            d = vid(i + 1, j + 1)
            faces.append([a, c, d])
            faces.append([a, d, b])
    return TriangleMesh(np.asarray(vertices), np.asarray(faces, dtype=np.int64), np.asarray(normals), name)
