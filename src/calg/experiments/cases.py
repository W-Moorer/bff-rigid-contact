from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from calg.modeling.mesh import TriangleMesh
from calg.modeling.generators import (
    make_plane_grid,
    make_uv_sphere,
    make_paraboloid_patch,
    make_open_wavy_sheet,
    make_cylinder_patch,
    make_shifted_paraboloid_patch,
)


@dataclass
class ValidationCase:
    name: str
    mesh_a: TriangleMesh
    mesh_b: TriangleMesh
    d_hat: float
    expected_gap: float | None
    description: str


def make_plane_plane_case(offset: float = 0.08, n: int = 8) -> ValidationCase:
    # A is a lower plane with upward normal, B is an upper plane with downward normal.
    a = make_plane_grid(size=1.5, n=n, z=0.0, normal=(0, 0, 1), name="plane_A")
    b = make_plane_grid(size=1.5, n=n, z=offset, normal=(0, 0, -1), name="plane_B")
    return ValidationCase("plane_plane_gap", a, b, d_hat=offset + 0.02, expected_gap=offset, description="Two parallel open planes with known normal gap.")


def make_sphere_plane_case(gap: float = 0.08, n_lat: int = 12, n_lon: int = 24, plane_n: int = 20) -> ValidationCase:
    sphere = make_uv_sphere(radius=1.0, center=(0, 0, 1.0 + gap), n_lat=n_lat, n_lon=n_lon, name="sphere")
    plane = make_plane_grid(size=2.8, n=plane_n, z=0.0, normal=(0, 0, 1), name="ground_plane")
    return ValidationCase("sphere_plane_gap", sphere, plane, d_hat=gap + 0.03, expected_gap=gap, description="Sphere over a plane with analytic gap equal to center_z-radius.")


def make_paraboloid_plane_case(gap: float = 0.06, n: int = 20) -> ValidationCase:
    para = make_paraboloid_patch(size=1.5, n=n, a=0.22, z0=gap, downward=True, name="paraboloid")
    plane = make_plane_grid(size=1.8, n=n, z=0.0, normal=(0, 0, 1), name="plane")
    return ValidationCase("paraboloid_plane_gap", para, plane, d_hat=gap + 0.03, expected_gap=gap, description="Open paraboloid patch above a plane; the minimum gap occurs at the vertex.")


def make_wavy_sheet_plane_case(gap: float = 0.04, n: int = 20) -> ValidationCase:
    sheet = make_open_wavy_sheet(size=1.5, n=n, amplitude=0.02, z0=gap + 0.02, name="wavy_sheet")
    # expected minimum z roughly gap because z0 - amplitude = gap.
    plane = make_plane_grid(size=1.8, n=n, z=0.0, normal=(0, 0, 1), name="plane")
    return ValidationCase("open_wavy_sheet_plane", sheet, plane, d_hat=gap + 0.04, expected_gap=gap, description="Open nonplanar sheet against a plane; tests topology-independent local graph contact.")


def make_cylinder_plane_case(gap: float = 0.05, n_theta: int = 24, n_z: int = 10) -> ValidationCase:
    cyl = make_cylinder_patch(radius=0.5, length=1.4, n_theta=n_theta, n_z=n_z, center=(0, 0, 0.5 + gap), axis="x", name="cylinder")
    plane = make_plane_grid(size=2.0, n=18, z=0.0, normal=(0, 0, 1), name="plane")
    return ValidationCase("cylinder_plane_line_contact", cyl, plane, d_hat=gap + 0.04, expected_gap=gap, description="Open-ended cylinder over a plane; tests line-contact style geometry.")



def make_shifted_paraboloid_plane_case(gap: float = 0.05, n: int = 6, a: float = 0.65) -> ValidationCase:
    para = make_shifted_paraboloid_patch(size=1.6, n=n, a=a, z0=gap, downward=True, name="shifted_paraboloid")
    plane = make_plane_grid(size=1.9, n=n, z=0.0, normal=(0, 0, 1), name="plane")
    return ValidationCase(
        "shifted_paraboloid_plane_gap",
        para,
        plane,
        d_hat=gap + 0.06,
        expected_gap=gap,
        description="Paraboloid whose apex lies inside a cell; tests sub-cell smooth-contact accuracy.",
    )

def default_cases() -> list[ValidationCase]:
    return [
        make_plane_plane_case(n=4),
        make_sphere_plane_case(n_lat=6, n_lon=12, plane_n=8),
        make_paraboloid_plane_case(n=8),
        make_wavy_sheet_plane_case(n=8),
        make_cylinder_plane_case(n_theta=12, n_z=4),
    ]
