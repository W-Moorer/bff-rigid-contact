import numpy as np

from calg.modeling.generators import make_plane_grid, make_uv_sphere
from calg.core.primitives import build_primitives
from calg.solver.curved_newton import solve_curved_patch_pair
from calg.solver.interval_fallback import bezier_interval_fallback


def test_curved_newton_plane_pair_gap():
    a = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=1, z=0.08, normal=(0, 0, -1), name="b")
    pa = build_primitives(a, contact_margin=0.1)[0]
    pb = build_primitives(b, contact_margin=0.1)[0]
    r = solve_curved_patch_pair(pa, pb, d_hat=0.12)
    assert r.valid
    assert r.contact
    assert abs(r.gap - 0.08) < 1e-8
    assert np.allclose(r.bary_a.sum(), 1.0)
    assert np.allclose(r.bary_b.sum(), 1.0)


def test_interval_fallback_plane_pair_gap():
    a = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=1, z=0.08, normal=(0, 0, -1), name="b")
    pa = build_primitives(a, contact_margin=0.1)[0]
    pb = build_primitives(b, contact_margin=0.1)[0]
    r = bezier_interval_fallback(pa, pb, d_hat=0.12, max_depth=3)
    assert r.valid
    assert r.contact
    assert abs(r.gap - 0.08) < 1e-6


def test_curved_newton_sphere_plane_reasonable():
    sphere = make_uv_sphere(radius=1.0, center=(0, 0, 1.08), n_lat=8, n_lon=16, name="sphere")
    plane = make_plane_grid(size=2.4, n=4, z=0.0, normal=(0, 0, 1), name="plane")
    ps = build_primitives(sphere, contact_margin=0.15)
    pp = build_primitives(plane, contact_margin=0.15)
    # Pick the lowest sphere primitive and a central plane primitive.
    pa = min(ps, key=lambda p: p.vertices[:, 2].min())
    pb = min(pp, key=lambda p: np.linalg.norm(p.centroid[:2]))
    r = solve_curved_patch_pair(pa, pb, d_hat=0.15)
    assert r.valid
    assert r.gap < 0.15
