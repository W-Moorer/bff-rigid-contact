from calg.modeling.generators import make_plane_grid, make_uv_sphere
from calg.solver.detector import ContactDetector


def test_detector_plane_plane():
    a = make_plane_grid(size=1.0, n=2, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=2, z=0.08, normal=(0, 0, -1), name="b")
    result = ContactDetector(d_hat=0.12, mu_min=0.1).detect(a, b)
    assert result.stats.contacts > 0
    assert abs(result.min_gap - 0.08) < 1e-8
    assert result.stats.graph_passed > 0


def test_detector_sphere_plane_gap_reasonable():
    sphere = make_uv_sphere(radius=1.0, center=(0, 0, 1.08), n_lat=8, n_lon=16, name="sphere")
    plane = make_plane_grid(size=2.4, n=10, z=0.0, normal=(0, 0, 1), name="plane")
    result = ContactDetector(d_hat=0.13, mu_min=0.1).detect(sphere, plane)
    assert result.stats.contacts > 0
    assert abs(result.min_gap - 0.08) < 0.04
