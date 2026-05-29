import numpy as np

from calg.core.triangle_distance import triangle_triangle_closest, closest_point_on_triangle


def test_point_triangle_closest_inside():
    tri = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    p = np.array([0.25, 0.25, 1.0])
    q, bary = closest_point_on_triangle(p, tri)
    assert np.allclose(q, [0.25, 0.25, 0.0])
    assert np.allclose(bary.sum(), 1.0)
    assert np.all(bary >= -1e-12)


def test_triangle_triangle_parallel_gap():
    a = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=float)
    b = a + np.array([0, 0, 0.1])
    cp = triangle_triangle_closest(a, b)
    assert abs(cp.distance - 0.1) < 1e-12
