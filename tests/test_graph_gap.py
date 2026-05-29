import numpy as np

from calg.modeling.generators import make_plane_grid, make_uv_sphere
from calg.core.primitives import build_primitives
from calg.solver.contact_frame import build_contact_frame, graphability_certificate
from calg.solver.graph_gap import solve_graph_gap


def test_graph_gap_parallel_planes():
    a = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=1, z=0.1, normal=(0, 0, -1), name="b")
    pa = build_primitives(a, contact_margin=0.2)[0]
    pb = build_primitives(b, contact_margin=0.2)[0]
    frame = build_contact_frame(pa, pb)
    graph = graphability_certificate(pa, pb, frame, mu_min=0.1, two_sided=True)
    assert graph.passed
    res = solve_graph_gap(pa, pb, frame, graph, d_hat=0.2)
    assert res.valid
    assert res.contact
    assert abs(res.gap - 0.1) < 1e-10


def test_graph_gap_rejects_inconsistent_quadratic_fit():
    sphere = make_uv_sphere(radius=1.0, center=(0, 0, 1.08), n_lat=5, n_lon=10, name="sphere")
    plane = make_plane_grid(size=2.4, n=2, z=0.0, normal=(0, 0, 1), name="plane")
    ps = build_primitives(sphere, contact_margin=0.15)
    pp = build_primitives(plane, contact_margin=0.15)
    pa = min(ps, key=lambda p: p.vertices[:, 2].min())
    pb = min(pp, key=lambda p: np.linalg.norm(p.centroid[:2]))
    frame = build_contact_frame(pa, pb)
    graph = graphability_certificate(pa, pb, frame, mu_min=0.1, two_sided=True)

    assert graph.passed
    res = solve_graph_gap(pa, pb, frame, graph, d_hat=0.15)
    assert not res.valid
    assert "quadratic graph fit residual too high" in res.reason
