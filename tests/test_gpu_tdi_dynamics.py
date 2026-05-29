import numpy as np

from calg.core.aabb import AABB
from calg.gpu import query_aabb_pairs_vectorized, aabb_arrays_from_boxes
from calg.modeling.generators import make_plane_grid
from calg.solver import ContactDetector, TimeDependentInclusionCCD
from calg.dynamics import MassSpringBody, ContactPairSpec, ImplicitScene, ImplicitDynamicsSolver


def test_gpu_vectorized_broadphase_cpu_fallback():
    boxes_a = [AABB(np.array([0, 0, 0.0]), np.array([1, 1, 0.1])), AABB(np.array([3, 0, 0]), np.array([4, 1, 1]))]
    boxes_b = [AABB(np.array([0.5, 0.5, 0.0]), np.array([1.5, 1.5, 0.1])), AABB(np.array([5, 0, 0]), np.array([6, 1, 1]))]
    loa, hia = aabb_arrays_from_boxes(boxes_a)
    lob, hib = aabb_arrays_from_boxes(boxes_b)
    pairs, info = query_aabb_pairs_vectorized(loa, hia, lob, hib, prefer_gpu=True)
    assert pairs.shape[1] == 2
    assert (pairs == np.array([[0, 0]])).all()
    assert info.name in ("numpy", "cupy", "torch")


def test_tdi_ccd_certified_no_contact_for_separated_planes():
    a0 = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a0")
    a1 = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a1")
    b0 = make_plane_grid(size=1.0, n=1, z=1.0, normal=(0, 0, -1), name="b0")
    b1 = make_plane_grid(size=1.0, n=1, z=1.2, normal=(0, 0, -1), name="b1")
    ccd = TimeDependentInclusionCCD(d_hat=0.1, detector=ContactDetector(d_hat=0.1, mu_min=0.1), time_tol=1e-4)
    r = ccd.detect(a0, a1, b0, b1)
    assert r.certified_no_impact
    assert not r.impact
    assert r.safe_fraction == 1.0


def test_tdi_ccd_blocks_moving_plane_before_penetration():
    a0 = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a0")
    a1 = a0.copy(name="a1")
    b0 = make_plane_grid(size=1.0, n=1, z=0.30, normal=(0, 0, -1), name="b0")
    b1 = make_plane_grid(size=1.0, n=1, z=0.02, normal=(0, 0, -1), name="b1")
    ccd = TimeDependentInclusionCCD(d_hat=0.10, detector=ContactDetector(d_hat=0.10, mu_min=0.1), time_tol=5e-3, max_depth=20)
    r = ccd.detect(a0, a1, b0, b1)
    assert r.impact
    assert not r.certified_no_impact
    assert r.toi_lower is not None and r.toi_upper is not None
    expected = (0.30 - 0.10) / (0.30 - 0.02)
    assert r.toi_lower <= expected + 0.05
    assert r.toi_upper >= expected - 0.05


def test_implicit_dynamics_global_solve_with_ccd_gate():
    mover_mesh = make_plane_grid(size=1.0, n=1, z=0.25, normal=(0, 0, 1), name="mover")
    fixed_mesh = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="fixed")
    mover = MassSpringBody(mover_mesh, velocity=np.tile(np.array([0.0, 0.0, -1.0]), (len(mover_mesh.vertices), 1)), spring_stiffness=1.0, name="mover")
    fixed = MassSpringBody(fixed_mesh, fixed=np.ones(len(fixed_mesh.vertices), dtype=bool), spring_stiffness=0.0, name="fixed")
    scene = ImplicitScene(
        bodies=[mover, fixed],
        contact_pairs=[ContactPairSpec(0, 1, d_hat=0.08, stiffness=20.0, detector=ContactDetector(d_hat=0.08, mu_min=0.1, enable_interval_fallback=True))],
        gravity=np.array([0.0, 0.0, 0.0]),
    )
    solver = ImplicitDynamicsSolver(max_iter=8, grad_tol=1e-5)
    res = solver.step(scene, dt=0.30, use_ccd_gate=True)
    # The full unconstrained step would cross the threshold; the conservative
    # gate should clamp or otherwise keep the final reported gap outside deep penetration.
    assert res.ccd_gate is not None
    det = ContactDetector(d_hat=0.08, mu_min=0.1, enable_interval_fallback=True).detect(scene.bodies[0].mesh, scene.bodies[1].mesh)
    assert det.min_gap >= 0.075
