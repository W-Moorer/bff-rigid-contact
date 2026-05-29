import numpy as np

from calg.modeling.generators import make_plane_grid
from calg.solver import ContactDetector, ContinuousContactDetector, ContactEnergyModel, evaluate_contact_response, assemble_nodal_forces


def test_threshold_ccd_plane_plane():
    a0 = make_plane_grid(size=1.0, n=2, z=0.0, normal=(0, 0, 1), name="a0")
    a1 = a0.copy(name="a1")
    b0 = make_plane_grid(size=1.0, n=2, z=0.20, normal=(0, 0, -1), name="b0")
    b1 = make_plane_grid(size=1.0, n=2, z=0.05, normal=(0, 0, -1), name="b1")
    ccd = ContinuousContactDetector(d_hat=0.10, detector=ContactDetector(d_hat=0.10, mu_min=0.1), samples=8, time_tol=1e-3)
    r = ccd.detect(a0, a1, b0, b1)
    assert r.impact
    assert r.toi is not None
    assert abs(r.toi - (0.20 - 0.10) / (0.20 - 0.05)) < 0.02
    assert r.contacts is not None and r.contacts.stats.contacts > 0


def test_barrier_response_and_force_assembly():
    a = make_plane_grid(size=1.0, n=2, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=2, z=0.08, normal=(0, 0, -1), name="b")
    result = ContactDetector(d_hat=0.12, mu_min=0.1).detect(a, b)
    assert result.contacts
    contacts = result.contacts[:3]
    model = ContactEnergyModel(d_hat=0.12, stiffness=10.0, model="ipc_barrier", friction_mu=0.3)
    velocities = [np.array([1.0, 0.0, 0.0]) for _ in contacts]
    resp = evaluate_contact_response(contacts, model, velocities)
    assert resp.total_energy > 0.0
    assert np.linalg.norm(resp.total_force_on_a + resp.total_force_on_b) < 1e-8
    fa, fb = assemble_nodal_forces(a, b, contacts, resp)
    assert fa.shape == a.vertices.shape
    assert fb.shape == b.vertices.shape
    assert np.linalg.norm(fa.sum(axis=0) + fb.sum(axis=0)) < 1e-8
