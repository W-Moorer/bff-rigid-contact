import numpy as np

from calg.modeling.generators import make_plane_grid
from calg.core.primitives import build_primitives
from calg.solver.curved_newton import solve_curved_patch_pair
from calg.solver.response import ContactEnergyModel, evaluate_contact_response, FrictionStateStore
from calg.solver import ContactDetector
from calg.solver.tdi_ccd import TimeDependentInclusionCCD


def test_complete_active_set_curved_newton_finds_boundary_feature():
    # Two single triangles with closest points located on boundary features rather
    # than in both interiors.  Complete active-set enumeration should return an
    # edge/vertex feature and a finite threshold contact.
    a = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=1, z=0.06, normal=(0, 0, -1), name="b")
    # Shift B laterally so that only a boundary portion is close to A.
    b = b.transformed(t=np.array([1.05, 0.0, 0.0]), name="b_shifted")
    pa = build_primitives(a, contact_margin=0.12)[0]
    pb = build_primitives(b, contact_margin=0.12)[0]
    r = solve_curved_patch_pair(pa, pb, d_hat=0.12, enumerate_active_sets=True)
    assert r.valid
    assert r.contact
    assert "edge" in r.feature or "vertex" in r.feature
    assert np.isfinite(r.gap)


def test_stateful_friction_projects_to_coulomb_limit():
    a = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a")
    b = make_plane_grid(size=1.0, n=1, z=0.08, normal=(0, 0, -1), name="b")
    det = ContactDetector(d_hat=0.12, mu_min=0.1).detect(a, b)
    contacts = det.contacts[:1]
    assert contacts
    model = ContactEnergyModel(d_hat=0.12, stiffness=100.0, model="ipc_barrier", friction_mu=0.2)
    state = FrictionStateStore()
    resp = evaluate_contact_response(contacts, model, [np.array([10.0, 0.0, 0.0])], friction_state=state, dt=0.1, tangent_stiffness=1e5)
    w = resp.wrenches[0]
    assert w.friction_mode in ("stick", "slip")
    assert np.linalg.norm(w.friction_on_a) <= model.friction_mu * w.normal_lambda + 1e-8
    assert resp.total_friction_energy >= 0.0


def test_tdi_gate_is_conservative_for_crossing_planes():
    a0 = make_plane_grid(size=1.0, n=1, z=0.0, normal=(0, 0, 1), name="a0")
    a1 = a0.copy(name="a1")
    b0 = make_plane_grid(size=1.0, n=1, z=0.30, normal=(0, 0, -1), name="b0")
    b1 = make_plane_grid(size=1.0, n=1, z=-0.02, normal=(0, 0, -1), name="b1")
    ccd = TimeDependentInclusionCCD(d_hat=0.10, detector=ContactDetector(d_hat=0.10, mu_min=0.1), time_tol=2e-3, max_depth=24)
    r = ccd.detect(a0, a1, b0, b1)
    assert r.impact
    assert not r.certified_no_impact
    assert r.safe_fraction < 1.0
