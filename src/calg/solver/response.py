from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable
import numpy as np

from .detector import ContactSample


@dataclass
class ContactEnergyModel:
    """Barrier/penalty/GCP-style contact response model.

    model:
      * "ipc_barrier":  -(d-d_hat)^2 log(d/d_hat) for 0<d<d_hat.
      * "quadratic_proximity": 0.5 (d_hat-d)^2 for d<d_hat.
      * "gcp_patch": same barrier density as ipc_barrier, multiplied by the
        contact patch area weight carried by ContactSample.overlap_area.
    """

    d_hat: float
    stiffness: float = 1.0
    model: str = "ipc_barrier"
    friction_mu: float = 0.0
    friction_eps: float = 1e-8
    min_gap: float = 1e-9

    def weight(self, contact: ContactSample) -> float:
        if self.model == "gcp_patch":
            return max(float(contact.overlap_area), 1.0e-12)
        return max(float(contact.overlap_area), 1.0) if contact.method == "graph2d" and contact.overlap_area > 0 else 1.0

    def density_and_derivative(self, gap: float) -> tuple[float, float]:
        d = max(float(gap), self.min_gap)
        dh = float(self.d_hat)
        if d >= dh:
            return 0.0, 0.0
        if self.model in ("ipc_barrier", "gcp_patch"):
            # IPC-style smooth barrier used in many geometric contact codes.
            # b(d)=-(d-dh)^2 log(d/dh), b(dh)=b'(dh)=0.
            diff = d - dh
            log_term = np.log(d / dh)
            b = -(diff * diff) * log_term
            db = -(2.0 * diff * log_term + (diff * diff) / d)
            return float(self.stiffness * b), float(self.stiffness * db)
        if self.model == "quadratic_proximity":
            diff = dh - d
            return float(0.5 * self.stiffness * diff * diff), float(-self.stiffness * diff)
        raise ValueError(f"unknown contact energy model: {self.model}")



@dataclass
class TangentialFrictionState:
    """Persistent tangential slip state for a contact pair.

    The state stores the regularized tangential displacement used by a
    penalty-regularized Coulomb law.  It is intentionally keyed outside the
    geometry solver so that downstream applications may choose stable contact
    identifiers appropriate for their meshes or time integrators.
    """

    displacement: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))


@dataclass
class FrictionStateStore:
    """Small persistent state table for regularized Coulomb friction."""

    states: dict[str, TangentialFrictionState] = field(default_factory=dict)

    def get(self, key: str) -> TangentialFrictionState:
        if key not in self.states:
            self.states[key] = TangentialFrictionState()
        return self.states[key]


def default_contact_key(contact: "ContactSample") -> str:
    return f"{contact.face_a}:{contact.face_b}:{contact.method}:{contact.feature}"


def _stateful_friction_force(
    contact: "ContactSample",
    state: TangentialFrictionState,
    normal: np.ndarray,
    normal_lambda: float,
    relative_velocity: np.ndarray,
    dt: float,
    mu: float,
    tangent_stiffness: float,
) -> tuple[np.ndarray, float, str]:
    if mu <= 0.0 or normal_lambda <= 0.0 or dt <= 0.0 or tangent_stiffness <= 0.0:
        state.displacement = state.displacement - float(state.displacement @ normal) * normal
        return np.zeros(3, dtype=float), 0.0, "friction_inactive"
    vt = relative_velocity - float(relative_velocity @ normal) * normal
    xi = state.displacement + dt * vt
    xi = xi - float(xi @ normal) * normal
    trial_force = -tangent_stiffness * xi
    trial_norm = float(np.linalg.norm(trial_force))
    limit = float(mu * normal_lambda)
    if trial_norm <= limit or trial_norm <= 1e-14:
        state.displacement = xi
        return trial_force, 0.5 * tangent_stiffness * float(xi @ xi), "stick"
    f = limit * trial_force / trial_norm
    # Return mapping: keep the displacement consistent with the projected force.
    state.displacement = -f / tangent_stiffness
    return f, 0.5 * tangent_stiffness * float(state.displacement @ state.displacement), "slip"


@dataclass
class ContactWrench:
    contact_index: int
    energy: float
    normal_lambda: float
    force_on_a: np.ndarray
    force_on_b: np.ndarray
    friction_on_a: np.ndarray
    friction_on_b: np.ndarray
    total_force_on_a: np.ndarray
    total_force_on_b: np.ndarray
    friction_mode: str = "none"


@dataclass
class ContactResponseResult:
    total_energy: float
    total_force_on_a: np.ndarray
    total_force_on_b: np.ndarray
    total_friction_energy: float = 0.0
    wrenches: list[ContactWrench] = field(default_factory=list)

    def nodal_forces(self, n_vertices_a: int, n_vertices_b: int, contacts: list[ContactSample]) -> tuple[np.ndarray, np.ndarray]:
        fa = np.zeros((n_vertices_a, 3), dtype=float)
        fb = np.zeros((n_vertices_b, 3), dtype=float)
        for wrench, c in zip(self.wrenches, contacts):
            if c.bary_a is None or c.bary_b is None:
                continue
            for local_i, w in enumerate(c.bary_a):
                # This helper assumes the caller later maps local face vertices;
                # use assemble_nodal_forces when mesh face arrays are available.
                pass
        return fa, fb


def evaluate_contact_response(
    contacts: Iterable[ContactSample],
    model: ContactEnergyModel,
    relative_velocities: Iterable[np.ndarray] | None = None,
    friction_state: FrictionStateStore | None = None,
    dt: float = 0.0,
    tangent_stiffness: float | None = None,
) -> ContactResponseResult:
    contacts = list(contacts)
    if relative_velocities is None:
        velocities = [np.zeros(3, dtype=float) for _ in contacts]
    else:
        velocities = [np.asarray(v, float) for v in relative_velocities]
        if len(velocities) != len(contacts):
            raise ValueError("relative_velocities must match number of contacts")

    total_energy = 0.0
    total_friction_energy = 0.0
    total_a = np.zeros(3, dtype=float)
    total_b = np.zeros(3, dtype=float)
    wrenches: list[ContactWrench] = []

    for i, (c, vrel) in enumerate(zip(contacts, velocities)):
        n = np.asarray(c.normal, dtype=float)
        nn = np.linalg.norm(n)
        if nn <= 1e-14:
            continue
        n = n / nn
        density, deriv = model.density_and_derivative(c.gap)
        weight = model.weight(c)
        energy = density * weight
        # deriv=dE/dd. For a gap measured from A to B, force on A is deriv*n.
        f_normal_a = deriv * weight * n
        f_normal_b = -f_normal_a
        lambda_n = max(0.0, -deriv * weight)

        friction_mode = "velocity_regularized"
        friction_energy = 0.0
        if friction_state is not None and dt > 0.0:
            key = default_contact_key(c)
            kt = float(tangent_stiffness if tangent_stiffness is not None else max(model.stiffness, 1.0))
            f_fric_a, friction_energy, friction_mode = _stateful_friction_force(
                c, friction_state.get(key), n, lambda_n, vrel, dt, model.friction_mu, kt
            )
        else:
            vt = vrel - float(vrel @ n) * n
            vt_norm = float(np.linalg.norm(vt))
            if model.friction_mu > 0.0 and lambda_n > 0.0 and vt_norm > 0.0:
                # relative_velocities are interpreted as v_A - v_B at the contact.
                f_fric_a = -model.friction_mu * lambda_n * vt / np.sqrt(vt_norm * vt_norm + model.friction_eps * model.friction_eps)
            else:
                f_fric_a = np.zeros(3, dtype=float)
                friction_mode = "friction_inactive"
        f_fric_b = -f_fric_a
        fa = f_normal_a + f_fric_a
        fb = f_normal_b + f_fric_b
        total_energy += energy + friction_energy
        total_friction_energy += friction_energy
        total_a += fa
        total_b += fb
        wrenches.append(
            ContactWrench(
                contact_index=i,
                energy=float(energy),
                normal_lambda=float(lambda_n),
                force_on_a=f_normal_a,
                force_on_b=f_normal_b,
                friction_on_a=f_fric_a,
                friction_on_b=f_fric_b,
                total_force_on_a=fa,
                total_force_on_b=fb,
                friction_mode=friction_mode,
            )
        )
    return ContactResponseResult(float(total_energy), total_a, total_b, float(total_friction_energy), wrenches)


def assemble_nodal_forces(mesh_a, mesh_b, contacts: list[ContactSample], response: ContactResponseResult) -> tuple[np.ndarray, np.ndarray]:
    """Distribute contact forces to mesh vertices using contact barycentrics."""
    fa = np.zeros_like(mesh_a.vertices, dtype=float)
    fb = np.zeros_like(mesh_b.vertices, dtype=float)
    for wrench in response.wrenches:
        c = contacts[wrench.contact_index]
        if c.bary_a is None or c.bary_b is None:
            continue
        face_a = mesh_a.faces[c.face_a]
        face_b = mesh_b.faces[c.face_b]
        for li, vi in enumerate(face_a):
            fa[vi] += c.bary_a[li] * wrench.total_force_on_a
        for li, vi in enumerate(face_b):
            fb[vi] += c.bary_b[li] * wrench.total_force_on_b
    return fa, fb
