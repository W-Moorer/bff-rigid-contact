from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np

from calg.modeling.mesh import TriangleMesh
from calg.solver.detector import ContactDetector
from calg.solver.response import ContactEnergyModel, evaluate_contact_response, assemble_nodal_forces
from calg.solver.tdi_ccd import TimeDependentInclusionCCD, TimeDependentInclusionCCDResult


def _unique_edges(faces: np.ndarray) -> np.ndarray:
    edges = set()
    for f in np.asarray(faces, dtype=np.int64):
        for i, j in ((int(f[0]), int(f[1])), (int(f[1]), int(f[2])), (int(f[2]), int(f[0]))):
            if i > j:
                i, j = j, i
            edges.add((i, j))
    return np.asarray(sorted(edges), dtype=np.int64)


@dataclass
class MassSpringBody:
    """Surface-mesh body for the global implicit prototype.

    The body uses the triangle mesh as both rendering/contact geometry and a
    simple edge-spring elastic model.  It is intentionally lightweight: the goal
    is to exercise a full global implicit solve with contact, not to replace a
    production FEM backend.
    """

    mesh: TriangleMesh
    velocity: np.ndarray | None = None
    mass: np.ndarray | None = None
    fixed: np.ndarray | None = None
    spring_stiffness: float = 0.0
    damping: float = 0.0
    name: str = "body"
    edges: np.ndarray = field(init=False)
    rest_lengths: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        n = len(self.mesh.vertices)
        self.name = self.mesh.name if self.name == "body" else self.name
        self.velocity = np.zeros((n, 3), dtype=float) if self.velocity is None else np.asarray(self.velocity, dtype=float)
        self.mass = np.ones(n, dtype=float) if self.mass is None else np.asarray(self.mass, dtype=float)
        self.fixed = np.zeros(n, dtype=bool) if self.fixed is None else np.asarray(self.fixed, dtype=bool)
        if self.velocity.shape != self.mesh.vertices.shape:
            raise ValueError("velocity must have shape (n,3)")
        if self.mass.shape != (n,):
            raise ValueError("mass must have shape (n,)")
        if self.fixed.shape != (n,):
            raise ValueError("fixed must have shape (n,)")
        self.edges = _unique_edges(self.mesh.faces)
        x = self.mesh.vertices
        if len(self.edges):
            self.rest_lengths = np.linalg.norm(x[self.edges[:, 1]] - x[self.edges[:, 0]], axis=1)
        else:
            self.rest_lengths = np.zeros(0, dtype=float)
        # Fixed vertices are assigned infinite mass semantics via zero inverse mass.
        self.mass = np.maximum(self.mass, 1e-12)

    def copy_with_vertices(self, vertices: np.ndarray) -> "MassSpringBody":
        m = TriangleMesh(np.asarray(vertices, float), self.mesh.faces.copy(), None, self.mesh.name)
        out = MassSpringBody(m, self.velocity.copy(), self.mass.copy(), self.fixed.copy(), self.spring_stiffness, self.damping, self.name)
        out.edges = self.edges.copy()
        out.rest_lengths = self.rest_lengths.copy()
        return out

    def elastic_energy_gradient_diag(self, x: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
        grad = np.zeros_like(x, dtype=float)
        diag = np.zeros_like(x, dtype=float)
        energy = 0.0
        k = float(self.spring_stiffness)
        if k <= 0.0 or len(self.edges) == 0:
            return 0.0, grad, diag
        for eidx, (i, j) in enumerate(self.edges):
            d = x[j] - x[i]
            L = float(np.linalg.norm(d))
            L0 = float(self.rest_lengths[eidx])
            if L <= 1e-14:
                continue
            stretch = L - L0
            energy += 0.5 * k * stretch * stretch
            f = k * stretch * d / L
            grad[i] -= f
            grad[j] += f
            # Safe diagonal upper bound for globalized diagonal Newton.
            diag[i] += k
            diag[j] += k
        return float(energy), grad, diag


@dataclass
class ContactPairSpec:
    body_a: int
    body_b: int
    d_hat: float
    stiffness: float = 1.0
    friction_mu: float = 0.0
    model: str = "ipc_barrier"
    detector: ContactDetector | None = None


@dataclass
class ImplicitScene:
    bodies: list[MassSpringBody]
    contact_pairs: list[ContactPairSpec] = field(default_factory=list)
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=float))


@dataclass
class ImplicitStepResult:
    converged: bool
    iterations: int
    initial_energy: float
    final_energy: float
    max_gradient_norm: float
    accepted_alpha: float
    ccd_gate: TimeDependentInclusionCCDResult | None
    ccd_clamped: bool
    reason: str


class ImplicitDynamicsSolver:
    """Global implicit Euler solver with contact and a conservative CCD gate.

    The unknown vector contains all non-fixed vertices of all bodies.  Each
    nonlinear iteration re-detects contacts at the current global state and uses
    a diagonalized, globally line-searched Newton step.  This is deliberately a
    compact research prototype: it verifies the full coupling architecture
    (inertia + elasticity + contact + friction + CCD gate) while leaving room for
    later replacement by sparse exact Hessians or FEM elements.
    """

    def __init__(
        self,
        max_iter: int = 30,
        grad_tol: float = 1e-6,
        line_search_steps: int = 14,
        ccd_safety: float = 1e-4,
    ):
        self.max_iter = int(max_iter)
        self.grad_tol = float(grad_tol)
        self.line_search_steps = int(line_search_steps)
        self.ccd_safety = float(ccd_safety)

    def _offsets(self, scene: ImplicitScene) -> list[int]:
        offsets = [0]
        for b in scene.bodies:
            offsets.append(offsets[-1] + len(b.mesh.vertices))
        return offsets

    def _pack(self, scene: ImplicitScene) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        x = np.vstack([b.mesh.vertices for b in scene.bodies]).astype(float)
        v = np.vstack([b.velocity for b in scene.bodies]).astype(float)
        m = np.concatenate([b.mass for b in scene.bodies]).astype(float)
        fixed = np.concatenate([b.fixed for b in scene.bodies]).astype(bool)
        return x, v, m, fixed

    def _unpack_to_meshes(self, scene: ImplicitScene, x: np.ndarray) -> list[TriangleMesh]:
        offsets = self._offsets(scene)
        meshes = []
        for bi, b in enumerate(scene.bodies):
            xb = x[offsets[bi]: offsets[bi + 1]]
            meshes.append(TriangleMesh(xb, b.mesh.faces.copy(), None, b.mesh.name))
        return meshes

    def _objective_grad_diag(self, scene: ImplicitScene, x: np.ndarray, y: np.ndarray, mass: np.ndarray, fixed: np.ndarray, dt: float) -> tuple[float, np.ndarray, np.ndarray, float]:
        offsets = self._offsets(scene)
        inv_dt2 = 1.0 / max(dt * dt, 1e-30)
        diff = x - y
        energy = 0.5 * float(np.sum(mass[:, None] * diff * diff)) * inv_dt2
        grad = mass[:, None] * diff * inv_dt2
        diag = np.repeat((mass * inv_dt2)[:, None], 3, axis=1)

        # Elastic energies.
        for bi, body in enumerate(scene.bodies):
            sl = slice(offsets[bi], offsets[bi + 1])
            e, g, d = body.elastic_energy_gradient_diag(x[sl])
            energy += e
            grad[sl] += g
            diag[sl] += d

        contact_energy = 0.0
        meshes = self._unpack_to_meshes(scene, x)
        for spec in scene.contact_pairs:
            detector = spec.detector if spec.detector is not None else ContactDetector(d_hat=spec.d_hat, enable_interval_fallback=True)
            det = detector.detect(meshes[spec.body_a], meshes[spec.body_b])
            if not det.contacts:
                continue
            model = ContactEnergyModel(d_hat=spec.d_hat, stiffness=spec.stiffness, model=spec.model, friction_mu=spec.friction_mu)
            # Friction in the global implicit objective is treated as a force-level
            # regularization using current relative velocities.  Normal contact is
            # still energy based through model.density_and_derivative.
            resp = evaluate_contact_response(det.contacts, model)
            fa, fb = assemble_nodal_forces(meshes[spec.body_a], meshes[spec.body_b], det.contacts, resp)
            sa = slice(offsets[spec.body_a], offsets[spec.body_a + 1])
            sb = slice(offsets[spec.body_b], offsets[spec.body_b + 1])
            grad[sa] -= fa
            grad[sb] -= fb
            # Add a conservative diagonal stiffness near active contacts for stable
            # line-searched Newton without forming the full IPC Hessian.
            kdiag = max(spec.stiffness, 1.0) / max(spec.d_hat * spec.d_hat, 1e-12)
            touched_a = np.unique(meshes[spec.body_a].faces[[c.face_a for c in det.contacts]].reshape(-1))
            touched_b = np.unique(meshes[spec.body_b].faces[[c.face_b for c in det.contacts]].reshape(-1))
            da = diag[sa]
            db = diag[sb]
            da[touched_a] += kdiag
            db[touched_b] += kdiag
            diag[sa] = da
            diag[sb] = db
            energy += resp.total_energy
            contact_energy += resp.total_energy

        grad[fixed] = 0.0
        diag[fixed] = 1.0
        return float(energy), grad, np.maximum(diag, 1e-12), float(contact_energy)

    def step(self, scene: ImplicitScene, dt: float, use_ccd_gate: bool = True) -> ImplicitStepResult:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        x0, v0, mass, fixed = self._pack(scene)
        gravity = np.asarray(scene.gravity, dtype=float)
        y = x0 + dt * v0 + (dt * dt) * gravity[None, :]
        y[fixed] = x0[fixed]
        x = x0.copy()

        e0, grad, diag, _ = self._objective_grad_diag(scene, x, y, mass, fixed, dt)
        initial_energy = e0
        last_alpha = 0.0
        reason = "max_iter"
        converged = False
        gnorm = float(np.max(np.linalg.norm(grad[~fixed], axis=1))) if np.any(~fixed) else 0.0

        for it in range(1, self.max_iter + 1):
            energy, grad, diag, _ = self._objective_grad_diag(scene, x, y, mass, fixed, dt)
            free = ~fixed
            gnorm = float(np.max(np.linalg.norm(grad[free], axis=1))) if np.any(free) else 0.0
            if gnorm <= self.grad_tol:
                converged = True
                reason = "gradient_tolerance"
                break
            direction = -grad / diag
            direction[fixed] = 0.0
            alpha = 1.0
            accepted = False
            for _ in range(self.line_search_steps):
                trial = x + alpha * direction
                trial[fixed] = x0[fixed]
                trial_energy, _, _, _ = self._objective_grad_diag(scene, trial, y, mass, fixed, dt)
                if trial_energy <= energy + 1e-12:
                    x = trial
                    last_alpha = alpha
                    accepted = True
                    break
                alpha *= 0.5
            if not accepted:
                reason = "line_search_stalled"
                break
        else:
            it = self.max_iter

        # Conservative global CCD gate.  If the proposed end state cannot be
        # certified safe, clamp to the last certified fraction and update x.
        ccd_result: TimeDependentInclusionCCDResult | None = None
        ccd_clamped = False
        if use_ccd_gate and scene.contact_pairs:
            meshes0 = self._unpack_to_meshes(scene, x0)
            meshes1 = self._unpack_to_meshes(scene, x)
            safe_fraction = 1.0
            worst_result = None
            for spec in scene.contact_pairs:
                tdi = TimeDependentInclusionCCD(
                    d_hat=spec.d_hat,
                    detector=spec.detector if spec.detector is not None else ContactDetector(d_hat=spec.d_hat, enable_interval_fallback=True),
                    time_tol=max(1e-6, dt * 1e-4),
                )
                r = tdi.detect(meshes0[spec.body_a], meshes1[spec.body_a], meshes0[spec.body_b], meshes1[spec.body_b])
                worst_result = r
                if not r.certified_no_impact:
                    safe_fraction = min(safe_fraction, max(0.0, r.safe_fraction - self.ccd_safety))
            ccd_result = worst_result
            if safe_fraction < 1.0:
                ccd_clamped = True
                x = x0 + safe_fraction * (x - x0)
                reason = f"ccd_clamped:{ccd_result.reason if ccd_result else 'unknown'}"

        # Commit updated positions and velocities.
        offsets = self._offsets(scene)
        for bi, body in enumerate(scene.bodies):
            sl = slice(offsets[bi], offsets[bi + 1])
            old = body.mesh.vertices.copy()
            new = x[sl]
            new[body.fixed] = old[body.fixed]
            body.mesh = TriangleMesh(new, body.mesh.faces.copy(), None, body.mesh.name)
            body.velocity = (new - old) / dt
            body.velocity[body.fixed] = 0.0

        final_energy, final_grad, _, _ = self._objective_grad_diag(scene, x, y, mass, fixed, dt)
        final_g = float(np.max(np.linalg.norm(final_grad[~fixed], axis=1))) if np.any(~fixed) else 0.0
        return ImplicitStepResult(
            converged=bool(converged),
            iterations=int(it),
            initial_energy=float(initial_energy),
            final_energy=float(final_energy),
            max_gradient_norm=float(final_g),
            accepted_alpha=float(last_alpha),
            ccd_gate=ccd_result,
            ccd_clamped=ccd_clamped,
            reason=reason,
        )
