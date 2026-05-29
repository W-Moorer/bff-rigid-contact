from __future__ import annotations

from dataclasses import dataclass
from typing import Callable
import numpy as np

from calg.core.curved_patch import QuadraticPatch3D
from calg.core.math_utils import EPS, normalize
from calg.core.primitives import CurvedJetPrimitive
from calg.core.triangle_distance import triangle_triangle_closest


@dataclass
class CurvedClosestResult:
    valid: bool
    contact: bool
    point_a: np.ndarray
    point_b: np.ndarray
    normal: np.ndarray
    gap: float
    xi_a: np.ndarray
    xi_b: np.ndarray
    bary_a: np.ndarray
    bary_b: np.ndarray
    iterations: int
    residual: float
    feature: str
    reason: str


@dataclass(frozen=True)
class _FeatureDomain:
    kind: str
    name: str
    data: tuple[int, ...]
    dim: int

    @staticmethod
    def face() -> "_FeatureDomain":
        return _FeatureDomain("face", "face", (), 2)

    @staticmethod
    def edge(i: int, j: int, name: str) -> "_FeatureDomain":
        return _FeatureDomain("edge", name, (i, j), 1)

    @staticmethod
    def vertex(i: int) -> "_FeatureDomain":
        return _FeatureDomain("vertex", f"vertex{i}", (i,), 0)


def _all_features() -> list[_FeatureDomain]:
    return [
        _FeatureDomain.face(),
        _FeatureDomain.edge(0, 1, "edge01"),
        _FeatureDomain.edge(1, 2, "edge12"),
        _FeatureDomain.edge(2, 0, "edge20"),
        _FeatureDomain.vertex(0),
        _FeatureDomain.vertex(1),
        _FeatureDomain.vertex(2),
    ]


def _as_var(feature: _FeatureDomain, patch: QuadraticPatch3D, xi_hint: np.ndarray | None) -> np.ndarray:
    if feature.kind == "face":
        return patch.project_to_domain(patch.eval(xi_hint)) if xi_hint is not None else patch.centroid_xi()
    if feature.kind == "edge":
        i, j = feature.data
        a, b = patch.domain[i], patch.domain[j]
        if xi_hint is None:
            return np.array([0.5], dtype=float)
        e = b - a
        denom = float(e @ e)
        t = 0.5 if denom <= EPS else float(((xi_hint - a) @ e) / denom)
        return np.array([min(1.0, max(0.0, t))], dtype=float)
    return np.zeros(0, dtype=float)


def _xi_from_var(feature: _FeatureDomain, patch: QuadraticPatch3D, z: np.ndarray) -> np.ndarray:
    if feature.kind == "face":
        return patch.clamp(z)
    if feature.kind == "edge":
        i, j = feature.data
        t = min(1.0, max(0.0, float(z[0])))
        return (1.0 - t) * patch.domain[i] + t * patch.domain[j]
    i = feature.data[0]
    return patch.domain[i].copy()


def _clamp_var(feature: _FeatureDomain, patch: QuadraticPatch3D, z: np.ndarray) -> np.ndarray:
    if feature.kind == "face":
        return patch.clamp(z)
    if feature.kind == "edge":
        return np.array([min(1.0, max(0.0, float(z[0])))], dtype=float)
    return np.zeros(0, dtype=float)


def _feature_jac_second(feature: _FeatureDomain, patch: QuadraticPatch3D, z: np.ndarray) -> tuple[np.ndarray, list[np.ndarray], np.ndarray]:
    """Return xi, restricted first derivatives, and restricted second derivatives.

    The second derivative list is stored in row-major order for the local
    feature variables: S[i * dim + j] = d^2 X / dz_i dz_j.
    """
    xi = _xi_from_var(feature, patch, z)
    if feature.kind == "face":
        J = patch.derivatives(xi)
        x_xx, x_xy, x_yy = patch.second_derivatives()
        S = [x_xx, x_xy, x_xy, x_yy]
        return xi, J, S
    if feature.kind == "edge":
        i, j = feature.data
        e = patch.domain[j] - patch.domain[i]
        J2 = patch.derivatives(xi)
        J = (J2 @ e.reshape(2, 1)).reshape(3, 1)
        S = [patch.second_directional(e, e)]
        return xi, J, S
    return xi, np.zeros((3, 0), dtype=float), []


def _energy(pa: QuadraticPatch3D, pb: QuadraticPatch3D, xa: np.ndarray, xb: np.ndarray) -> float:
    r = pa.eval(xa) - pb.eval(xb)
    return 0.5 * float(r @ r)


def _objective_for_features(
    patch_a: QuadraticPatch3D,
    patch_b: QuadraticPatch3D,
    feat_a: _FeatureDomain,
    feat_b: _FeatureDomain,
    za: np.ndarray,
    zb: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    xi_a, Ja, Sa = _feature_jac_second(feat_a, patch_a, za)
    xi_b, Jb, Sb = _feature_jac_second(feat_b, patch_b, zb)
    Xa = patch_a.eval(xi_a)
    Xb = patch_b.eval(xi_b)
    r = Xa - Xb
    E = 0.5 * float(r @ r)
    da = feat_a.dim
    db = feat_b.dim
    ga = Ja.T @ r if da else np.zeros(0, dtype=float)
    gb = -Jb.T @ r if db else np.zeros(0, dtype=float)
    grad = np.concatenate([ga, gb])
    H = np.zeros((da + db, da + db), dtype=float)
    if da:
        Haa = Ja.T @ Ja
        for i in range(da):
            for j in range(da):
                Haa[i, j] += float(r @ Sa[i * da + j])
        H[:da, :da] = Haa
    if db:
        Hbb = Jb.T @ Jb
        for i in range(db):
            for j in range(db):
                Hbb[i, j] -= float(r @ Sb[i * db + j])
        H[da:, da:] = Hbb
    if da and db:
        Hab = -Ja.T @ Jb
        H[:da, da:] = Hab
        H[da:, :da] = Hab.T
    return E, grad, H, xi_a, xi_b, Xa, Xb, float(np.linalg.norm(grad))


def _solve_feature_pair(
    patch_a: QuadraticPatch3D,
    patch_b: QuadraticPatch3D,
    feat_a: _FeatureDomain,
    feat_b: _FeatureDomain,
    xi_a_hint: np.ndarray,
    xi_b_hint: np.ndarray,
    max_iter: int,
    grad_tol: float,
    step_tol: float,
    damping: float,
) -> tuple[bool, str, int, float, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    za = _as_var(feat_a, patch_a, xi_a_hint)
    zb = _as_var(feat_b, patch_b, xi_b_hint)
    za = _clamp_var(feat_a, patch_a, za)
    zb = _clamp_var(feat_b, patch_b, zb)
    E, grad, H, xi_a, xi_b, Xa, Xb, residual = _objective_for_features(patch_a, patch_b, feat_a, feat_b, za, zb)
    if feat_a.dim + feat_b.dim == 0:
        return True, "zero_dimensional_feature", 0, residual, xi_a, xi_b, Xa, Xb, E

    converged = False
    reason = "max_iter"
    it = 0
    for it in range(1, max_iter + 1):
        E, grad, H, xi_a, xi_b, Xa, Xb, residual = _objective_for_features(patch_a, patch_b, feat_a, feat_b, za, zb)
        if residual <= grad_tol:
            converged = True
            reason = "converged_gradient"
            break
        Hreg = H + damping * np.eye(H.shape[0])
        # Use full Newton when positive enough, otherwise damped least-squares descent.
        try:
            step = -np.linalg.solve(Hreg, grad)
        except np.linalg.LinAlgError:
            step = -np.linalg.lstsq(Hreg, grad, rcond=None)[0]
        if not np.all(np.isfinite(step)) or float(grad @ step) > 0.0:
            step = -grad / max(float(np.linalg.norm(grad)), EPS)
        if float(np.linalg.norm(step)) <= step_tol:
            converged = True
            reason = "converged_step"
            break
        alpha = 1.0
        accepted = False
        da = feat_a.dim
        for _ in range(16):
            na = _clamp_var(feat_a, patch_a, za + alpha * step[:da]) if da else za
            nb = _clamp_var(feat_b, patch_b, zb + alpha * step[da:]) if feat_b.dim else zb
            nE, *_ = _objective_for_features(patch_a, patch_b, feat_a, feat_b, na, nb)
            if nE <= E + 1e-14:
                za, zb = na, nb
                accepted = True
                break
            alpha *= 0.5
        if not accepted:
            reason = "line_search_stalled"
            break
    E, grad, H, xi_a, xi_b, Xa, Xb, residual = _objective_for_features(patch_a, patch_b, feat_a, feat_b, za, zb)
    return converged, reason, it, residual, xi_a, xi_b, Xa, Xb, E


def solve_curved_patch_pair(
    pa: CurvedJetPrimitive,
    pb: CurvedJetPrimitive,
    d_hat: float,
    max_iter: int = 40,
    grad_tol: float = 1e-10,
    step_tol: float = 1e-11,
    damping: float = 1e-10,
    enumerate_active_sets: bool = True,
) -> CurvedClosestResult:
    """Complete active-set Newton closest-point solve between two curved patches.

    The solver enumerates face, edge, and vertex feature combinations on both
    quadratic normal-lifted patches.  Interior face-face cases are solved as a
    full 4D Newton problem with second-derivative curvature terms; boundary
    active sets are solved in their reduced variables.  The best feasible
    feature-pair candidate is returned, which prevents the common failure mode
    of a single projected 4D iteration getting stuck on the wrong boundary.
    """
    patch_a = QuadraticPatch3D.from_primitive(pa)
    patch_b = QuadraticPatch3D.from_primitive(pb)

    linear = triangle_triangle_closest(pa.vertices, pb.vertices)
    xi_a_hint = patch_a.project_to_domain(linear.point_a)
    xi_b_hint = patch_b.project_to_domain(linear.point_b)

    features = _all_features() if enumerate_active_sets else [_FeatureDomain.face()]
    best: tuple[float, bool, str, int, float, _FeatureDomain, _FeatureDomain, np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
    total_iter = 0
    best_residual = float("inf")
    any_valid = False

    # Prefer lower-dimensional features only when their energy wins; otherwise
    # regular smooth contact remains face-face.
    for fa in features:
        for fb in features:
            try:
                conv, reason, it, residual, xia, xib, Xa, Xb, E = _solve_feature_pair(
                    patch_a, patch_b, fa, fb, xi_a_hint, xi_b_hint, max_iter, grad_tol, step_tol, damping
                )
            except Exception:
                continue
            if not (np.all(np.isfinite(Xa)) and np.all(np.isfinite(Xb)) and np.isfinite(E)):
                continue
            any_valid = True
            total_iter += int(it)
            best_residual = min(best_residual, float(residual))
            # Tie breaking matters at feature boundaries: a face-domain solve
            # may clamp to an edge and obtain the same energy with a nonzero
            # constrained residual.  Prefer converged, lower-dimensional active
            # sets when the energies are indistinguishable.
            active_dim = fa.dim + fb.dim
            score = float(E) + (0.0 if conv else 1e-9) + 1e-12 * active_dim + 1e-13 * min(float(residual), 1.0)
            if best is None or score < best[0]:
                best = (score, bool(conv), reason, int(it), float(residual), fa, fb, xia, xib, Xa, Xb)

    if best is None:
        Xa = pa.centroid
        Xb = pb.centroid
        sep = Xb - Xa
        gap = float(np.linalg.norm(sep))
        n = normalize(sep, normalize(pb.centroid - pa.centroid, pa.face_normal))
        return CurvedClosestResult(False, False, Xa, Xb, n, gap, np.zeros(2), np.zeros(2), np.array([1.0, 0, 0]), np.array([1.0, 0, 0]), 0, float("inf"), "invalid", "no_valid_active_set")

    E, conv, reason, it, residual, fa, fb, xi_a, xi_b, Xa, Xb = best
    sep = Xb - Xa
    gap = float(np.linalg.norm(sep))
    if gap <= EPS:
        na = patch_a.normal(xi_a)
        nb = patch_b.normal(xi_b)
        n = normalize(na - nb, normalize(pb.centroid - pa.centroid, pa.face_normal))
    else:
        n = sep / gap
    valid = bool(any_valid and np.all(np.isfinite(Xa)) and np.all(np.isfinite(Xb)) and np.isfinite(gap))
    contact = bool(valid and gap <= d_hat + pa.error_bound + pb.error_bound)
    feature = f"A_{fa.name}-B_{fb.name}"
    if conv:
        final_reason = f"active_set_full_newton:{reason}"
    else:
        final_reason = f"active_set_best_candidate:{reason}"
    return CurvedClosestResult(
        valid=valid,
        contact=contact,
        point_a=Xa,
        point_b=Xb,
        normal=n,
        gap=gap,
        xi_a=xi_a,
        xi_b=xi_b,
        bary_a=patch_a.barycentric(xi_a),
        bary_b=patch_b.barycentric(xi_b),
        iterations=int(total_iter),
        residual=float(residual if np.isfinite(residual) else best_residual),
        feature=feature,
        reason=final_reason,
    )
