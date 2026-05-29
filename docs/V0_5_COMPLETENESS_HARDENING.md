# CALG v0.5 completeness hardening

This update addresses the items that were still incomplete in v0.4: complete curved 4D Newton solving, conservative subdivision/interval fallback, CCD acceptance, and frictional contact response.

## 1. Complete curved 4D Newton solver

File: `src/calg/solver/curved_newton.py`

The solver now enumerates all reduced active sets for two quadratic normal-lifted triangular patches:

- face-face: full 4D Newton with curvature second-derivative terms;
- edge-face and face-edge: reduced 3D active-set Newton;
- edge-edge: reduced 2D active-set Newton;
- vertex-face, face-vertex, vertex-edge, edge-vertex and vertex-vertex degenerate cases.

The returned feature label is no longer a byproduct of clamping only; tie breaking favors converged lower-dimensional active sets when a face-domain solve lands on a feature boundary.

## 2. Conservative subdivision / interval fallback

File: `src/calg/solver/interval_fallback.py`

The fallback remains a conservative branch-and-bound routine over quadratic Bézier control-hull AABBs. It returns three distinct states:

- `contact=True`: a sampled candidate within the threshold was found;
- `certified_no_contact=True`: all unprocessed control-hull intervals are separated beyond the threshold plus error margins;
- `reason='uncertain_best_sample'`: the algorithm intentionally refuses to certify no contact when the remaining interval bounds are not decisive.

This means it is conservative for rejecting contact. It may produce false positives or uncertain intervals, but it does not convert unresolved intervals into no-contact certificates.

## 3. CCD / no-penetration acceptance gate

Files: `src/calg/solver/tdi_ccd.py`, `src/calg/solver/ccd.py`

The project now distinguishes two CCD layers:

- `ContinuousContactDetector`: practical threshold TOI bracketing by repeated static detection;
- `TimeDependentInclusionCCD`: conservative time-dependent inclusion acceptance gate.

The TDI gate accepts a full step only when all swept face-pair intervals are separated by conservative AABB inclusion bounds. Unresolved terminal intervals are blocking events, not accepted safe steps. This is appropriate for a global no-penetration *acceptance gate*.

## 4. Frictional contact response

File: `src/calg/solver/response.py`

The response layer now supports both:

- velocity-regularized Coulomb friction, compatible with previous v0.4 behavior;
- stateful penalty-regularized stick/slip friction through `FrictionStateStore` and `TangentialFrictionState`.

The stateful law integrates tangential relative displacement, projects the resulting tangential force to the Coulomb limit, and stores the projected tangential displacement for the next step. Returned wrenches include `friction_mode` (`stick`, `slip`, `friction_inactive`, or `velocity_regularized`).

## 5. Validation status

The v0.5 regression suite adds three targeted tests:

- active-set curved Newton finds boundary feature contact;
- stateful friction is projected to the Coulomb limit;
- conservative TDI CCD blocks a crossing motion before accepting the full step.

Command used in the sandbox:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q
```

Result:

```text
19 passed in 23.95s
```

## Remaining distinction

This v0.5 update closes the research-prototype gaps at the algorithm/API level. It is still not a production-grade industrial dynamics engine: the implicit global solver remains a compact mass-spring/mesh prototype with diagonalized contact stiffness, not a full sparse FEM/IPC solver with exact barrier Hessians and GPU kernels for every stage.
