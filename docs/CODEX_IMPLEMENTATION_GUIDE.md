# Codex implementation guide for CALG contact validation

This guide is intentionally stricter than the current prototype.  It describes the staged implementation path required to complete the numerical examples in the CMAME-oriented paper draft.

## Non-negotiable architecture constraints

1. The core contact solver must not depend on BFF, global UVs, disk topology, or manual cuts.
2. BFF is allowed only through `calg.solver.bff_adapter.BFFAdapter` as an optional local conditioning or reference tool.
3. Modeling and computation must remain separated:
   - `calg.modeling`: geometry generation and I/O only.
   - `calg.core`: primitives, AABB, BVH, local geometry math.
   - `calg.solver`: contact algorithms only.
   - `calg.experiments`: validation protocols and outputs.
   - `calg.frontend`: CLI/API user-facing entry points.
4. Numerical-example values in the paper must come from generated CSV/JSON files under `results/`; do not hand-enter data.
5. The complex industrial example is reserved.  Do not implement it until all analytic and semi-analytic validation cases pass.

## Current prototype status

Implemented:

- Triangle mesh container and analytic mesh generators.
- Curved-jet primitive metadata: per-face vertices, vertex normals, curvature/error estimates, normal cone, inflated AABB.
- 3D AABB BVH pair traversal.
- Contact-aligned frame construction from a linear triangle-triangle closest pair.
- Normal-cone graphability certificate.
- Quadratic height-jet fitting in the local contact frame.
- Two-dimensional graph-gap minimization over projected triangle overlap.
- Generic triangle-triangle closest-point fallback.
- Validation runner writing CSV/JSON/Markdown and optional plots.
- Optional BFF CLI adapter.

Known limitations:

- The fallback is currently linear triangle-pair closest point, not a full curved 4D Newton solver.
- The graph fast path uses local quadratic height jets fitted on demand, not persistent PN/Bézier patches.
- There is no continuous collision detection yet.
- There is no friction/contact response yet; outputs are geometric contact samples.
- Conservative interval/Bézier subdivision fallback is not yet implemented.

## Phase 1: make the curved primitive explicit

Replace on-demand quadratic-only graph jets with persistent primitive choices:

- `LinearTrianglePrimitive`
- `QuadraticHeightPrimitive`
- `PNTrianglePrimitive`
- `BezierTrianglePrimitive`

Each primitive must expose:

```python
eval(u, v) -> x
derivatives(u, v) -> Xu, Xv
normal(u, v) -> n
normal_cone() -> cone
curvature_bound() -> float
error_bound() -> float
bounds(margin) -> AABB
```

Acceptance tests:

- Plane primitive evaluates exactly.
- Sphere patch normal error decreases under refinement.
- Curvature/error bounds are nonnegative and monotone under uniform refinement on analytic cases.

## Phase 2: curved 4D patch-pair fallback

Implement a real curved closest-point solve:

```text
min_{p in T_A, q in T_B} 0.5 ||X_A(p)-X_B(q)||^2
```

Use linear closest barycentric coordinates as the initial guess.  Support active-set downgrade:

- surface-surface
- curve-surface
- curve-curve
- point-surface
- point-curve
- point-point

Acceptance tests:

- Matches analytic plane-plane gap.
- Matches sphere-plane gap under refinement.
- Does not fail on open patch boundaries.
- Degenerate or ill-conditioned cases return a structured `uncertain` result rather than raising.

## Phase 3: conservative fallback

Implement local subdivision / interval bounds for unresolved pairs.

Required behavior:

- If Newton residual is large, subdivide the primitive pair locally.
- If a conservative lower-distance bound exceeds `d_hat`, reject.
- If local subdivision reaches tolerance, return the best certified sample.

Acceptance tests:

- Difficult tangency case does not miss contact.
- High-curvature edge case returns fallback rather than invalid graph result.
- Runtime remains output-sensitive: fallback count must be reported.

## Phase 4: contact patch integration

Move from contact samples to surface-to-surface contact patch quantities:

```math
E_AB = \int_D Phi(g(xi)) w(xi) dxi
w(xi)=0.5(1/|N_A dot n_c|+1/|N_B dot n_c|)
```

Implementation tasks:

- Projected-domain clipping for graph-valid pairs.
- Adaptive quadrature near `g < d_hat`.
- Patch clustering and duplicate merging.
- CSV exports for patch area, mean pressure proxy, normal force proxy.

Acceptance tests:

- Plane-plane patch area matches analytic overlap area.
- Sphere-plane active area scales with contact threshold.
- Symmetric A->B/B->A contact energy agrees within tolerance.

## Phase 5: numerical examples for the paper

Generate these datasets:

1. Analytic gap accuracy: plane-plane, sphere-plane, paraboloid-plane.
2. Refinement convergence: compare linear triangle fallback, graph gap, and globally refined linear mesh.
3. Fast-path statistics: graphability pass ratios across smooth, high-curvature, boundary, and sharp-feature cases.
4. Robustness: open surfaces, unoriented/two-sided shells, boundary contact.
5. Optional BFF local conditioning: run only if official BFF CLI is available; report skip otherwise.

Outputs:

```text
results/paper/tables/*.csv
results/paper/figures/*.png
results/paper/logs/*.txt
results/paper/metadata/*.json
```

## Phase 6: industrial scenario placeholder

Only after Phases 1--5 pass, implement one complex industrial case. Candidate cases:

- cam-roller contact,
- gear tooth flank contact,
- bearing clearance contact,
- CAD bracket/shell contact.

Requirements:

- Model import isolated under `calg.modeling`.
- Solver settings stored as JSON/YAML.
- Geometry preprocessing and computation outputs separated.
- The case must include a baseline comparison: linear triangle contact, globally refined triangle contact, and CALG.

Do not fabricate industrial results.  If geometry or reference data is unavailable, leave the industrial example as a documented placeholder.
