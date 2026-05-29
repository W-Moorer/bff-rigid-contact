# CALG Contact Prototype

This repository implements the first validation-ready prototype of **Contact-Aligned Local Graphs (CALG)** for high-order surface-to-surface contact detection.  The solver is designed as a geometry front-end for later contact response methods such as penalty, barrier, IPC/GCP-style potentials, and multibody/FEM pipelines.

The implementation follows these constraints:

- No global UV parameterization is required.
- No manual cutting or disk-topology assumption is required for the core solver.
- BFF is optional and isolated in an adapter; the core contact solver never depends on BFF.
- Modeling/generation, core geometry, contact computation, experiments, and frontend scripts are separated.
- Numerical examples write CSV/JSON/figures; paper tables should be generated from these outputs only.
- The industrial example is intentionally reserved until the analytic validation suite passes.

## Quick start

```bash
python -m pip install -e .[plots,test]
pytest -q
python scripts/run_validation.py --out-dir results/validation
```

or after installation:

```bash
calg-run-validation --out-dir results/validation
```

## Repository layout

```text
src/calg/modeling/    mesh containers, analytic mesh generators, OBJ I/O
src/calg/core/        vector math, AABB/BVH, triangle distance, curved jet primitives
src/calg/solver/      contact frame, graphability certificate, graph gap, detector, BFF adapter
src/calg/experiments/ validation cases, metrics, runner
src/calg/frontend/    CLI entry point
tests/                unit and regression tests
scripts/              reproducible validation entry points
docs/                 implementation notes and Codex instructions
```

## Algorithmic skeleton

For each candidate pair produced by the inflated 3D BVH, CALG builds a local frame aligned with the current candidate contact direction.  If the two local normal cones certify that both patches are graphs over the same contact plane, the solver reduces regular surface-to-surface contact to a two-dimensional gap-field minimization.  Otherwise it falls back to a generic curved/linear patch-pair closest-point query, and unresolved cases can be handled by local subdivision in later phases.

The current implementation includes a quadratic graph-jet fast path, triangle-pair fallback, analytic validation generators, and an optional BFF command-line adapter.  Continuous collision detection, frictional response, GPU kernels, and the complex industrial example are intentionally left for later phases.
