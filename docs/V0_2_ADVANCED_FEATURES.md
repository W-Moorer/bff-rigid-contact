# CALG v0.2 advanced feature notes

This document records the implementation added after the initial v0.1 prototype.

## 1. Curved 4D patch-pair Newton solver

Implemented in:

- `src/calg/core/curved_patch.py`
- `src/calg/solver/curved_newton.py`

Each input triangle is lifted into a local quadratic normal-lifted patch:

```text
X(x, y) = o + x t1 + y t2 + h(x, y) n0
```

where `h` is fitted from the three vertex positions and vertex-normal gradients.  The 4D solver minimizes

```text
0.5 || X_A(u, v) - X_B(s, t) ||^2
```

with a damped Gauss--Newton system and projected triangle-domain constraints.

## 2. Interval / Bezier conservative fallback

Implemented in:

- `src/calg/solver/interval_fallback.py`

Because the lifted patch is quadratic, the code constructs an exact quadratic triangular Bezier control hull over each subtriangle.  The AABB of this control hull gives a conservative subdivision bound for local branch-and-bound refinement.

## 3. Continuous collision detection

Implemented in:

- `src/calg/solver/ccd.py`

The current CCD API supports linearly moving meshes with identical topology at the beginning and end of a step.  It brackets the first threshold crossing of `min_gap <= d_hat` using the high-order static detector, then refines time of impact by bisection.

This is a project-level CCD front end.  It is intentionally structured so that later pairwise time-dependent inclusion kernels can replace the bracketing backend without changing the public API.

## 4. Contact response, friction, IPC/GCP-style energies

Implemented in:

- `src/calg/solver/response.py`

Supported response models:

- `ipc_barrier`: `-(d-d_hat)^2 log(d/d_hat)` for `0 < d < d_hat`.
- `quadratic_proximity`: `0.5 (d_hat-d)^2` for `d < d_hat`.
- `gcp_patch`: IPC-style barrier density multiplied by the contact patch overlap area.

The response module also implements regularized Coulomb friction and barycentric nodal force assembly.

## 5. Validation

Run:

```bash
PYTHONPATH=src pytest -q
PYTHONPATH=src python scripts/run_validation.py --out-dir results/validation_v0_2
PYTHONPATH=src python scripts/run_advanced_validation.py
```

Expected sandbox status:

```text
11 passed
```

Generated outputs:

- `results/validation_v0_2/validation_summary.csv`
- `results/advanced_validation/advanced_validation_summary.csv`
