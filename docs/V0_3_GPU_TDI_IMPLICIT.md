# CALG v0.3: GPU backend, conservative TDI-CCD gate, and implicit global dynamics

This document records what has been implemented in v0.3 and what remains outside the current prototype.

## Implemented modules

### 1. Optional GPU broad phase

Files:

- `src/calg/gpu/backend.py`
- `src/calg/gpu/broadphase.py`

The GPU path is implemented as an optional CuPy backend for vectorized AABB overlap queries. If CuPy or a CUDA device is unavailable, the same API falls back to NumPy and reports the backend metadata explicitly. This keeps the project runnable on CPU-only validation machines while preserving the CUDA execution path for a workstation.

The current kernel is a dense/tiled AABB overlap query. It is not a full GPU BVH. It is intended for validation and for later replacement by a CUDA BVH or spatial hash without changing the public API.

### 2. Conservative time-dependent inclusion CCD gate

File:

- `src/calg/solver/tdi_ccd.py`

The new `TimeDependentInclusionCCD` class implements a conservative global step-acceptance gate for linearly moving meshes.

Core behavior:

1. Build swept AABB candidate pairs over the entire time interval.
2. For each candidate pair, recursively bisect time intervals.
3. Certify no contact only when the swept inclusion boxes are farther than the contact threshold.
4. If an interval cannot be certified safe at the requested tolerance, return it as an impact/uncertain blocking interval, not as no-contact.

This gives a practical strict no-penetration acceptance rule: the simulator accepts a full step only when all swept intervals are certified safe. Ambiguous intervals may cause false positives and smaller steps, but are not used to accept unsafe steps.

### 3. Full implicit global dynamics prototype

Files:

- `src/calg/dynamics/__init__.py`
- `src/calg/dynamics/implicit.py`

The `ImplicitDynamicsSolver` implements a coupled global implicit Euler step over all surface-mesh body vertices. The global objective contains:

- inertial backward-Euler term,
- edge-spring elastic energy,
- barrier/GCP-style contact response from the contact detector,
- optional regularized Coulomb friction at the force level,
- a conservative TDI-CCD gate before committing the step.

The solver is a compact research prototype. It uses a diagonalized, line-searched Newton update rather than a production sparse exact Hessian. The architecture is complete enough to validate global coupling and step acceptance, while leaving a clear path to replace the spring model with FEM and the diagonal Hessian with a sparse Hessian.

## Validation commands

```bash
PYTHONPATH=src pytest -q
PYTHONPATH=src python scripts/run_validation.py --out-dir results/validation_v0_3_base
PYTHONPATH=src python scripts/run_advanced_validation.py
PYTHONPATH=src python scripts/run_v0_3_validation.py --out-dir results/v0_3_validation
```

Current test result:

```text
15 passed
```

Current v0.3 validation result:

```text
gpu_vectorized_broadphase: passed; backend is NumPy on the current CPU-only host
tdi_ccd_moving_plane: passed; TOI interval contains analytic threshold time
implicit_global_step_with_tdi_gate: passed; final explicit gap remains above threshold after CCD clamping
```

## Important limitations

The following items are not yet production-grade:

1. The GPU path is API-complete and CuPy-ready, but the present host has no CUDA device; true GPU speedup numbers require a CUDA workstation.
2. The TDI-CCD gate is conservative and step-safe, but it is not yet the full TDIB/SOS-style high-order parametric CCD kernel for all curved patches.
3. The implicit solver is a global coupled solver, but the mechanical model is an edge-spring prototype rather than FEM or modal flexible-body dynamics.
4. The contact Hessian is diagonalized for robustness and simplicity; exact sparse IPC/GCP Hessians are left for the next stage.
5. The complex industrial example is still intentionally left empty until the validation suite is completed on target hardware.
