# Implemented status

This package directly implements the first validation-ready CALG prototype requested for the contact-computation project.

## Implemented now

- Topology-independent core contact detector; no global UV, no manual cutting, no disk-topology assumption.
- Optional BFF adapter only; core solver does not depend on BFF.
- Mesh/modeling front end separated from solver computation.
- Analytic geometry generators for plane, sphere, paraboloid, wavy open sheet, and cylinder cases.
- Curved-jet primitive metadata with normal cones, curvature estimates, error estimates, and inflated AABBs.
- 3D BVH broad phase.
- Contact-aligned frame construction.
- Normal-cone graphability certificate.
- 2D graph-gap fast path using a local quadratic height jet.
- Generic triangle-pair fallback.
- Validation runner producing CSV/JSON/Markdown outputs.
- Unit tests: 6 passed in the sandbox using `PYTHONPATH=src pytest -q`.
- Generated validation outputs in `results/validation/`.

## Validation outputs currently generated

See `results/validation/validation_summary.csv` and `results/validation/validation_summary.md`.

The current quick validation suite includes:

1. parallel plane-plane gap,
2. sphere-plane analytic gap,
3. paraboloid-plane analytic gap,
4. open wavy sheet-plane topology-independent case,
5. cylinder-plane line-contact-style case.

## Not implemented yet by design

- Full curved 4D Newton patch-pair solver.
- Conservative interval/Bézier subdivision fallback.
- Continuous collision detection.
- Friction/contact response.
- GPU implementation.
- Final industrial example.

These are staged in `docs/CODEX_IMPLEMENTATION_GUIDE.md` so Codex can continue from this working prototype without changing the architecture.


## v0.3 additions

Implemented after v0.2:

- optional GPU/CuPy vectorized AABB broad-phase API with NumPy fallback;
- conservative time-dependent inclusion CCD gate in `src/calg/solver/tdi_ccd.py`;
- full coupled implicit global dynamics prototype in `src/calg/dynamics/implicit.py`;
- v0.3 validation script in `scripts/run_v0_3_validation.py`;
- documentation in `docs/V0_3_GPU_TDI_IMPLICIT.md` and `NUMERICAL_EXAMPLES_READINESS.md`.

Current validation commands:

```bash
PYTHONPATH=src pytest -q
PYTHONPATH=src python scripts/run_validation.py --out-dir results/validation_v0_3_base
PYTHONPATH=src python scripts/run_advanced_validation.py
PYTHONPATH=src python scripts/run_v0_3_validation.py --out-dir results/v0_3_validation
```

Current test status: `15 passed`.

Important qualification: the GPU code path is implemented but this sandbox is CPU-only, so the current validation uses the NumPy fallback. CUDA speedup claims require running the same validation on a CUDA workstation.


## v0.4 additions

Implemented numerical-example completion infrastructure:

- CUDA/CuPy-aware GPU broad-phase benchmark harness with explicit fallback labeling.
- Mesh refinement convergence study.
- Baseline comparisons against linear triangle BVH, global subdivision linear contact, and open-source Trimesh sampled proximity.
- Deterministic complex-surface robustness statistics.
- v0.4 regression test covering the benchmark harnesses.

Current validation:

```text
16 passed
```

Important: the current sandbox has no CUDA device.  GPU benchmark outputs generated here are NumPy fallback timings and are intentionally marked with `is_true_cuda=False`.


## v0.5 completeness hardening

Implemented after the v0.4 numerical-example infrastructure:

- complete active-set curved Newton solver for face/edge/vertex combinations;
- conservative Bézier control-hull subdivision fallback semantics retained and documented;
- conservative TDI-CCD acceptance gate semantics documented as blocking uncertain intervals;
- stateful regularized Coulomb friction with stick/slip projection;
- new targeted regression tests in `tests/test_v0_5_completeness.py`.

Validated command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q
```

Result: `19 passed in 23.95s`.

## V0.6 Advantage-focused validation update

Added targeted validation cases designed to show the intended strengths of CALG without overstating universal runtime superiority:

- `make_shifted_paraboloid_patch`: analytic smooth sub-cell valley benchmark with true gap inside a mesh cell.
- `scripts/run_advantage_benchmarks.py`: runs the new advantage-focused suite.
- `results/v0_6_advantage/subcell_valley_accuracy.csv`: CALG vs linear BVH on sub-cell smooth contact.
- `results/v0_6_advantage/equal_accuracy_cost.csv`: coarse CALG vs refined linear meshes under an equal-accuracy framing.
- `results/v0_6_advantage/sliding_normal_continuity.csv`: sliding-normal continuity metrics.
- `docs/V0_6_ADVANTAGE_CASES.md`: interpretation and claims allowed by these new examples.

The PyTorch CUDA backend is now accepted by the GPU/TDI unit test.


## v0.7 final-strengthening additions

- Added randomized sub-cell smooth-contact statistics for under-resolved quadratic contact.
- Added contact force/friction smoothness benchmark comparing PL face normals and CALG smooth normals.
- Added dedicated fallback validation for graph failure, active-set 4D fallback, and interval/Bezier fallback.
- Added `scripts/run_final_strengthening.py` and `tests/test_final_strengthening.py`.
- Dedicated v0.7 test: `1 passed in 23.17s`.

Claim boundary: v0.7 supports CALG's advantages in under-resolved smooth-contact geometry and sliding-force continuity, but it should not be used to claim that the Python prototype is generally faster than optimized linear BVH.
