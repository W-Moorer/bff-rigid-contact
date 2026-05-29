# Numerical Examples Readiness for CALG Manuscript

## Status

The project now contains enough infrastructure and generated CPU-side data to write a **preliminary numerical examples section**.  The final CMAME submission still needs true CUDA workstation timing and larger-scale production runs.

## Completed in v0.4

1. Mesh refinement convergence experiments.
2. Baseline comparison against:
   - CALG curved graph contact,
   - linear triangle BVH contact,
   - global subdivision linear contact,
   - open-source Trimesh sampled nearest-distance baseline.
3. Complex-surface robustness statistics on deterministic generated shapes.
4. GPU benchmark harness with explicit CUDA/NumPy backend metadata.
5. Test coverage for the v0.4 benchmark harnesses.

## Current generated files

- `results/v0_4_convergence/mesh_convergence.csv`
- `results/v0_4_baselines/baseline_comparison.csv`
- `results/v0_4_robustness/robustness_statistics.csv`
- `results/v0_4_gpu_benchmark/gpu_broadphase_benchmark.csv`

## Important limitation

The current environment has no CUDA device, so the GPU CSV is a NumPy fallback benchmark.  It is useful for checking the benchmark pipeline, but it must not be reported as GPU speedup.  To obtain actual GPU acceleration data, run:

```bash
PYTHONPATH=src python scripts/run_gpu_benchmark.py --out-dir results/v0_4_gpu_benchmark_cuda --sizes 1024,2048,4096,8192 --repeats 5
```

on the CUDA workstation.

## Manuscript recommendation

The numerical examples can now be written as:

- analytic validation and convergence,
- comparison with linear and globally subdivided contact,
- comparison with an open-source Trimesh proximity baseline,
- robustness statistics on complex surfaces,
- GPU benchmark protocol with CUDA results pending.

Do not claim final GPU acceleration or strict industrial-scale production readiness until external CUDA and industrial-scene runs are completed.


## v0.5 update

The formerly incomplete algorithmic items have been hardened at research-prototype level: active-set curved Newton, conservative interval fallback behavior, conservative TDI-CCD gate, and stateful frictional response. These modules can now be included in preliminary numerical examples. For final CMAME numerical claims, the remaining requirements are still: true CUDA workstation timing, external baseline replication, and at least one industrial-scale scenario after all earlier validation gates pass.
