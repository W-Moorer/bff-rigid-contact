# CALG v0.4 Numerical-Example Completion Pack

This version adds the missing numerical-example infrastructure requested for the CMAME-oriented manuscript.  It does **not** fabricate CUDA workstation results.  The current sandbox has no CUDA/CuPy device, so GPU benchmark files generated here are explicitly marked as NumPy fallback.  The same script is ready to run on a CUDA workstation and will write `is_true_cuda=True` rows when a device is detected.

## Added modules

- `calg.experiments.gpu_benchmark`: vectorized AABB broad-phase benchmark with optional CuPy/CUDA backend and explicit backend metadata.
- `calg.experiments.convergence`: mesh-refinement convergence study on rotated sphere--plane, paraboloid--plane, and cylinder--plane cases.
- `calg.experiments.baselines`: CALG, linear triangle BVH, global subdivision linear contact, and open-source Trimesh sampled nearest-distance baselines.
- `calg.experiments.baseline_comparison`: automatic comparison tables for the baseline methods.
- `calg.experiments.robustness`: deterministic complex-surface robustness statistics using ruffled spheres, lobed rotor patches, and open wavy sheets.
- `calg.modeling.mesh_ops`: midpoint subdivision utility for global-subdivision cost baselines.

## Commands

```bash
PYTHONPATH=src pytest -q

# CUDA workstation benchmark; on this sandbox it records NumPy fallback.
PYTHONPATH=src python scripts/run_gpu_benchmark.py \
  --out-dir results/v0_4_gpu_benchmark \
  --sizes 512,1024,2048 \
  --repeats 2

PYTHONPATH=src python scripts/run_convergence.py \
  --out-dir results/v0_4_convergence \
  --resolutions 4,6,8 \
  --families sphere_plane,paraboloid_plane,cylinder_plane

PYTHONPATH=src python scripts/run_baseline_comparison.py \
  --out-dir results/v0_4_baselines

PYTHONPATH=src python scripts/run_robustness.py \
  --out-dir results/v0_4_robustness \
  --num-cases 21
```

## Generated outputs

- `results/v0_4_gpu_benchmark/gpu_broadphase_benchmark.csv`
- `results/v0_4_convergence/mesh_convergence.csv`
- `results/v0_4_baselines/baseline_comparison.csv`
- `results/v0_4_robustness/robustness_statistics.csv`

## Current readiness assessment

The numerical-example section can now be drafted with real CPU-side convergence, baseline, and robustness results.  CUDA acceleration results are structurally supported but remain pending until the script is run on the user's CUDA workstation.  This distinction must be preserved in the paper: current sandbox values are **not** GPU acceleration data.

## What still must be run externally before final CMAME submission

1. Run `scripts/run_gpu_benchmark.py` on the RTX 3080/CUDA workstation.
2. Repeat convergence and baseline studies with larger meshes if runtime allows.
3. Add the final industrial scene only after the analytic and robustness validations remain stable.
4. For strong production claims, replace the research-prototype TDI gate with a fully certified moving Bézier inclusion kernel and verify it on difficult CCD cases.
