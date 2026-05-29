# V0.6 Advantage-focused validation cases

This version adds targeted, fair validation cases that emphasize the intended strengths of the contact-aligned local graph method without claiming that the current Python prototype is universally faster than a linear BVH.

## What changed

### 1. Sub-cell smooth valley benchmark

A new analytic generator `make_shifted_paraboloid_patch` places the true minimum gap inside a mesh cell rather than at a vertex. This exposes a known weakness of a piecewise-linear contact surface: it may miss the sub-cell smooth minimum until the mesh is refined. The CALG curved-jet path uses vertex normals to reconstruct the local curvature.

Generated results:

- `results/v0_6_advantage/subcell_valley_accuracy.csv`
- `results/v0_6_advantage/equal_accuracy_cost.csv`

Headline result: at resolution 4, CALG obtains machine-precision gap for the shifted paraboloid-plane gap, while linear BVH has an absolute gap error of about `3.01e-2`.

### 2. Equal-accuracy comparison

The new equal-accuracy study compares coarse CALG against regenerated finer linear meshes. It avoids the misleading same-mesh-only comparison. The goal is to answer: how much linear refinement is required to match the accuracy reached by the curved-jet coarse representation?

In the current result, the coarse CALG case reaches the target tolerance, while linear refinements up to resolution 12 remain above the `1e-5` target tolerance.

### 3. Sliding normal continuity

The new sliding test compares piecewise-constant face normals against the smooth normal field encoded by the curved-jet representation. This targets a surface-to-surface contact advantage that min-gap-only tests cannot show: reduced normal jumps during sliding.

Generated result:

- `results/v0_6_advantage/sliding_normal_continuity.csv`

Headline result: the maximum normal jump of the linear face-normal sequence is about 33 times larger than the CALG smooth-normal sequence.

## New entry point

```bash
PYTHONPATH=src python scripts/run_advantage_benchmarks.py --out-dir results/v0_6_advantage
```

## Important interpretation

These cases are intended to support accurate claims such as:

- CALG can improve accuracy per primitive in under-resolved smooth contact.
- CALG can reduce contact-normal jumps during sliding.
- CALG should be compared against refined linear meshes at equal accuracy, not only against a same-mesh linear BVH.

They should not be used to claim that the current Python prototype is faster than an optimized linear BVH for every contact problem.
