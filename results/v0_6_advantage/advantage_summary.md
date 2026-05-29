# Advantage-focused validation suite

This suite targets regimes where the proposed method is expected to be useful: under-resolved smooth contact and sliding-normal continuity. It is not used to claim that the current Python prototype is faster than an optimized linear BVH on every case.

## Headline results

- Sub-cell valley, resolution 4: CALG gap error = 0; linear BVH gap error = 0.0301.
- The coarse CALG result reaches the target gap to machine precision, while linear refinements up to resolution 12 remain above the 1e-5 target tolerance in this experiment.
- Sliding normal total-variation ratio, linear/CALG = 1.5.
- Sliding normal maximum-jump ratio, linear/CALG = 33.2.

## Generated files

- `subcell_valley_accuracy.csv`
- `equal_accuracy_cost.csv`
- `sliding_normal_continuity.csv`
- `advantage_summary.json`
