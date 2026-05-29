# CALG v0.7 final-strengthening validation for CMAME manuscript

This version adds three validation blocks intended to close the strongest remaining gaps in the CMAME draft.

## 1. Randomized sub-cell smooth contact statistics

File: `src/calg/experiments/final_strengthening.py`

Command:

```bash
PYTHONPATH=src python scripts/run_final_strengthening.py --out-dir results/v0_7_final_strengthening
```

Output:

- `results/v0_7_final_strengthening/randomized_subcell_statistics.csv`
- `results/v0_7_final_strengthening/randomized_subcell_statistics.json`
- `results/v0_7_final_strengthening/figures/fig_randomized_subcell_statistics.pdf`

Purpose: validate the intended advantage of curved jets in under-resolved smooth contact. Each randomized case places the apex of a quadratic paraboloid inside a grid cell so that the true minimum gap is not located at a mesh vertex. The curved-jet representation is exact for the quadratic local geometry, while the piecewise-linear baseline can only attain a minimum at sampled vertices.

Important scope note: this is a primitive-level geometric accuracy benchmark, not a full industrial contact-dynamics benchmark. It supports the subdivision-free high-order geometry claim, not a blanket statement that the full prototype is faster than linear BVH in all cases.

## 2. Contact force / friction smoothness

Outputs:

- `results/v0_7_final_strengthening/contact_force_friction_smoothness.csv`
- `results/v0_7_final_strengthening/figures/fig_contact_force_friction_smoothness.pdf`

Purpose: validate the mechanical relevance of the smooth normal field. The same regularized contact/friction response is evaluated along a sliding path using either piecewise-linear face normals or the smooth curved-jet normal field. The main metric is maximum total-force jump; total variation is also reported.

Current result: the PL/CALG max total-force jump ratio is about 55.9, and the PL/CALG friction-force variation ratio is about 1.19. Total variation is not lower for every metric, so the manuscript should emphasize maximum jump / local force discontinuity rather than claiming universally lower force variation.

## 3. Dedicated fallback validation

Outputs:

- `results/v0_7_final_strengthening/fallback_validation.csv`
- `results/v0_7_final_strengthening/figures/fig_fallback_validation.pdf`

Purpose: validate the hierarchy:

```text
graph certificate -> 2D graph path -> active-set 4D curved fallback -> interval / Bezier fallback
```

The suite includes:

1. a strict graphability-fail cylinder-plane case that forces 4D curved fallback;
2. a direct interval/Bezier fallback test on a near-contact curved primitive pair;
3. a graphable plane-plane control case.

## Test status

The dedicated v0.7 test passes:

```text
1 passed in 23.17s
```

See `results/v0_7_final_strengthening/v0_7_final_strengthening_pytest_log.txt`.

## Recommended manuscript claim boundary

Supported by v0.7:

- CALG is advantageous for under-resolved smooth contact where a piecewise-linear mesh misses sub-cell geometric extrema.
- CALG smooth normals reduce large local force jumps during sliding contact.
- The fallback hierarchy is operational and can be exercised by dedicated tests.

Not yet supported as a final claim:

- CALG is generally faster than optimized linear BVH.
- The full prototype is a production-ready frictional dynamics engine.
- The interval fallback provides a complete analytical CCD guarantee for arbitrary deforming curved surfaces.
