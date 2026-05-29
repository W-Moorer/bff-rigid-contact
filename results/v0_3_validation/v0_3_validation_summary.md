# CALG v0.3 validation summary

GPU available on this host: `False`

| case | passed | key result |
|---|---:|---|
| gpu_vectorized_broadphase | True | backend=numpy, pairs=1 |
| tdi_ccd_moving_plane | True | toi=[0.712890625, 0.71484375], expected=0.714286, reason=endpoint_threshold_contact |
| implicit_global_step_with_tdi_gate | True | explicit_gap=0.083853, ccd_clamped=True |
