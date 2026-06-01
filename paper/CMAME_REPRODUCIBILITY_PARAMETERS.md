# CMAME Reproducibility Parameters

This table mirrors Table `tab:reproducibility_parameters` in the CMAME draft.

| suite | case | steps | dt | mu | body radius | d_hat | k_n | surface resolution |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| dynamic backend | bearing raceway | 700 | 0.0011 | 0.36 | 0.075 | 0.065 | 2550 | analytic raceway, coarse backend mesh |
| dynamic backend | cam-roller follower | 720 | 0.0010 | 0.40 | 0.085 | 0.070 | 2800 | analytic cam track, coarse backend mesh |
| dynamic backend | wavy guide slot | 680 | 0.0012 | 0.46 | 0.090 | 0.070 | 2400 | analytic wave/groove, coarse backend mesh |
| dynamic backend | spherical socket | 700 | 0.0010 | 0.44 | 0.105 | 0.075 | 3000 | analytic spherical seat, coarse backend mesh |
| Chrono/RecurDyn validation | guide slot | 6000 | 0.0005 | 0.40 | 0.090 | 0.002 contact / 0.018 detector | 80000 | 3 s window; 120x50 track mesh |
| Chrono/RecurDyn validation | deep ball-joint pendulum | 25000 | 0.0002 | 0.35 | 0.340 | 0 contact / 0.0002 clearance | 200000 | 5 s window; 64x144 socket mesh; RecurDyn equivalent payload SolidSolid k=1.0e8, c=1.5e4 |
| Chrono/RecurDyn validation | rotating raceway | 10000 | 0.0005 | 0.08 | 0.1612 | 0 contact / 0.010 detector | 700 | 5 s window; 192x18 raceway strip mesh; 2.0e-4 two-sided preload |
