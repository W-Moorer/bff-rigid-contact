# Current Case Timing Summary

| Case | Solver | Scope | Steps | Sim time (s) | Total (ms) | us/step |
|---|---|---|---:|---:|---:|---:|
| Guide slot | chrono_smc | Project Chrono SMC solver call | 6000 | 3.000 | 868.311 | 144.718 |
| Guide slot | calg_cpp | native C++ solver loop; excludes one-time dense SDF build; uses mesh detector, tricubic SDF signed refinement, and normalized surface-contact response quadrature | 6000 | 3.000 | 661.168 | 110.195 |
| Guide slot | calg_cpp_with_csv_trace | native C++ solver loop with full time-series CSV trace; excludes one-time dense SDF build | 6000 | 3.000 | 743.777 | 123.963 |
| Guide slot | recurdyn_solidsolid_cpu_time | RecurDyn reported CPU Time | 6000 | 3.000 | 12400.000 | 2066.667 |
| Ball-and-socket pendulum | chrono_smc | Project Chrono SMC solver call | 25000 | 5.000 | 150218.881 | 6008.755 |
| Ball-and-socket pendulum | calg_cpp | native C++ solver loop; excludes one-time dense SDF build; uses mesh detector, tricubic SDF signed refinement, and normalized surface-contact response quadrature | 25000 | 5.000 | 20978.700 | 839.148 |
| Ball-and-socket pendulum | calg_cpp_with_csv_trace | native C++ solver loop with full time-series CSV trace; excludes one-time dense SDF build | 25000 | 5.000 | 21481.800 | 859.272 |
| Ball-and-socket pendulum | recurdyn_solidsolid_cpu_time | RecurDyn reported CPU Time | 25000 | 5.000 | 454830.000 | 18193.200 |
| Bearing raceway | chrono_smc | Project Chrono SMC solver call | 10000 | 5.000 | 49426.548 | 4942.655 |
| Bearing raceway | calg_cpp | native C++ solver loop; excludes one-time dense SDF build; uses mesh detector, tricubic SDF signed refinement, and normalized surface-contact response quadrature | 10000 | 5.000 | 18657.600 | 1865.760 |
| Bearing raceway | calg_cpp_with_csv_trace | native C++ solver loop with full time-series CSV trace; excludes one-time dense SDF build | 10000 | 5.000 | 19059.800 | 1905.980 |
| Bearing raceway | recurdyn_solidsolid_cpu_time | RecurDyn reported CPU Time | 10000 | 5.000 | 61190.000 | 6119.000 |

| Case | CALG basis | Steps | CALG C++ solver (ms) | Chrono (ms) | Chrono/CALG | RecurDyn CPU (ms) | RecurDyn/CALG |
|---|---|---:|---:|---:|---:|---:|
| Guide slot | CALG C++ surface patch + tricubic SDF | 6000 | 661.168 | 868.311 | 1.313x | 12400.000 | 18.755x |
| Ball-and-socket pendulum | CALG C++ surface patch + tricubic SDF | 25000 | 20978.700 | 150218.881 | 7.161x | 454830.000 | 21.681x |
| Bearing raceway | CALG C++ surface patch + tricubic SDF | 10000 | 18657.600 | 49426.548 | 2.649x | 61190.000 | 3.280x |
