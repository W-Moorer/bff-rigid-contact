# Static patch-contact equivalence check

The scene uses two graphable but geometrically complex analytic patches. Their signed graph gap is prescribed so that the active contact set is a small ellipse, allowing area, gap, pressure-center, and average-normal quantities to be compared.

| quantity | value |
|---|---:|
| exact_active_area_m2 | 1.696460032938e-01 |
| calg_active_area_m2 | 1.696460032938e-01 |
| active_area_abs_error_m2 | 2.775557561563e-17 |
| active_area_rel_error | 1.636087799107e-16 |
| exact_int_gap_m3 | -2.261946710585e-05 |
| calg_int_gap_m3 | -2.261946710585e-05 |
| int_gap_abs_error_m3 | 3.049318610115e-20 |
| int_gap_rel_error | 1.348094805172e-15 |
| exact_mean_gap_m | -1.333333333333e-04 |
| calg_mean_gap_m | -1.333333333333e-04 |
| mean_gap_abs_error_m | 1.626303258728e-19 |
| exact_normal_force_N | 4.523893421169e+01 |
| calg_normal_force_N | 4.523893421169e+01 |
| normal_force_abs_error_N | 5.684341886081e-14 |
| normal_force_rel_error | 1.256515429714e-15 |
| pressure_center_error_m | 1.016968229659e-15 |
| average_normal_angle_error_deg | 0.000000000000e+00 |
| reference_pressure_center_x_m | 1.600000000000e-01 |
| reference_pressure_center_y_m | -1.200000000000e-01 |
| reference_pressure_center_z_m | -1.606618384432e-02 |
| calg_pressure_center_x_m | 1.600000000000e-01 |
| calg_pressure_center_y_m | -1.200000000000e-01 |
| calg_pressure_center_z_m | -1.606618384432e-02 |
| reference_average_normal_x | 1.262796819559e-01 |
| reference_average_normal_y | -1.720673907123e-02 |
| reference_average_normal_z | 9.918454365755e-01 |
| calg_average_normal_x | 1.262796819559e-01 |
| calg_average_normal_y | -1.720673907123e-02 |
| calg_average_normal_z | 9.918454365755e-01 |
| reference_radial_order | 180 |
| reference_theta_order | 720 |
| calg_radial_order | 36 |
| calg_theta_order | 144 |
| active_band_min_NA_dot_nc | 9.728944969496e-01 |
| active_band_min_minus_NB_dot_nc | 9.726268934756e-01 |
| active_band_sample_count | 19440 |

The CALG graph-region quadrature reproduces the analytic active area, gap integral, mean gap, and normal force, while the pressure-weighted center and average normal agree with the high-order reference quadrature.