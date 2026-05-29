# Baseline comparison

Methods: CALG curved graph, linear triangle BVH, global midpoint-subdivision linear contact, and an optional open-source Trimesh sampled nearest-distance baseline.

| case | method | expected | min gap | abs error | time (s) | contacts | candidates/samples | notes |
|---|---|---:|---:|---:|---:|---:|---:|---|
| plane_plane_gap | calg_curved_graph | 0.08 | 0.08 | 0.0 | 1.404 | 328 | 400 | fast_path_ratio=0.82; curved_attempted=264; interval_attempted=0 |
| plane_plane_gap | linear_triangle_bvh | 0.08 | 0.08 | 0.0 | 0.1695 | 328 | 400 |  |
| plane_plane_gap | trimesh_nearest_sampled | 0.08 | 0.08 | 0.0 | 0.08822 | 306 | 306 | open_source_baseline=trimesh.closest_point_naive |
| plane_plane_gap | global_subdivision_linear_L1 | 0.08 | 0.08 | 0.0 | 1.742 | 1544 | 4624 | faces_a=128; faces_b=128 |
| sphere_plane_gap | calg_curved_graph | 0.08 | 0.0785587 | 0.0014412853004546061 | 0.5206 | 60 | 596 | fast_path_ratio=0.107383; curved_attempted=46; interval_attempted=0 |
| sphere_plane_gap | linear_triangle_bvh | 0.08 | 0.08 | 6.938893903907228e-17 | 0.1052 | 60 | 168 |  |
| sphere_plane_gap | trimesh_nearest_sampled | 0.08 | 0.08 | 6.938893903907228e-17 | 0.2351 | 2 | 699 | open_source_baseline=trimesh.closest_point_naive |
| sphere_plane_gap | global_subdivision_linear_L1 | 0.08 | 0.08 | 6.938893903907228e-17 | 0.6146 | 60 | 1088 | faces_a=320; faces_b=288 |
| paraboloid_plane_gap | calg_curved_graph | 0.06 | 0.06 | 2.7755575615628914e-17 | 1.544 | 180 | 1008 | fast_path_ratio=0.190476; curved_attempted=100; interval_attempted=0 |
| paraboloid_plane_gap | linear_triangle_bvh | 0.06 | 0.06 | 0.0 | 0.4123 | 176 | 1008 |  |
| paraboloid_plane_gap | trimesh_nearest_sampled | 0.06 | 0.06 | 0.0 | 0.2035 | 94 | 674 | open_source_baseline=trimesh.closest_point_naive |
| paraboloid_plane_gap | global_subdivision_linear_L1 | 0.06 | 0.06 | 0.0 | 3.775 | 514 | 8528 | faces_a=288; faces_b=288 |
| open_wavy_sheet_plane | calg_curved_graph | 0.04 | 0.0410664 | 0.0010663964108982224 | 2.687 | 489 | 1024 | fast_path_ratio=0.488281; curved_attempted=146; interval_attempted=0 |
| open_wavy_sheet_plane | linear_triangle_bvh | 0.04 | 0.0426795 | 0.002679491924311224 | 0.4643 | 478 | 1024 |  |
| open_wavy_sheet_plane | trimesh_nearest_sampled | 0.04 | 0.0426795 | 0.002679491924311224 | 0.2142 | 619 | 674 | open_source_baseline=trimesh.closest_point_naive |
| open_wavy_sheet_plane | global_subdivision_linear_L1 | 0.04 | 0.0426795 | 0.002679491924311224 | 3.307 | 2704 | 8464 | faces_a=288; faces_b=288 |
| cylinder_plane_line_contact_small | calg_curved_graph | 0.05 | 0.0493703 | 0.00062968513335529 | 2.891 | 344 | 1152 | fast_path_ratio=0.322917; curved_attempted=216; interval_attempted=0 |
| cylinder_plane_line_contact_small | linear_triangle_bvh | 0.05 | 0.0744717 | 0.02447174185242322 | 0.355 | 252 | 768 |  |
| cylinder_plane_line_contact_small | trimesh_nearest_sampled | 0.05 | 0.0744717 | 0.02447174185242322 | 0.3086 | 183 | 963 | open_source_baseline=trimesh.closest_point_naive |
