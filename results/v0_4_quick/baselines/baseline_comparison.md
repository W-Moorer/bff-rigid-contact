# Baseline comparison

Methods: CALG curved graph, linear triangle BVH, global midpoint-subdivision linear contact, and an optional open-source Trimesh sampled nearest-distance baseline.

| case | method | expected | min gap | abs error | time (s) | contacts | candidates/samples | notes |
|---|---|---:|---:|---:|---:|---:|---:|---|
| plane_plane_gap | calg_curved_graph | 0.08 | 0.08 | 0.0 | 1.392 | 328 | 400 | fast_path_ratio=0.82; curved_attempted=264; interval_attempted=0 |
| plane_plane_gap | linear_triangle_bvh | 0.08 | 0.08 | 0.0 | 0.1404 | 328 | 400 |  |
| plane_plane_gap | trimesh_nearest_sampled | 0.08 | 0.08 | 0.0 | 0.08347 | 306 | 306 | open_source_baseline=trimesh.closest_point_naive |
| plane_plane_gap | global_subdivision_linear_L1 | 0.08 | 0.08 | 0.0 | 1.695 | 1544 | 4624 | faces_a=128; faces_b=128 |
| sphere_plane_gap | calg_curved_graph | 0.08 | 0.0785587 | 0.0014412853004546061 | 0.5018 | 60 | 596 | fast_path_ratio=0.107383; curved_attempted=46; interval_attempted=0 |
| sphere_plane_gap | linear_triangle_bvh | 0.08 | 0.08 | 6.938893903907228e-17 | 0.1199 | 60 | 168 |  |
| sphere_plane_gap | trimesh_nearest_sampled | 0.08 | 0.08 | 6.938893903907228e-17 | 0.2397 | 2 | 699 | open_source_baseline=trimesh.closest_point_naive |
| sphere_plane_gap | global_subdivision_linear_L1 | 0.08 | 0.08 | 6.938893903907228e-17 | 0.5963 | 60 | 1088 | faces_a=320; faces_b=288 |
| paraboloid_plane_gap | calg_curved_graph | 0.06 | 0.06 | 2.7755575615628914e-17 | 1.429 | 180 | 1008 | fast_path_ratio=0.190476; curved_attempted=100; interval_attempted=0 |
| paraboloid_plane_gap | linear_triangle_bvh | 0.06 | 0.06 | 0.0 | 0.4189 | 176 | 1008 |  |
| paraboloid_plane_gap | trimesh_nearest_sampled | 0.06 | 0.06 | 0.0 | 0.1919 | 94 | 674 | open_source_baseline=trimesh.closest_point_naive |
| paraboloid_plane_gap | global_subdivision_linear_L1 | 0.06 | 0.06 | 0.0 | 3.397 | 514 | 8528 | faces_a=288; faces_b=288 |
