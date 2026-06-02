# CALG Coarse-Detector Snapshot Exports

These files are one-step debug exports from the current C++ backend. They
visualize the local detector meshes used by the inflated-AABB coarse detector
before the tricubic SDF signed-gap response refinement.

Selected frames:

| Case | Step | Time (s) | VTP file prefix |
|---|---:|---:|---|
| Guide slot | 2835 | 1.4175 | `guide_slot_cpp_step2835_guide_track_detector_snapshot` |
| Ball-and-socket pendulum | 6184 | 1.2368 | `deep_ball_joint_pendulum_mesh_cpp_step6184_finite_socket_inner_detector_snapshot` |
| Bearing raceway, inner side | 5000 | 2.5000 | `bearing_rotating_inner_cpp_step5000_bearing_inner_detector_snapshot` |
| Bearing raceway, outer side | 5000 | 2.5000 | `bearing_rotating_inner_cpp_step5000_bearing_outer_detector_snapshot` |

Each prefix is split into independent VTP files:

| Suffix | Content | `part_id` values |
|---|---|---|
| `_terrain_patch.vtp` | fixed/terrain local detector mesh | 1: regular triangle; 3: AABB-candidate triangle; 10: detector-accepted triangle |
| `_body_patch.vtp` | moving-body local detector mesh | 2: regular triangle; 4: AABB-candidate triangle; 11: detector-accepted triangle |
| `_terrain_aabb.vtp` | inflated AABB line boxes for candidate fixed/terrain triangles | 5 |
| `_body_aabb.vtp` | inflated AABB line boxes for candidate moving-body triangles | 6 |
| `_closest_segments.vtp` | initial closest-point segments for all AABB candidate pairs | 7 |
| `_accepted_closest_segments.vtp` | initial closest-point segments for detector-accepted pairs | 7 |
| `_contact_segments.vtp` | curved-graph closest segments after local solve | 8 |
| `_representative_normal.vtp` | one representative detector normal | 9 |
| `_sdf_gap_active_terrain_patch.vtp` | final response cells where SDF signed gap is non-positive on fixed/terrain side | 12 |
| `_sdf_gap_active_body_patch.vtp` | final response cells where SDF signed gap is non-positive on moving-body side | 13 |
| `_normal_force_active_terrain_patch.vtp` | final response cells where normal force is positive on fixed/terrain side | 14 |
| `_normal_force_active_body_patch.vtp` | final response cells where normal force is positive on moving-body side | 15 |
| `_normal_force_active_normals.vtp` | final positive normal-force directions | 16 |

The companion `_index.csv` files record the case, pair id, selected step, time,
detector gap radius, local face counts, AABB candidate-pair count, graph-contact
count, detector-accepted face counts, and the split VTP filenames. The
detector-accepted counts mean pairs inside the CALG detector radius after the
curved graph solve. They are not the final signed-gap force-active contact area,
which is evaluated later by the SDF response field.

The companion `_response_active_index.csv` files record the final response
sample count, the number of samples with `SDF gap <= 0`, the number of samples
with positive normal force, the minimum response gap, and the maximum normal
force. These are the layers to use when the figure needs to show physically
active contact rather than detector candidates.

The Python review figures in `paper/figures/detector_snapshots` include separate
`*_closest_normal.*` files. They draw only the patch VTP layers, the gray initial
closest-point segments, and the green representative normal. The AABB overlay
figures remain separate.
