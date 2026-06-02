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
| `_terrain_patch.vtp` | fixed/terrain local detector mesh | 1: regular triangle; 3: AABB-candidate triangle |
| `_body_patch.vtp` | moving-body local detector mesh | 2: regular triangle; 4: AABB-candidate triangle |
| `_terrain_aabb.vtp` | inflated AABB line boxes for candidate fixed/terrain triangles | 5 |
| `_body_aabb.vtp` | inflated AABB line boxes for candidate moving-body triangles | 6 |

The companion `_index.csv` files record the case, pair id, selected step, time,
detector gap radius, local face counts, AABB candidate-pair count, graph-contact
count, and the four split VTP filenames. These exports are intended for
mechanism visualization and should not be interpreted as additional dynamics
results.

The previous combined debug lines are intentionally omitted here. The old red
line was the accepted curved-graph contact segment, gray lines were initial
closest segments, and the green line was the representative detector normal.
Those are useful for debugging but are not included in the review snapshots so
that the patch and AABB layers can be overlaid cleanly in ParaView.
