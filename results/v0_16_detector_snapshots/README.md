# CALG Coarse-Detector Snapshot Exports

These files are one-step debug exports from the current C++ backend.  They
visualize the local detector meshes used by the inflated-AABB coarse detector
before the tricubic SDF signed-gap response refinement.

Selected frames:

| Case | Step | Time (s) | VTP files |
|---|---:|---:|---|
| Guide slot | 2835 | 1.4175 | `guide_slot_cpp_step2835_guide_track_detector_snapshot.vtp` |
| Ball-and-socket pendulum | 6184 | 1.2368 | `deep_ball_joint_pendulum_mesh_cpp_step6184_finite_socket_inner_detector_snapshot.vtp` |
| Bearing raceway | 5000 | 2.5000 | `bearing_rotating_inner_cpp_step5000_bearing_inner_detector_snapshot.vtp`, `bearing_rotating_inner_cpp_step5000_bearing_outer_detector_snapshot.vtp` |

VTP `part_id` values:

| `part_id` | Meaning |
|---:|---|
| 1 | regular terrain/local fixed patch triangles |
| 2 | regular moving-body patch triangles |
| 3 | terrain triangles that participate in an AABB candidate pair |
| 4 | moving-body triangles that participate in an AABB candidate pair |
| 5 | inflated AABB edges for candidate terrain triangles |
| 6 | inflated AABB edges for candidate moving-body triangles |
| 7 | initial linear closest segment for each AABB candidate pair |
| 8 | accepted curved-graph contact segment |
| 9 | representative detector normal |

The companion CSV files record the case, pair id, selected step, time, detector
gap radius, local face counts, AABB candidate-pair count, and graph-contact
count.  These exports are intended for mechanism visualization and should not
be interpreted as additional dynamics results.
