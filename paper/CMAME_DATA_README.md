# CALG CMAME Validation Data README

## Summary

This dataset supports the manuscript `Contact-Aligned Local Graphs for Smooth
Rigid Frictional Contact on Coarse Surface Meshes`. It contains the source code,
scripts, locked numerical results, manuscript figures, and full-frame VTK
animation exports used to validate Contact-Aligned Local Graphs (CALG) against
Linear-triangle BVH, PN-triangle BVH, and Project Chrono SMC scenes. The executable
evidence is scoped to rigid frictional contact over smooth analytic or CAD-like
surfaces.

## File Organization

| Path | Contents |
|---|---|
| `src/calg/` | Python implementation of contact geometry, rigid friction response, analytic generators, and experiment logic |
| `cpp/` | CALG C++ validation executable and solver-loop timing code |
| `scripts/` | Reproduction, summarization, Chrono validation, VTK export, and manuscript-figure scripts |
| `tests/` | Regression tests for geometry, PN-triangle, engineering, VTK, and rigid-friction modules |
| `results/v0_6_advantage/` | Static sub-cell accuracy and sliding normal-continuity checks |
| `results/v0_9_engineering_rigid/` | Four engineering rigid-friction time series and summary metrics |
| `results/v0_10_chrono_validation/` | Chrono validation locks, C++ comparison source traces, and VTK exports |
| `results/timing/` | Current Chrono, RecurDyn CPU Time, and CALG C++ timing summaries |
| `results/v0_11_dynamic_backend_comparison/` | Same-scene CALG, PN-triangle BVH, and Linear-triangle BVH dynamic comparison tables |
| `results/v0_12_chrono_scene_backend_comparison/` | Contact-geometry diagnostics on accepted Chrono trajectories |
| `results/v0_13_graphability_fallback_diagnostics/` | Graphability, curved fallback, and interval fallback diagnostics |
| `results/v0_14_reproducibility/` | Reproducibility parameter table |
| `paper/figures/` | Manuscript figures exported as PNG, PDF, SVG, and TIFF |

The file-level checksum manifest is `paper/CMAME_DATA_MANIFEST.csv`. The tier
summary is `paper/CMAME_DATA_MANIFEST_SUMMARY.md`.

## Main Result Files

| File | Manuscript use |
|---|---|
| `results/v0_6_advantage/subcell_valley_accuracy.csv` | Static sub-cell valley gap-error comparison |
| `results/v0_6_advantage/sliding_normal_continuity.csv` | Static sliding normal-continuity comparison |
| `results/v0_9_engineering_rigid/*_timeseries.csv` | Four independent dynamic contact-geometry comparison curves |
| `results/v0_9_engineering_rigid/engineering_rigid_summary.csv` | Per-case rigid-friction metrics |
| `results/v0_10_chrono_validation/validation_locks.json` | Accepted Chrono-CALG validation correlations |
| `results/v0_10_chrono_validation/ball_joint_pendulum_impulse_composite_5s/three_way_equiv_payload_metrics.json` | 5 s deep ball-joint Chrono/CALG/RecurDyn equivalent-payload trajectory correlations |
| `results/cpp_backend_guide_sdf_3s/guide_slot_cpp.csv` | CALG C++ guide-slot trace with local mesh detector and dense tricubic SDF signed refinement |
| `results/cpp_backend_deep_ball_joint_mesh_5s/deep_ball_joint_pendulum_mesh_cpp.csv` | CALG C++ deep ball-joint trace with local mesh detector and dense tricubic SDF signed refinement |
| `results/cpp_backend_bearing_sdf_5s/bearing_rotating_inner_cpp.csv` | CALG C++ rotating-raceway trace with local mesh detector and dense tricubic SDF signed refinement |
| `results/recurdyn_validation_step_solid/ball_joint_pendulum_equiv_payload_5s/ball_joint_pendulum_step_solid_recurdyn_timeseries.csv` | RecurDyn STEP/SolidSolid deep ball-joint equivalent-payload validation series |
| `results/timing/current_case_timing_speedup.csv` | Chrono solver-call time, RecurDyn CPU Time, and CALG C++ solver-loop timing |
| `results/v0_11_dynamic_backend_comparison/dynamic_backend_ratios.csv` | Dynamic Linear/CALG and PN/CALG ratios |
| `results/v0_12_chrono_scene_backend_comparison/chrono_scene_backend_metrics.csv` | Direct diagnostics on Chrono-validated trajectories |
| `results/v0_13_graphability_fallback_diagnostics/graphability_fallback_metrics.csv` | Graphability/fallback pathway statistics |
| `results/v0_14_reproducibility/parameter_table.csv` | Primary simulation parameters reported in the manuscript |

## Variables and Units

### Static Accuracy and Equal-Accuracy Tables

Used in `subcell_valley_accuracy.csv`, `equal_accuracy_cost.csv`, and related
JSON files.

| Column | Definition | Unit |
|---|---|---|
| `study` | Study group identifier | categorical |
| `case` | Analytic contact case name | categorical |
| `method` | Contact-geometry method | categorical |
| `resolution` | Surface mesh resolution used for the comparison | count |
| `faces_a`, `faces_b` | Number of faces in each compared surface | count |
| `expected_gap` | Analytic or reference signed gap | length, simulation units |
| `min_gap` | Computed minimum signed gap | length, simulation units |
| `abs_error` | Absolute gap error relative to `expected_gap` | length, simulation units |
| `elapsed_s` | Runtime for the case | seconds |
| `contacts` | Number of reported contacts | count |
| `candidate_pairs` | Number of broad-phase candidate primitive pairs | count |
| `notes` | Protocol notes | text |

### Sliding Normal-Continuity Table

Used in `sliding_normal_continuity.csv`.

| Column | Definition | Unit |
|---|---|---|
| `method` | Contact-geometry method | categorical |
| `samples` | Number of path samples | count |
| `total_variation_rad` | Total variation of the contact-normal sequence | radians |
| `max_jump_rad` | Largest consecutive normal-angle jump | radians |
| `mean_jump_rad` | Mean consecutive normal-angle jump | radians |
| `rms_jump_rad` | Root-mean-square consecutive normal-angle jump | radians |
| `notes` | Protocol notes | text |

### Engineering Rigid-Friction Time Series

Used in `results/v0_9_engineering_rigid/*_timeseries.csv`.

| Column group | Columns | Definition | Unit |
|---|---|---|---|
| identity | `case`, `method`, `step` | Case name, method name, and timestep index | categorical/count |
| time | `time` | Physical simulation time | seconds |
| position | `x`, `y`, `z` | Rigid body centre position | length, simulation units |
| linear velocity | `vx`, `vy`, `vz` | Rigid body translational velocity | length/second |
| angular velocity | `wx`, `wy`, `wz` | Rigid body angular velocity | radians/second |
| contact gap | `gap` | Signed contact gap | length, simulation units |
| normal | `normal_x`, `normal_y`, `normal_z` | Unit contact normal components | dimensionless |
| normal force | `normal_force_x`, `normal_force_y`, `normal_force_z`, `normal_force` | Normal contact force vector and norm | force, simulation units |
| friction force | `friction_force_x`, `friction_force_y`, `friction_force_z`, `friction_force` | Tangential friction force vector and norm | force, simulation units |
| torques | `friction_torque_*`, `total_torque_*`, `friction_torque`, `total_torque` | Friction and total torque components/norms | torque, simulation units |
| speeds | `tangential_speed`, `slip_speed` | Tangential contact speed and rolling/slip mismatch | length/second |
| energy | `kinetic_energy`, `total_energy` | Rigid body kinetic and total energy | energy, simulation units |
| state | `friction_mode`, `ccd_clamped` | Friction state and continuous-collision clamp flag | categorical/Boolean |

### Engineering and Dynamic Summary Tables

Used in `engineering_rigid_summary.csv`, `dynamic_backend_metrics.csv`, and
`chrono_scene_backend_metrics.csv`.

| Column | Definition | Unit |
|---|---|---|
| `max_normal_angle_jump_deg` | Maximum consecutive contact-normal angle jump | degrees |
| `max_normal_force_jump` | Maximum consecutive normal-force norm jump | force, simulation units |
| `max_friction_force_jump` | Maximum consecutive friction-force norm jump | force, simulation units |
| `max_friction_torque_jump` | Maximum consecutive friction-torque norm jump | torque, simulation units |
| `tangential_speed_jitter` | Jitter measure for tangential speed | length/second |
| `angular_speed_jitter` | Jitter measure for angular speed | radians/second |
| `stick_slip_switches` | Number of discrete stick/slip state changes | count |
| `sliding_distance` | Accumulated sliding path length | length, simulation units |
| `min_gap` | Minimum signed gap over the trajectory | length, simulation units |
| `energy_loss` | Energy decrease over the trajectory | energy, simulation units |
| `steps_per_second` | Python prototype throughput | steps/second |

Ratio tables use the prefix `linear_over_calg_*` or `pn_over_calg_*` for
baseline/CALG ratios. Values above 1 indicate larger jump, jitter, or runtime
metric in the baseline. Columns ending in `_minus_calg_stick_slip_switches`
report absolute count differences.

### Chrono, RecurDyn, and CALG C++ Timing Tables

Used in `results/timing/current_case_timing_speedup.csv` and
`results/timing/current_case_timing_summary.csv`.

| Column | Definition | Unit |
|---|---|---|
| `case` | Validation scene | categorical |
| `steps` | Number of simulated steps | count |
| `chrono_ms` | Project Chrono SMC solver-call wall time | milliseconds |
| `recurdyn_cpu_ms` | RecurDyn solver-reported CPU Time | milliseconds |
| `calg_wall_ms` | CALG C++ no-CSV solver-loop time; one-time geometry preprocessing such as dense SDF construction is excluded | milliseconds |
| `calg_internal_ms` | Internal CALG C++ reported solve time | milliseconds |
| `speedup_vs_chrono_wall` | Chrono wall time divided by CALG C++ solver-loop time | dimensionless |
| `speedup_vs_recurdyn_wall` | RecurDyn CPU Time divided by CALG C++ solver-loop time | dimensionless |
| `avg_contacts` | Average CALG contacts per step | count |
| `avg_candidate_pairs` | Average candidate primitive pairs per step | count |

### Graphability and Fallback Diagnostics

Used in `graphability_fallback_metrics.csv`.

| Column | Definition | Unit |
|---|---|---|
| `case` | Diagnostic case | categorical |
| `steps` | Number of replay or control samples | count |
| `candidate_pairs` | Candidate primitive pairs considered | count |
| `graph_attempted`, `graph_passed`, `graph_contacts` | Graph-query attempts, certificate passes, and graph contacts | count |
| `curved_attempted`, `curved_contacts` | Generic curved patch solve attempts and contacts | count |
| `interval_attempted`, `interval_contacts` | Interval fallback attempts and contacts | count |
| `contacts` | Total final contacts | count |
| `graph_pass_rate` | `graph_passed / graph_attempted` | dimensionless |
| `avg_candidate_pairs_per_step` | Average candidate pairs per step | count/step |
| `avg_contacts_per_step` | Average contacts per step | count/step |
| `dominant_path` | Dominant query pathway | categorical |
| `notes` | Protocol notes | text |

### Reproducibility Parameter Table

Used in `parameter_table.csv`.

| Column | Definition | Unit |
|---|---|---|
| `suite` | Experiment suite | categorical |
| `case` | Case name | categorical |
| `steps` | Number of simulated steps | count |
| `dt` | Timestep size | seconds |
| `friction_mu` | Coulomb friction coefficient | dimensionless |
| `body_radius` | Rigid body radius | length, simulation units |
| `d_hat` | Contact activation/detector distance | length, simulation units |
| `normal_stiffness` | Normal penalty stiffness used in the replay | force/length, simulation units |
| `surface_resolution` | Mesh or analytic surface resolution descriptor | text |
| `source` | Script or module that generated the case | path |

## Methods and Provenance

The result files were generated by the scripts and modules in this repository.
The manuscript tables and figures are not edited manually; they are derived
from the locked CSV/JSON artifacts.

| Output | Reproduction entry point |
|---|---|
| Static advantage results | `scripts/run_advantage_benchmarks.py` and `src/calg/experiments/advantage.py` |
| Four rigid-friction engineering scenes | `scripts/run_engineering_rigid_cases.py` and `src/calg/experiments/engineering_rigid.py` |
| Dynamic CALG/PN-triangle BVH/Linear-triangle BVH summaries | `scripts/summarize_dynamic_backend_comparison.py` |
| Chrono validation locks | `scripts/check_chrono_validation_locks.py` and Chrono scene scripts |
| Chrono-scene geometry diagnostics | `scripts/summarize_chrono_scene_backend_comparison.py` |
| Graphability/fallback diagnostics | `scripts/summarize_graphability_fallback_diagnostics.py` |
| Paper figures | `scripts/make_paper_story_figures.py` |
| Data manifest | `scripts/make_cmame_data_manifest.py` |

## Software and Environment

- Python package layout: `src/calg`
- Python requirements: `numpy`; optional extras for plots/tests are listed in
  `pyproject.toml`
- Test command: `PYTHONPATH=src python -m pytest -q`
- Figure generation: Python/matplotlib, configured to export Times-family text
  in SVG/PDF-compatible formats
- LaTeX build: run `pdflatex -interaction=nonstopmode calg_cmame_draft.tex`
  from the `paper/` directory
- C++ timing executable: stored under `cpp/`
- Chrono validation scripts require a Project Chrono-capable environment

## Access and Licence

The repository is prepared for deposition in a DOI-bearing archive. Code is
distributed under the BSD 3-Clause licence. Generated numerical data and figures
should be deposited under an open data licence such as CC-BY-4.0 or the
repository-equivalent licence, subject to final author and institutional
approval.

## Citation

Before journal submission, replace this draft citation with the final DataCite
record assigned by the selected repository:

```text
W-Moorer (2026). Contact-Aligned Local Graphs validation data for smooth rigid
frictional contact on coarse surface meshes. Repository name, version
2026-manuscript-validation-package. DOI to be inserted after deposition.
```
