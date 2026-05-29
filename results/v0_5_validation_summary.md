# CALG v0.5 validation summary

| item | status | evidence |
|---|---|---|
| Complete curved 4D active-set Newton | passed | `tests/test_v0_5_completeness.py::test_complete_active_set_curved_newton_finds_boundary_feature` |
| Conservative interval fallback | passed | Existing interval fallback tests and branch-and-bound certification logic |
| Conservative TDI CCD gate | passed | `tests/test_v0_5_completeness.py::test_tdi_gate_is_conservative_for_crossing_planes` |
| Stateful frictional contact response | passed | `tests/test_v0_5_completeness.py::test_stateful_friction_projects_to_coulomb_limit` |
| Full regression suite | passed | 19 tests passed in 23.95 s with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src pytest -q` |

Important note: v0.5 provides a conservative no-penetration acceptance gate, not an exact analytic global TOI oracle for arbitrary curved surfaces. Uncertain space-time intervals block the step.
