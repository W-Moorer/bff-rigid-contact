# CALG Contact Validation Package

This repository contains the validation package for **Contact-Aligned Local
Graphs (CALG)**, a contact-geometry backend for smooth rigid-body frictional
contact on coarse surface representations.

The current evidence is intentionally scoped to rigid frictional contact. The
manuscript and experiments focus on:

- analytic inclined-plane friction controls;
- dynamic comparisons between Linear-triangle BVH, PN-triangle BVH, and CALG;
- selected engineering rigid-contact scenes validated against Chrono and
  RecurDyn trends;
- graphability/fallback diagnostics;
- a C++ CALG backend with timing comparisons.

The formulation also points toward surface-patch contact energies, but the
executable results in this repository should be read as a rigid-contact
geometry-backend study rather than a complete deformable finite element contact
solver.

## Core Idea

CALG separates the contact geometry backend from the rigid-body response model.
For each candidate pair produced by an inflated three-dimensional broad phase,
CALG builds a local contact-aligned frame. When the local normal cones certify
that the two patches are graphs over a common contact plane, the narrow phase
reduces the query to a low-dimensional graph-gap problem. When the certificate
fails, the implementation falls back to a more conservative curved or linear
patch-pair query.

The current rigid validation backend couples the CALG detector with dense
tricubic signed-distance refinement. CALG supplies candidate pairs, local graph
certificates, contact samples, and fallback decisions; the tricubic signed field
refines the response gap and normal used by the regularized Coulomb friction
model.

## Quick Start

```bash
python -m pip install -e .[plots,test,baseline]
pytest -q
python scripts/run_validation.py --out-dir results/validation
```

After installation, the basic validation runner is also available as:

```bash
calg-run-validation --out-dir results/validation
```

## Manuscript Package

The CMAME draft is stored under `paper/`:

```text
paper/calg_cmame_draft.tex
paper/calg_cmame_draft.pdf
paper/calg_cmame_highlights.txt
paper/CMAME_*.md
paper/figures/
```

Compile the manuscript with:

```bash
cd paper
latexmk -pdf -interaction=nonstopmode -halt-on-error calg_cmame_draft.tex
```

Generated LaTeX intermediate files are ignored by Git. Submission-grade TIFF
exports are also kept local by default; PDF, PNG, SVG, and source CSV artifacts
are the preferred tracked figure assets.

## Reproducing Main Results

The current manuscript is generated from locked and curated artifacts. The most
relevant entry points are:

```bash
python scripts/run_analytic_friction_controls.py
python scripts/run_rigid_friction_dynamics.py
python scripts/run_engineering_rigid_cases.py
python scripts/summarize_dynamic_backend_comparison.py
python scripts/check_chrono_validation_locks.py
python scripts/summarize_chrono_scene_backend_comparison.py
python scripts/summarize_graphability_fallback_diagnostics.py
python scripts/make_paper_story_figures.py
python scripts/check_cmame_submission_package.py
```

The principal result directories are:

```text
results/v0_15_analytic_friction_controls/
results/v0_11_dynamic_backend_comparison/
results/v0_12_chrono_scene_backend_comparison/
results/v0_13_graphability_fallback_diagnostics/
results/v0_14_reproducibility/
results/timing/
paper/figures/
```

Some raw solver outputs and exploratory result folders are large and remain
local unless explicitly curated into the paper package.

## C++ Backend

The C++ timing backend is under `cpp/`. A typical build is:

```bash
cmake -S cpp -B cpp/build
cmake --build cpp/build --config Release
```

For WSL/Linux-style builds:

```bash
cmake -S cpp -B cpp/build-wsl -DCMAKE_BUILD_TYPE=Release
cmake --build cpp/build-wsl
```

Build directories are ignored by Git. The manuscript reports the C++ CALG
backend timing rather than Python prototype timing.

## External Solver Validation

Chrono and RecurDyn are used as external references for selected engineering
rigid-contact scenes. These solvers are not required for the basic Python tests.
The relevant scripts are:

```text
scripts/run_chrono_guide_slot_validation.py
scripts/run_chrono_ball_joint_pendulum_validation.py
scripts/run_chrono_bearing_raceway_validation.py
scripts/run_recurdyn_step_solid_validation.py
scripts/run_recurdyn_ball_joint_pendulum_validation.py
```

Chrono validation expects a working Chrono installation, typically through the
local WSL environment used during development. RecurDyn validation expects a
local RecurDyn installation with the corresponding automation interface.

## Repository Layout

```text
src/calg/modeling/      mesh containers, analytic geometry generators, OBJ I/O
src/calg/core/          vector math, BVH, triangle distance, curved patches
src/calg/solver/        graphability, contact frames, graph gap, BFF adapter
src/calg/dynamics/      rigid friction response prototypes
src/calg/experiments/   validation cases, metrics, result writers
src/calg/frontend/      CLI entry point
cpp/                    C++ CALG timing backend
scripts/                reproducible validation and figure-generation scripts
docs/                   implementation notes and validation records
paper/                  CMAME manuscript, figures, metadata, and data package
tests/                  unit and regression tests
results/                generated and locked numerical artifacts
```

## Git and Data Policy

The repository tracks curated source, manuscript, figure, and result artifacts.
Local scratch folders, LaTeX intermediates, build products, duplicated manuscript
experiments, and large TIFF exports are ignored. When adding new results, prefer
small source CSV/JSON files plus PDF/PNG/SVG figures, and only promote large raw
solver output after it has been explicitly selected for the manuscript package.
