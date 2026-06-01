# CMAME Submission Metadata

This file collects ready-to-paste manuscript metadata for the CMAME submission
system. Author names, affiliations, ORCID identifiers, corresponding-author
details, funding, and the dataset DOI still require author confirmation.

## Article Type

Research article.

## Title

Contact-Aligned Local Graphs for Smooth Rigid Frictional Contact on Coarse Surface Meshes

## Short Title

Contact-Aligned Local Graphs for Rigid Frictional Contact

## Abstract

Frictional contact on smooth engineering surfaces is often evaluated through
coarse piecewise-linear collision meshes or dense refined proxies. These
representations are inexpensive, but they tie the gap, normal, and tangential
friction frame to mesh facets; during sliding or rolling, normal jumps can
generate nonphysical force spikes and stick--slip chatter. We propose
Contact-Aligned Local Graphs (CALG), a curved-contact geometry module for coarse
smooth surface meshes. For each broad-phase candidate pair, CALG builds a local
contact frame and, when a normal-cone graphability certificate is satisfied,
represents the curved patches as height graphs over a common contact plane. This
reduces regular curved contact from a coupled four-dimensional closest-point
problem to a two-dimensional gap-field query, while uncertified cases use a
generic curved patch solve. The resulting contact samples are coupled to
regularized Coulomb rigid-body response and compared with Linear-triangle BVH and
PN-triangle BVH. In four analytic engineering examples, Linear-triangle BVH produces
71.9--2004 times larger contact-normal angle jumps and 37.7--108 times larger
friction-force jumps than CALG; PN-triangle BVH produces 40.2--751 times larger
normal-angle jumps and 35.8--56.9 times larger friction-force jumps. Three
Project Chrono scenes show matching motion trends. In the implemented validation
benchmark, CALG C++ reports lower solver-loop time than Chrono SMC in all three
accepted validation scenes. These
results support CALG as a geometry module for smooth rigid frictional contact
where normal and tangential-frame continuity control the physical response.

## Keywords

- Frictional contact
- Rigid-body dynamics
- Local graph representation
- Contact detection
- Curved triangle patches
- Coarse surface meshes
- Computational contact mechanics

## Highlights

- CALG reduces smooth rigid contact to certified local graph queries.
- Coarse Linear-triangle BVH causes normal jumps and friction-force spikes.
- PN-triangle BVH improves smoothness but remains tessellation dependent.
- Chrono validation confirms trajectory trends in three rigid scenes.
- C++ timing is reported only for the implemented validation benchmark.

## Suggested Classifications

- Computational contact mechanics
- Rigid-body dynamics
- Frictional contact
- Collision/contact detection
- Curved or high-order surface geometry

## Core Contribution Statement

The paper introduces Contact-Aligned Local Graphs (CALG), a contact-geometry
module that constructs a local contact plane for each candidate pair and, when
certified by a normal-cone graphability condition, reduces curved smooth patch
contact from a four-dimensional closest-point query to a two-dimensional
gap-field query. The evidence supports CALG as a rigid frictional
contact-geometry module that improves normal, tangent-frame, and friction-force
smoothness on coarse smooth surface meshes.

## Evidence Boundary

The manuscript does not claim to be a complete production finite element contact
solver. The current executable evidence covers static curved-contact accuracy,
rigid frictional dynamics with Linear-triangle BVH and PN-triangle BVH baselines, Project
Chrono SMC trend validation, graphability/fallback diagnostics, and C++ whole
case timing for the accepted validation scenes.

## Data Availability Summary

The code, input definitions, scripts, generated figures, and numerical result
artifacts are prepared for deposition in a DOI-bearing repository. The local
package includes a dataset README, source-result mappings, repository metadata,
and a SHA256 file manifest. The final submission should replace the current
repository-only wording with the assigned archival dataset DOI.

## Generative AI Declaration Summary

OpenAI Codex was used to assist with manuscript restructuring, wording,
consistency checks, and preparation of reproducibility and plotting scripts. No generative image model was used to create the scientific figures. The authors
must review and approve the final content.
