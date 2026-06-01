# CMAME Cover Letter Draft

Dear Editor,

We submit the manuscript entitled "Contact-Aligned Local Graphs for Smooth Rigid
Frictional Contact on Coarse Surface Meshes" for consideration as a research
article in `Computer Methods in Applied Mechanics and Engineering`.

The manuscript addresses a contact-geometry bottleneck in rigid frictional
simulation on smooth engineering surfaces represented by coarse meshes. Standard
piecewise-linear collision meshes and fixed-tessellation curved proxies can
introduce discontinuous contact normals and tangential frames when a contact
point crosses mesh cells. These geometric discontinuities enter the friction
law directly and can produce nonphysical force spikes, tangential-speed jitter,
and stick--slip chatter.

We propose Contact-Aligned Local Graphs (CALG), a curved-contact geometry module
that constructs a local contact frame for each broad-phase candidate pair. When
a normal-cone graphability certificate is satisfied, the two curved patches are
represented as height graphs over a shared contact plane, reducing regular
curved contact from a coupled four-dimensional closest-point query to a
two-dimensional gap-field query. Cases that fail the certificate are routed to a
generic curved-patch solve or conservative fallback, so the reduced query is
used only when its geometric assumptions are certified.

The manuscript is evidence-driven and deliberately bounded to smooth rigid
frictional contact. Four analytic engineering scenes compare CALG directly with
Linear-triangle BVH and PN-triangle BVH under identical rigid-body, timestep, penalty,
and friction parameters. The results show substantially smaller normal-angle
jumps, friction-force jumps, tangential-speed jitter, and stick--slip switching
for CALG. Three accepted scenes are also validated against Project Chrono SMC at
the trajectory-trend level, and a C++ implementation reports lower whole-case
wall time than Chrono SMC in the implemented validation benchmark. The timing
claim is stated as benchmark-specific because the two contact regularizations
are not identical.

The submission includes independent model schematics, independent result curves
for each engineering scene, graphability/fallback diagnostics, reproducibility
parameters, and an archival data package plan with file-level SHA256 checksums.
All scientific figures are generated from analytic model definitions and locked
CSV/JSON result artifacts; no generative image model was used to create the
figures.

The authors confirm that the work is original, is not under consideration
elsewhere, and has been approved by all authors. The authors declare no known
competing financial interests or personal relationships that could have
appeared to influence the work reported in this paper.

Before submission, replace this sentence with the corresponding-author name,
affiliation, postal address, and email address.

Sincerely,

Author-supplied corresponding author
