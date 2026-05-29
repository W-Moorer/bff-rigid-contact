# BFF integration notes

The uploaded official BFF code is useful as a reference implementation and optional local patch conditioner, but it is not part of the core CALG solver.

BFF's algorithmic workflow is boundary-first flattening of disk-topology meshes: construct compatible boundary data, integrate a boundary curve, and extend the curve over the interior by solving sparse Poisson/Laplace systems.  This is not used as a global contact representation here because arbitrary contact surfaces may be closed, open, multiply connected, or topologically complex.

Use BFF only through `calg.solver.bff_adapter.BFFAdapter`.  The adapter is skip-safe; if no BFF executable is found, validation continues and records that BFF was unavailable.

Expected location if the official package is copied into this repository:

```text
third_party/bff_official/
```

Typical Windows executable from the uploaded package:

```text
third_party/bff_official/binaries/windows-v1.6/bff-command-line.exe
```

The core solver is topology-independent and uses contact-aligned local graph coordinates generated from each candidate contact pair.
