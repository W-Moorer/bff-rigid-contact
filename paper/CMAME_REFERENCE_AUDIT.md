# CMAME Reference Audit

This audit records the reference state of `paper/calg_cmame_draft.tex` after the
current CMAME manuscript revision.

## Current Status

- Manuscript checked: `paper/calg_cmame_draft.tex`
- Audit date: 2026-06-01
- In-text citation keys: 42
- Bibliography entries: 42
- Missing bibliography entries: 0
- Uncited bibliography entries: 0
- DOI entries checked through the Crossref Works API: 42
- Crossref DOI failures: 0

The citation mapping is therefore internally consistent: every cited key has a
matching `\bibitem`, every `\bibitem` is cited in the manuscript, and every DOI
string in the bibliography resolved through Crossref during the audit.

## Verified Bibliography Entries

| key | DOI | audit note |
|---|---|---|
| johnson1985 | 10.1017/CBO9781139171731 | foundational contact mechanics book |
| kikuchi1988 | 10.1137/1.9781611970845 | variational contact and finite element contact book |
| laursen2002 | 10.1007/978-3-662-04864-1 | computational contact and impact mechanics book |
| wriggers2006 | 10.1007/978-3-540-32609-0 | computational contact mechanics book |
| sauer2012 | 10.1016/j.cma.2012.09.002 | surface-potential contact formulation |
| li2020ipc | 10.1145/3386569.3392425 | incremental potential contact |
| huang2025gcp | 10.1145/3731142 | geometric contact potential |
| du2024smooth | 10.1111/cgf.15187 | smooth-surface deformable contact |
| ferguson2023highorder | 10.1145/3588432.3591488 | high-order IPC on curved meshes |
| chen2024tdib | 10.1145/3687960 | continuous collision detection between parametric surfaces |
| chen2025ogc | 10.1145/3731205 | offset geometric contact |
| pelletier2025trisdf | 10.1145/3747862 | Triangle-SDF continuous collision detection |
| stewart1996 | 10.1002/(SICI)1097-0207(19960815)39:15<2673::AID-NME972>3.0.CO;2-I | implicit rigid-body contact time stepping |
| anitescu1997 | 10.1023/A:1008292328909 | complementarity formulation for rigid-body frictional contact |
| jean1999 | 10.1016/S0045-7825(98)00383-1 | nonsmooth contact dynamics |
| acary2008 | 10.1007/978-3-540-75392-6 | nonsmooth dynamical systems book |
| gavrea2008 | 10.1137/060675745 | semi-implicit nonsmooth rigid multibody dynamics |
| anitescu2010 | 10.1007/s10589-008-9223-4 | cone-complementarity method for nonsmooth dynamics |
| tasora2011 | 10.1016/j.cma.2010.06.030 | matrix-free cone complementarity for large rigid-body systems |
| chakraborty2014 | 10.1177/0278364913501210 | geometrically implicit multibody contact |
| machado2012 | 10.1016/j.mechmachtheory.2012.02.010 | compliant contact force model review |
| flores2021 | 10.1007/s11044-021-09803-y | contact mechanics for dynamical systems review |
| corral2021 | 10.1007/s11071-021-06344-z | nonlinear contact phenomena review |
| tasora2016 | 10.1007/978-3-319-40361-8_2 | Project Chrono reference |
| gilbert1988 | 10.1109/56.2083 | GJK distance query |
| gottschalk1996 | 10.1145/237170.237244 | OBBTree collision hierarchy |
| mirtich1998 | 10.1145/285857.285860 | V-Clip polyhedral collision detection |
| teschner2005 | 10.1111/j.1467-8659.2005.00829.x | collision detection survey |
| bridson2002 | 10.1145/566654.566623 | robust collision, contact, and friction treatment |
| brochu2012 | 10.1145/2185520.2185592 | geometrically exact continuous collision detection |
| lu2011 | 10.1016/j.cma.2010.10.001 | isogeometric contact analysis |
| temizer2011 | 10.1016/j.cma.2010.11.020 | NURBS-based isogeometric contact treatment |
| delorenzis2011 | 10.1002/nme.3159 | large-deformation frictional NURBS contact |
| delorenzis2014 | 10.1002/gamm.201410005 | isogeometric contact review |
| osher1988 | 10.1016/0021-9991(88)90002-2 | level-set and Hamilton-Jacobi front propagation |
| curless1996 | 10.1145/237170.237269 | volumetric reconstruction from range images |
| jones2006 | 10.1109/TVCG.2006.56 | 3D distance-field survey |
| levy2002 | 10.1145/566654.566590 | least-squares conformal maps |
| sheffer2007 | 10.1561/0600000011 | mesh parameterization survey |
| floater2003 | 10.1016/S0167-8396(03)00002-5 | mean value coordinates |
| sawhney2018bff | 10.1145/3132705 | boundary-first flattening |
| vlachos2001pn | 10.1145/364338.364387 | curved PN triangles |

## Remaining Publication-Time Checks

The current DOI audit verifies resolvability, not final publisher formatting.
Before submission, the final author should still check journal-specific
capitalization and article-number formatting after any bibliography edits, and
add a `[dataset]` citation once the data package receives a DOI.
