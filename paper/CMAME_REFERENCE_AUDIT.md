# CMAME Reference Audit

This audit records the reference state of `paper/calg_cmame_draft.tex` after the
current CMAME manuscript revision.

## Current Status

- Manuscript checked: `paper/calg_cmame_draft.tex`
- Audit date: 2026-06-03
- In-text citation keys: 54
- Bibliography entries: 54
- Missing bibliography entries: 0
- Uncited bibliography entries: 0
- DOI entries checked through the Crossref Works API: 54
- Crossref DOI failures: 0

The citation mapping is internally consistent: every cited key has a matching
`\bibitem`, every `\bibitem` is cited in the manuscript, and every DOI string in
the bibliography resolved through Crossref during the audit.

## Verified Bibliography Entries

| key | DOI | scope |
|---|---|---|
| johnson1985 | 10.1017/CBO9781139171731 | foundational contact mechanics |
| kikuchi1988 | 10.1137/1.9781611970845 | variational and finite-element contact |
| laursen2002 | 10.1007/978-3-662-04864-1 | computational contact and impact mechanics |
| wriggers2006 | 10.1007/978-3-540-32609-0 | computational contact mechanics |
| sauer2012 | 10.1016/j.cma.2012.09.002 | surface-potential contact |
| lu2011 | 10.1016/j.cma.2010.10.001 | isogeometric contact |
| temizer2011 | 10.1016/j.cma.2010.11.020 | NURBS-based contact treatment |
| delorenzis2011 | 10.1002/nme.3159 | frictional isogeometric contact |
| li2020ipc | 10.1145/3386569.3392425 | incremental potential contact |
| li2021codimipc | 10.1145/3450626.3459767 | codimensional IPC |
| lan2021medialipc | 10.1145/3450626.3459753 | medial IPC |
| ferguson2021rigidipc | 10.1145/3450626.3459802 | rigid-body IPC |
| fang2024aipc | 10.1109/TVCG.2023.3295656 | augmented IPC |
| huang2024gipc | 10.1145/3643028 | Gauss-Newton IPC |
| stiffgipc2025 | 10.1145/3735126 | GPU IPC for stiff systems |
| huang2025gcp | 10.1145/3731142 | geometric contact potential |
| du2024smooth | 10.1111/cgf.15187 | smooth-surface contact |
| ferguson2023highorder | 10.1145/3588432.3591488 | high-order IPC |
| crespel2024fibres | 10.1145/3658191 | high-order curved-fibre contact |
| chen2024tdib | 10.1145/3687960 | parametric-surface CCD |
| liu2024generalsdf | 10.1016/j.cagd.2024.102305 | general SDF collision detection |
| chen2025ogc | 10.1145/3731205 | offset geometric contact |
| pelletier2025trisdf | 10.1145/3747862 | Triangle-SDF CCD |
| stewart1996 | 10.1002/(SICI)1097-0207(19960815)39:15<2673::AID-NME972>3.0.CO;2-I | rigid-body contact time stepping |
| anitescu1997 | 10.1023/A:1008292328909 | complementarity contact formulation |
| jean1999 | 10.1016/S0045-7825(98)00383-1 | nonsmooth contact dynamics |
| acary2008 | 10.1007/978-3-540-75392-6 | nonsmooth dynamical systems |
| gavrea2008 | 10.1137/060675745 | semi-implicit nonsmooth dynamics |
| anitescu2010 | 10.1007/s10589-008-9223-4 | cone-complementarity dynamics |
| tasora2011 | 10.1016/j.cma.2010.06.030 | matrix-free cone complementarity |
| chakraborty2014 | 10.1177/0278364913501210 | geometrically implicit contact |
| halm2024setvalued | 10.1177/02783649241236860 | simultaneous frictional impacts |
| solanillas2024unilateral | 10.1016/j.mechmachtheory.2024.105809 | unilateral rigid-body interactions |
| luo2024symplectic | 10.1016/j.cma.2023.116726 | nonsmooth frictional-contact integration |
| chen2024contactforce | 10.1177/16878132241307004 | compliant contact-force model |
| machado2012 | 10.1016/j.mechmachtheory.2012.02.010 | compliant contact-force models |
| flores2021 | 10.1007/s11044-021-09803-y | dynamical contact review |
| corral2021 | 10.1007/s11071-021-06344-z | nonlinear contact phenomena |
| tasora2016 | 10.1007/978-3-319-40361-8_2 | Project Chrono |
| gilbert1988 | 10.1109/56.2083 | GJK distance query |
| gottschalk1996 | 10.1145/237170.237244 | OBBTree hierarchy |
| mirtich1998 | 10.1145/285857.285860 | V-Clip proximity |
| teschner2005 | 10.1111/j.1467-8659.2005.00829.x | collision-detection survey |
| bridson2002 | 10.1145/566654.566623 | robust cloth collision/contact/friction |
| harmon2008 | 10.1145/1399504.1360622 | simultaneous collision response |
| redon2002 | 10.1111/1467-8659.00587 | rigid-body CCD |
| brochu2012 | 10.1145/2185520.2185592 | geometrically exact CCD |
| pabst2010 | 10.1111/j.1467-8659.2010.01769.x | CPU/GPU collision detection |
| wang2021ccd | 10.1145/3460775 | large-scale CCD benchmark |
| delorenzis2014 | 10.1002/gamm.201410005 | isogeometric contact review |
| osher1988 | 10.1016/0021-9991(88)90002-2 | level-set methods |
| curless1996 | 10.1145/237170.237269 | volumetric reconstruction |
| jones2006 | 10.1109/TVCG.2006.56 | distance-field survey |
| vlachos2001pn | 10.1145/364338.364387 | curved PN triangles |

## Remaining Publication-Time Checks

This DOI audit verifies internal citation consistency and DOI resolvability, not
final publisher style. Before submission, the final author should still check
journal-specific capitalization, article-number formatting, and any data-package
DOI once the repository deposit is finalized.
