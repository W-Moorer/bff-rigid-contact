# CMAME Self-Review

This audit follows the manuscript-review and polishing checklists used during the revision.

| requirement | evidence in current draft | status |
|---|---|---|
| Claims are bounded to current evidence | Scope subsection states that results validate a smooth rigid-friction contact-geometry module, not a complete finite element contact solver. | pass |
| Abstract and keywords | Abstract audit reports 245 words and the keyword list contains 7 items. | pass |
| CALG naming is consistent | Main text defines Contact-Aligned Local Graphs and uses `\calg{}` thereafter; spelling is consistent in the draft. | pass |
| Dynamic baselines are included | Four engineering scenes compare CALG, PN-triangle BVH, and Linear-triangle BVH under identical rigid-friction parameters. | pass |
| Chrono validation is separated from contact-geometry comparison | Chrono validation is described as trajectory-level agreement because regularizations differ. | pass |
| Each model has an independent schematic | Four separate model schematic figures are included in the results section. | pass |
| Each model has independent contact-geometry curves | Four separate contact-geometry comparison curve figures are included in the results section. | pass |
| Figure typography and artwork package | Generated SVG files retain editable text with Times-family font declarations; `paper/CMAME_ARTWORK_PACKAGE.md` maps each figure to formats, panel labels, source data, and the no-generative-image-model statement. | pass |
| PDF visual order | Rendered PDF pages show the dynamic figures before the Chrono section and no result figures drifting into declarations. | pass |
| Reference coverage | Bibliography expanded from 14 to 31 entries; all 31 DOI strings resolve through the Crossref Works API. | pass |
| Reference mapping | Current citation audit reports 31 in-text keys and 31 bibliography items with no missing or uncited items. | pass |
| Submission declarations | Funding, CRediT, competing interests, data availability, and AI-use statements are present. | pass |
| Data deposit package | `paper/CMAME_DATA_DEPOSIT_PACKAGE.md`, `paper/CMAME_DATA_README.md`, `paper/CMAME_DATA_MANIFEST.csv`, `.zenodo.json`, and `CITATION.cff` define the files, variables, units, checksums, metadata, and citation route for a DOI-bearing repository deposit. The manifest separates core results from full-frame VTK animation files. | pass |
| Submission checklist | `paper/CMAME_SUBMISSION_CHECKLIST.md` records the final author/DOI replacement steps and verification commands. | pass |
| Completion audit | `paper/CMAME_COMPLETION_AUDIT.md` maps each active objective requirement to current evidence and identifies only author/DOI-dependent open items. | pass |
| Submission metadata and cover letter | `paper/CMAME_SUBMISSION_METADATA.md` and `paper/CMAME_COVER_LETTER_DRAFT.md` provide ready-to-paste submission text while preserving author-supplied fields for identities and correspondence. | pass |
| Machine-readable submission gate | `scripts/check_cmame_submission_package.py` checks CMAME limits, metadata consistency, citation mapping, figure formats, font markers, required package files, and tracked placeholder tokens. | pass |
| Author metadata template | `paper/CMAME_AUTHOR_METADATA_TEMPLATE.md` records the exact author, affiliation, ORCID, corresponding author, and CRediT fields that still require author input. | pass |
| LaTeX build | `pdflatex` builds a 27-page PDF; the log has no undefined references or citations and only one bibliography underfull warning. | pass |
| Regression tests | With `PYTHONPATH=src`, `python -m pytest -q` reports 30 passed tests. | pass |
| Remaining pre-submission dependency | An immutable data DOI still needs to be minted by repository deposition before journal submission. | open |
| Author metadata | The manuscript remains anonymized because final author names, affiliations, corresponding author details, ORCID identifiers, and individual CRediT allocation have not been provided. | open |
