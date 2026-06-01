# CMAME Completion Audit

This audit checks the current workspace against the active manuscript objective
and the current CMAME/Elsevier submission constraints inspected on 2026-05-30.

Primary policy sources:

- CMAME Guide for Authors:
  `https://www.sciencedirect.com/journal/computer-methods-in-applied-mechanics-and-engineering/publish/guide-for-authors`
- Elsevier Generative AI policies for journals:
  `https://www.elsevier.com/about/policies-and-standards/generative-ai-policies-for-journals`
- Elsevier Artwork and media instructions:
  `https://www.elsevier.com/about/policies-and-standards/author/artwork-and-media-instructions`

## Objective-Level Audit

| Requirement | Evidence inspected | Status |
|---|---|---|
| References verified through skills and expanded beyond the earlier sparse list | `calg_cmame_draft.tex` contains 31 bibliography items; in-text citations and bibliography map one-to-one; `paper/CMAME_REFERENCE_AUDIT.md` records Crossref DOI checks; the current Crossref API rerun resolved all 31 DOI entries. | pass |
| Full draft manuscript completed | `calg_cmame_draft.tex` contains abstract, introduction/related work, method, numerical results, conclusion, declarations, and references; `pdflatex` builds `calg_cmame_draft.pdf` as a 27-page preview without undefined references or citations. | pass |
| Independent model schematics | Four separate model figures are included: bearing raceway, cam--roller follower, wavy guide slot, and spherical socket. | pass |
| Independent result-comparison curves for each model | Four separate dynamic contact-geometry comparison figures are included for the four engineering scenes, each using the locked time-series artifacts. | pass |
| Figure text uses Times-family English typography | `scripts/make_paper_story_figures.py` sets Times-family serif fonts; all 12 SVG figure stems contain font-family/Times markers; each figure stem has PNG, PDF, SVG, and TIFF exports. | pass |
| Language polishing and logic review | `paper/CMAME_SELF_REVIEW.md` records claim-boundary, terminology, reference, declaration, figure, and reproducibility checks; automated scan finds no legacy misspelling of CALG or tracked placeholder tokens in manuscript metadata files. | pass |
| Method name unified | The manuscript defines Contact-Aligned Local Graphs and uses uppercase CALG thereafter; the legacy transposed acronym spelling is absent. | pass |
| Do not overclaim beyond current experiments | Scope, results, and conclusion state that the evidence supports a rigid frictional contact-geometry module, not a complete deformable finite element solver. | pass |
| Do not add experiments | The draft is built around locked result directories v0.6, v0.9, v0.10, v0.11, v0.12, v0.13, and v0.14; the latest edits only add manuscript, metadata, artwork, and manifest material. | pass |
| CMAME abstract and keywords | Current audit reports 245 abstract words and 7 keywords, satisfying CMAME limits of at most 250 words and 1--7 keywords. | pass |
| Elsevier highlights | `calg_cmame_highlights.txt` contains 5 highlights with lengths 67, 64, 71, 67, and 69 characters, satisfying the 3--5 item and 85-character limits. | pass |
| Data availability package | `paper/CMAME_DATA_README.md`, `paper/CMAME_DATA_DEPOSIT_PACKAGE.md`, `paper/CMAME_DATA_MANIFEST.csv`, `.zenodo.json`, and `CITATION.cff` define the file map, variables, units, checksums, metadata, licence route, and repository deposit path. | needs author action |
| Artwork package | `paper/CMAME_ARTWORK_PACKAGE.md` maps each figure to submission formats, panel labels, dimensions, source data, and the no-generative-image-model statement. | pass |
| Submission metadata and cover letter | `paper/CMAME_SUBMISSION_METADATA.md` collects title, abstract, keywords, highlights, contribution, evidence boundary, data statement, and AI declaration text; `paper/CMAME_COVER_LETTER_DRAFT.md` provides a bounded cover-letter draft. | needs author action |
| Submission declarations | Funding, CRediT, competing interests, data availability, and generative-AI declarations are present in the manuscript. | needs author action |

## Current Verification Snapshot

- Regression tests: `PYTHONPATH=src python -m pytest -q` reports 30 passed tests.
- LaTeX: running `pdflatex -interaction=nonstopmode calg_cmame_draft.tex`
  from the `paper/` directory builds a 27-page PDF and the log contains no
  undefined references or citations.
- Citations: 31 in-text keys and 31 bibliography entries, with no missing or
  uncited entries.
- DOI audit: 31 DOI entries resolved through the Crossref Works API.
- Figure package: 12 figure stems, each with PNG/PDF/SVG/TIFF exports.
- Data manifest: 15,714 files, 4.037 GiB total, separated into core results,
  reproduction code, manuscript metadata, manuscript figures, high-resolution
  figures, and full-frame VTK animations.
- Submission gate: `python scripts/check_cmame_submission_package.py` verifies
  manuscript limits, highlight consistency, citation mapping, figure formats,
  SVG font markers, required metadata files, and tracked placeholder tokens.

## Remaining Conditions Before True Direct Submission

These items cannot be completed from the local repository without author input
or an external repository record:

1. Replace `Anonymous Author(s)` with true author names, affiliations,
   corresponding-author details, ORCID identifiers, and per-author CRediT
   allocations.
2. Confirm whether the current no-specific-funding statement is factually
   correct, or replace it with the actual funding sources and grant numbers.
3. Deposit the release package in a DOI-bearing repository and replace the
   repository-only Data Availability wording with the assigned dataset DOI.
4. Add the formal `[dataset]` citation to the reference list after the repository
   provides its DataCite citation.
5. Replace the corresponding-author sentence in the cover letter draft.
6. Recompile the PDF and regenerate the data manifest after the author and DOI
   replacements.
