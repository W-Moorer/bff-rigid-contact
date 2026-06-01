# CMAME Submission Checklist

This checklist records the remaining steps for converting the current local
draft into a journal-submission package.

## Ready in the current workspace

- Main manuscript source: `calg_cmame_draft.tex`
- Compiled manuscript PDF: `calg_cmame_draft.pdf`
- Highlights file: `calg_cmame_highlights.txt`
- Figures in PNG/PDF/SVG/TIFF: `paper/figures/`
- Artwork package map: `paper/CMAME_ARTWORK_PACKAGE.md`
- Completion audit: `paper/CMAME_COMPLETION_AUDIT.md`
- Submission metadata: `paper/CMAME_SUBMISSION_METADATA.md`
- Cover letter draft: `paper/CMAME_COVER_LETTER_DRAFT.md`
- Reference audit: `paper/CMAME_REFERENCE_AUDIT.md`
- Reproducibility parameter table: `paper/CMAME_REPRODUCIBILITY_PARAMETERS.md`
- Data deposit package description: `paper/CMAME_DATA_DEPOSIT_PACKAGE.md`
- File-level data manifest: `paper/CMAME_DATA_MANIFEST.csv`
- Data manifest summary: `paper/CMAME_DATA_MANIFEST_SUMMARY.md`
- Zenodo metadata draft: `.zenodo.json`
- Citation metadata draft: `CITATION.cff`

## Required before journal submission

1. Replace anonymized authors, affiliations, corresponding author details, and
   CRediT allocations using `paper/CMAME_AUTHOR_METADATA_TEMPLATE.md`.
2. Confirm the corresponding-author details in
   `paper/CMAME_COVER_LETTER_DRAFT.md`.
3. Deposit the release package in a DOI-bearing repository.
4. Replace the Data Availability repository-only wording with the assigned DOI.
5. Add a dataset citation to the reference list if the repository provides a
   DataCite citation.
6. Recompile the manuscript after replacing author and DOI fields.
7. Re-run the final checks:

```powershell
$env:PYTHONPATH='src'; python -m pytest -q
Push-Location paper
pdflatex -interaction=nonstopmode calg_cmame_draft.tex
pdflatex -interaction=nonstopmode calg_cmame_draft.tex
Pop-Location
python scripts/make_cmame_data_manifest.py
python scripts/check_cmame_submission_package.py
```

## Current verification status

- Regression tests pass with `PYTHONPATH=src`.
- The manuscript compiles without undefined references or citations.
- The bibliography contains 31 entries; all DOI strings resolve through the
  Crossref Works API.
- The abstract contains 245 words and the keyword list contains 7 items.
- The highlights file contains 5 items and all items are shorter than 85
  characters.
- SVG figure exports retain Times-family font declarations.
- The artwork package records separate figure files, panel labels, source-data
  mappings, and the no-generative-image-model statement.
- The submission metadata file records ready-to-paste title, abstract,
  keywords, highlights, contribution boundary, data availability summary, and
  AI declaration summary.
- The machine-readable submission package gate is available at
  `scripts/check_cmame_submission_package.py`.
- The rendered PDF has the result figures in the correct section order.
