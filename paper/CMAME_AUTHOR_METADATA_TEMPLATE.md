# CMAME Author Metadata Template

The manuscript is currently anonymized. Replace the anonymized author block and
CRediT statement only after the final author order and institutional details are
confirmed.

## Required front-matter fields

| Field | Value to provide |
|---|---|
| Author 1 full name |  |
| Author 1 affiliation |  |
| Author 1 ORCID |  |
| Author 1 email |  |
| Author 2 full name |  |
| Author 2 affiliation |  |
| Author 2 ORCID |  |
| Author 2 email |  |
| Corresponding author |  |
| Corresponding author email |  |
| Permanent address, if different |  |

Add or remove rows as needed for the final author list.

## CRediT role allocation

Use named authors rather than `Anonymous Author(s)`.

| CRediT role | Author(s) |
|---|---|
| Conceptualization |  |
| Methodology |  |
| Software |  |
| Validation |  |
| Formal analysis |  |
| Investigation |  |
| Data curation |  |
| Writing - original draft |  |
| Writing - review & editing |  |
| Visualization |  |
| Supervision |  |
| Project administration |  |
| Funding acquisition |  |

## LaTeX replacement pattern

Replace the anonymized block in `calg_cmame_draft.tex`:

```latex
\author[inst1]{Anonymous Author(s)}
\address[inst1]{Manuscript prepared for submission to Computer Methods in Applied Mechanics and Engineering}
```

with the final author and affiliation block required by `elsarticle`, for
example:

```latex
\author[inst1]{First Author}
\ead{first.author@example.edu}
\author[inst1,inst2]{Second Author\corref{cor1}}
\ead{second.author@example.edu}
\cortext[cor1]{Corresponding author}
\address[inst1]{Department, Institution, City, Country}
\address[inst2]{Department, Institution, City, Country}
```

Do not submit the current anonymized CRediT statement unless the journal
explicitly requests an anonymized review version and a separate title page.

