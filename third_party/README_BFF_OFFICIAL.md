# Official BFF code location

The official BFF package supplied by the user is intentionally not copied into this prototype package by default because it contains platform binaries and external dependencies.

To enable optional BFF local-conditioning tests, unpack the provided `bff_official.zip` into this directory so that the layout becomes:

```text
third_party/bff_official/apps/...
third_party/bff_official/include/...
third_party/bff_official/src/...
third_party/bff_official/binaries/windows-v1.6/bff-command-line.exe
```

The adapter is implemented in:

```text
src/calg/solver/bff_adapter.py
```

Core CALG validation does not require this directory.
