# CMAME Data Manifest Summary

Total files: 16202
Total size: 5.356 GiB

| tier | files | size GiB |
|---|---:|---:|
| core_result | 148 | 0.195 |
| full_frame_vtk_animation | 15475 | 3.422 |
| high_resolution_figure | 65 | 1.600 |
| manuscript_and_metadata | 20 | 0.008 |
| manuscript_figure | 340 | 0.129 |
| reproduction_code | 154 | 0.001 |

The `full_frame_vtk_animation` tier contains high-time-resolution VTK
animation frames for visual inspection in ParaView. The manuscript tables
and plotted curves are reproducible from the `core_result`,
`reproduction_code`, `manuscript_figure`, and `manuscript_and_metadata`
tiers without downloading the full-frame animation tier.

The complete file-level manifest with SHA256 checksums is stored in
`paper/CMAME_DATA_MANIFEST.csv`.
