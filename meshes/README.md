# Gazebo Meshes

This directory is the repository entry point for geometry exported from the
final SolidWorks model.

Expected layout:

```text
meshes/
├── visual/       # DAE or STL files used only for appearance
├── collision/    # simplified STL collision geometry
└── cad_manifest.yaml
```

Keep the original `.SLDASM` and `.SLDPRT` sources outside this simulation
repository. Before importing geometry, follow
`docs/SOLIDWORKS_MODEL_INTERFACE.md` for units, axes, link names, origins,
mesh complexity, and staged validation.
