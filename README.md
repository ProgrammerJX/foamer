# myfoam

A from-scratch reimplementation of the core OpenFOAM mesh-generation pipeline
in modern C++17. Goal: understand and replicate `blockMesh` and
`snappyHexMesh` end-to-end.

## Status

- [x] **Stage 1** — Foundation: `Vector`, `Face`, `Cell`, CMake skeleton, tests
- [ ] **Stage 2** — `Block` primitive: single-hex local mesh generation
- [ ] **Stage 3** — `BlockMesh`: multi-block assembly, vertex/patch merging
- [ ] **Stage 4** — `PolyMesh` data structure + invariants
- [ ] **Stage 5** — Writers (VTK + OpenFOAM `polyMesh` format)
- [ ] **Stage 6** — Grading, curved edges, `blockMeshDict` parser
- [ ] **Stage 7** — `snappyHexMesh` (castellation → snap → layer addition)

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Layout (mirrors OpenFOAM)

```
src/OpenFOAM/                core library (primitives + polyMesh)
src/mesh/blockMesh/          blockMesh library
src/mesh/snappyHexMesh/      (placeholder)
applications/utilities/mesh/ command-line tools (blockMesh, ...)
tutorials/                   example cases
tests/                       unit tests
```
