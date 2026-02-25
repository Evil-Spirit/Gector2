# GectorPlan — Gector Kernel Development Roadmap

> **Gector Kernel** (Geometric Vector Kernel) is a production-quality, open-source geometric modeling kernel written in modern C++ with CMake, carrying no external library dependencies. Its goal is to surpass Parasolid and OpenCASCADE in precision, performance, and developer experience.

---

## Guiding Principles

- **No external dependencies** — pure C++17/20 and the standard library only.
- **Test-first** — every iteration ships with its own unit/integration tests.
- **Incremental shippability** — each iteration compiles, passes all tests, and can be released independently.
- **Numerical robustness** — interval arithmetic, adaptive tolerances, and exact predicates are first-class citizens.
- **Clean API** — immutable value types, handle/body separation (as in Parasolid), and strict ABI stability within a major version.

---

## Chapter 1 — Foundation Infrastructure

*Goal: establish the project skeleton, math primitives, and quality gates that every later chapter depends on.*

### Iteration 1.1 — Repository & Build System
- CMake 3.21+ project with `FetchContent`-free, self-contained layout.
- Directory structure: `kernel/`, `tests/`, `docs/`, `tools/`.
- CTest integration and CI pipeline (GitHub Actions) running build + tests on Linux, macOS, Windows.
- Coding-style enforcement (`.clang-format`, `.clang-tidy` baseline).
- **Deliverable:** `cmake --build . && ctest` passes on all three platforms with a hello-world smoke test.

### Iteration 1.2 — Core Math Types
- `Vec2`, `Vec3`, `Vec4` — fixed-size SIMD-friendly column vectors.
- `Mat2x2`, `Mat3x3`, `Mat4x4` — row-major matrices, compile-time dimensions.
- `Quaternion` — unit quaternion for rotation.
- `Interval` — closed real interval `[lo, hi]` for error bounding.
- All types: constexpr constructors, `operator==` with tolerance, serialization to/from JSON-like text.
- **Deliverable:** 100 % unit-test coverage for all math types, zero dynamic allocations in hot paths.

### Iteration 1.3 — Tolerance & Precision Model
- Global `Tolerances` singleton: `linear`, `angular`, `parametric` tolerances with per-session override.
- `FuzzyCompare` utility: ULP-based, absolute, and relative comparison strategies selectable at call-site.
- **Deliverable:** tolerance tests covering degenerate and near-degenerate cases.

### Iteration 1.4 — Memory & Handle Model
- `Handle<T>` — intrusive reference-counted smart pointer (Parasolid-style entity lifetime).
- `Arena` allocator for transient geometry objects (scratch computation).
- `EntityId` — opaque 64-bit stable identifier, never reused within a session.
- **Deliverable:** memory-leak tests via Address Sanitizer in CI.

### Iteration 1.5 — Error Handling & Diagnostics
- `Result<T, Error>` monad — no exceptions on the hot path.
- Structured error codes, human-readable messages, stack-capture in debug builds.
- Logging facade (`GK_LOG_*` macros) with pluggable back-end.
- **Deliverable:** all prior iterations converted to `Result`-returning APIs.

---

## Chapter 2 — Parametric Curves

*Goal: a complete, robust library of analytic and NURBS curves in 2-D and 3-D.*

### Iteration 2.1 — Curve Abstract Interface
- `ICurve3` interface: `evaluate(t)` → point + 1st/2nd derivatives, `domain()`, `isClosed()`, `approximateLength()`.
- `ICurve2` — identical interface in the plane.
- **Deliverable:** interface headers + mock test curve.

### Iteration 2.2 — Analytic Primitives
- `Line3`, `Line2` — infinite and bounded.
- `Circle3`, `Circle2`.
- `Ellipse3`, `Ellipse2`.
- `Hyperbola3`, `Parabola3`.
- Exact parameter-to-point and point-to-parameter (closest-point) for each.
- **Deliverable:** geometric property tests (length, tangent angle, curvature) to tolerance 1e-10.

### Iteration 2.3 — B-Spline & NURBS Curves
- `BSplineCurve3`, `BSplineCurve2` — arbitrary degree, open/clamped/periodic knot vectors.
- De Boor evaluation, knot insertion, degree elevation, knot removal.
- `NURBSCurve3` — rational variant.
- **Deliverable:** round-trip tests (construct → evaluate → reconstruct) and IGES/STEP-compatible data model.

### Iteration 2.4 — Curve Utilities
- Adaptive arc-length parameterization.
- Curve–curve intersection (analytic + iterative Newton refinement).
- Closest point on curve to an external point (Newton–Raphson + subdivision fallback).
- **Deliverable:** intersection accuracy tests for all analytic pairs; NURBS self-intersection detection.

---

## Chapter 3 — Parametric Surfaces

*Goal: a complete surface library mirroring Chapter 2.*

### Iteration 3.1 — Surface Abstract Interface
- `ISurface` interface: `evaluate(u,v)` → point + partial derivatives up to 2nd order, `domain()`, `normalAt(u,v)`.
- **Deliverable:** interface headers + mock test surface.

### Iteration 3.2 — Analytic Primitives
- `Plane`, `Sphere`, `Cylinder`, `Cone`, `Torus`.
- Exact principal curvatures and Gaussian curvature for each.
- Analytic UV → XYZ and XYZ → UV (inverse parameterization).
- **Deliverable:** curvature tests; UV round-trip to 1e-10.

### Iteration 3.3 — B-Spline & NURBS Surfaces
- `BSplineSurface`, `NURBSSurface` — tensor-product, arbitrary bi-degree.
- De Boor on surfaces, knot insertion (u and v directions), degree elevation.
- **Deliverable:** round-trip tests; comparison against known analytic surfaces at iso-parameter curves.

### Iteration 3.4 — Surface Utilities
- Adaptive surface tessellation (Chaikin/subdivision + curvature-aware).
- Surface–surface intersection curves (marching + Newton correction).
- Closest point on surface to external point.
- **Deliverable:** intersection tests vs. known analytic results; tessellation quality metrics.

---

## Chapter 4 — Topological Data Model

*Goal: a robust boundary-representation (B-Rep) data structure.*

### Iteration 4.1 — Topology Entities
- `Vertex`, `Edge`, `Wire`, `Face`, `Shell`, `Lump`, `Body` — mirror Parasolid's entity hierarchy.
- Forward/backward topological traversal iterators.
- **Deliverable:** hand-constructed B-Rep of a cube traversable without errors.

### Iteration 4.2 — Geometric Binding
- Attach `ICurve3` to `Edge`; attach `ISurface` to `Face`.
- `CoEdge` with orientation and `PCurve` (2-D UV curve on surface).
- `BRepValidator` — checks all topological/geometric consistency rules.
- **Deliverable:** validator rejects all intentionally broken models.

### Iteration 4.3 — Topological Queries
- `BRepQuery`: mass properties (area, volume, centroid, moments of inertia) via adaptive Gaussian quadrature.
- Entity classification: `PointInBody`, `PointOnFace`, `PointOnEdge`.
- Topological distance (number of shared edges between two faces).
- **Deliverable:** mass-property accuracy tests vs. analytically known primitives (sphere, box, cylinder).

### Iteration 4.4 — B-Rep Persistence
- Binary serialization format (`.gkb`) — versioned, backward-compatible.
- ASCII text round-trip format (`.gkt`) for debugging.
- **Deliverable:** round-trip tests for all primitive shapes.

---

## Chapter 5 — Primitive Solid Builders

*Goal: convenient high-level builders for standard shapes.*

### Iteration 5.1 — Box, Sphere, Cylinder, Cone
- `MakeBox`, `MakeSphere`, `MakeCylinder`, `MakeCone` — return fully validated `Body`.
- Exact geometry (analytic surfaces), correct orientation (outward normals).
- **Deliverable:** builders + mass-property acceptance tests.

### Iteration 5.2 — Torus, Wedge, Prism, Pyramid
- `MakeTorus`, `MakeWedge`, `MakePrism`, `MakePyramid`.
- **Deliverable:** builders + mass-property acceptance tests.

### Iteration 5.3 — Extrusion & Revolution
- `MakeExtrusion` — sweeps a wire along a direction.
- `MakeRevolution` — revolves a wire about an axis.
- **Deliverable:** sweep of a circle = cylinder; revolution of a line = cone — verified analytically.

### Iteration 5.4 — Loft & Pipe Sweep
- `MakeLoft` — multi-section loft with optional guide rails.
- `MakePipe` — sweeps a profile along a spine curve.
- **Deliverable:** C0 / G1 continuity checks along resulting faces.

---

## Chapter 6 — Boolean Operations

*Goal: robust, non-manifold-aware Boolean set operations.*

### Iteration 6.1 — Intersection Engine
- `IntersectFaceFace` — compute intersection curves between two faces.
- Robust handling of tangent faces, degenerate edges, and coincident geometry.
- **Deliverable:** intersection tests for all analytic surface pairs.

### Iteration 6.2 — Boolean Union
- `BooleanUnion(A, B)` — merge two bodies.
- Correct topology for coincident / tangent faces.
- **Deliverable:** union of two overlapping boxes = single convex body; mass = sum − overlap.

### Iteration 6.3 — Boolean Difference
- `BooleanDifference(A, B)` — subtract B from A.
- **Deliverable:** box minus sphere = box with spherical cavity; mass verified.

### Iteration 6.4 — Boolean Intersection
- `BooleanIntersection(A, B)` — keep common volume.
- **Deliverable:** intersection of two half-spaces = box; mass verified.

### Iteration 6.5 — Non-Manifold & Edge Cases
- Zero-volume result detection, open-shell results.
- Self-intersecting input detection with diagnostic messages.
- **Deliverable:** stress test suite of 500 randomly generated Boolean operations.

---

## Chapter 7 — Local Operations (Features)

*Goal: direct modeling operations on B-Rep bodies.*

### Iteration 7.1 — Chamfer & Fillet
- `ChamferEdge`, `FilletEdge` — constant-radius and variable-radius variants.
- Correct handling of convex, concave, and tangent-chain edges.
- **Deliverable:** fillet round-trip tests; radius accuracy to 1e-6.

### Iteration 7.2 — Draft (Taper)
- `DraftFaces` — taper planar or cylindrical faces by a given angle about a parting plane.
- **Deliverable:** draft angle accuracy tests.

### Iteration 7.3 — Shell (Hollow)
- `ShellBody` — offset all faces inward/outward to create a thin-walled shell.
- **Deliverable:** wall-thickness accuracy; volume = original − hollowed.

### Iteration 7.4 — Offset Face & Offset Body
- `OffsetFace` — move a single face along its normal.
- `OffsetBody` — uniform / per-face offset of an entire body.
- **Deliverable:** offset of sphere = concentric sphere; radius delta verified.

### Iteration 7.5 — Pattern & Mirror
- `LinearPattern`, `CircularPattern` — multiply bodies or features.
- `MirrorBody` — reflect about a plane.
- **Deliverable:** pattern count and mass-property tests.

---

## Chapter 8 — Meshing & Discretization

*Goal: high-quality surface and volume meshes from B-Rep input.*

### Iteration 8.1 — Surface Triangulation
- Incremental Delaunay triangulation on each face UV domain.
- Curvature-adaptive chord-height tolerance.
- **Deliverable:** watertight mesh for all primitive solids; no T-junctions across edges.

### Iteration 8.2 — Mesh Quality Metrics
- Aspect ratio, minimum angle, Jacobian determinant per triangle.
- `MeshValidator` — rejects meshes below configurable quality thresholds.
- **Deliverable:** quality metric tests on known bad and good meshes.

### Iteration 8.3 — Polyhedral Mesh (Volume)
- Constrained Delaunay tetrahedralization inside a closed shell.
- Size-field control via surface mesh gradation.
- **Deliverable:** tet mesh volume converges to B-Rep volume as refinement increases.

### Iteration 8.4 — Mesh Export
- STL (ASCII + binary), OBJ, OFF export.
- **Deliverable:** exported files readable by reference viewers; round-trip geometry check.

---

## Chapter 9 — File Format I/O

*Goal: industry-standard neutral format import and export.*

### Iteration 9.1 — STEP AP203/AP214 Export
- Write `Body` entities to ISO 10303-21 STEP files.
- Covers B-Spline and NURBS surfaces, analytic surfaces, edges, vertices.
- **Deliverable:** exported STEP opened in FreeCAD / reference STEP viewer without errors.

### Iteration 9.2 — STEP Import
- Parse STEP AP203/AP214 into Gector B-Rep.
- **Deliverable:** import → export round-trip: geometry deviation < 1e-6.

### Iteration 9.3 — IGES Export & Import
- IGES 5.3 subset: surfaces, curves, B-Rep solids.
- **Deliverable:** round-trip geometry deviation < 1e-5.

### Iteration 9.4 — STL / OBJ Import
- Reconstruct watertight B-Rep shell from triangulated mesh (reverse engineering path).
- **Deliverable:** STL → B-Rep → STL round-trip; Hausdorff distance < chord-height tolerance.

---

## Chapter 10 — Advanced Geometry & Analysis

*Goal: production-quality analysis tools for CAD/CAE workflows.*

### Iteration 10.1 — Continuity Analysis
- G0/G1/G2 continuity check along edges and between faces.
- Curvature-comb visualization data export.
- **Deliverable:** continuity tests for fillet, loft, and pipe outputs.

### Iteration 10.2 — Interference Detection
- Bounding-volume-hierarchy (BVH, AABB tree) over face meshes.
- `DetectInterference(A, B)` — returns intersecting face pairs.
- **Deliverable:** interference detection in under 10 ms for 10 000-face assemblies.

### Iteration 10.3 — Minimum Distance
- `DistanceBodyBody`, `DistanceFaceFace`, `DistanceEdgeEdge`.
- Exact distance for analytic pairs; iterative for NURBS.
- **Deliverable:** distance tests vs. analytically known values.

### Iteration 10.4 — Draft Analysis
- Per-face draft-angle map against a pull direction.
- Color-coded result data for visualization.
- **Deliverable:** draft values match ground truth on synthetic parts.

### Iteration 10.5 — Mass Properties (High Precision)
- Multi-precision (128-bit) quadrature for near-degenerate bodies.
- `MassProperties` struct: volume, surface area, CoG, inertia tensor, principal axes.
- **Deliverable:** relative error < 1e-12 vs. analytic values for all primitives.

---

## Chapter 11 — Assembly & Transformation

*Goal: multi-body assembly management.*

### Iteration 11.1 — Placement & Transformation
- `Transform` — affine 4×4, validated to be rigid (rotation + translation only) or full affine.
- `PlacedBody` — `Body` + `Transform` with lazy evaluation.
- **Deliverable:** transformed mass properties vs. analytic rotation.

### Iteration 11.2 — Assembly Graph
- `Assembly` — directed acyclic graph of `PlacedBody` nodes.
- Flattened iterator over all leaves with world transforms.
- **Deliverable:** assembly traversal tests; circular-reference detection.

### Iteration 11.3 — Assembly-Level Boolean
- Boolean operations applied to assembly instances without modifying source bodies.
- **Deliverable:** assembly subtraction test.

---

## Chapter 12 — Performance, Hardening & Release

*Goal: production readiness — performance, fuzz testing, and API stability.*

### Iteration 12.1 — Benchmarks & Profiling
- Google-Benchmark-style micro-benchmarks (implemented without the library) for all hot paths.
- Establish baseline performance numbers; document in `docs/benchmarks.md`.
- **Deliverable:** benchmark suite committed; no regression vs. baseline.

### Iteration 12.2 — Fuzz Testing
- LibFuzzer-based fuzz harnesses for STEP import, IGES import, B-Rep construction.
- Integrated into CI with a 5-minute corpus run.
- **Deliverable:** zero crashes on seed corpus after 1 hour of fuzzing.

### Iteration 12.3 — Thread Safety & Parallelism
- Audit and document thread-safety guarantees (const methods are safe, mutating requires external lock).
- Optional parallel tessellation using `std::execution::par`.
- **Deliverable:** thread-safety tests using TSan; parallel vs. serial tessellation correctness check.

### Iteration 12.4 — API Stability & Versioning
- Semantic versioning (`MAJOR.MINOR.PATCH`) documented in `CMakeLists.txt`.
- ABI compatibility matrix in `docs/abi.md`.
- Deprecation policy: one full minor version deprecation window.
- **Deliverable:** ABI checker script in CI (symbol list diff).

### Iteration 12.5 — Documentation & Examples
- Doxygen API reference generated in CI and published to GitHub Pages.
- Tutorial series: `examples/01_box`, `examples/02_boolean`, `examples/03_fillet`, `examples/04_step_io`.
- Developer guide: `docs/architecture.md`, `docs/contributing.md`.
- **Deliverable:** Doxygen builds with zero warnings; all example programs build and run.

---

## Milestone Summary

| Milestone | Chapters | Key Capability Unlocked |
|---|---|---|
| **M0 — Foundation** | 1 | Build system, math, handles, errors |
| **M1 — Curves & Surfaces** | 2–3 | Full parametric geometry library |
| **M2 — Solid Modeling** | 4–5 | B-Rep + primitive builders |
| **M3 — Boolean Engine** | 6 | CSG operations |
| **M4 — Feature Modeling** | 7 | Direct modeling features |
| **M5 — Meshing** | 8 | Mesh output for visualization/CAE |
| **M6 — Interoperability** | 9 | STEP / IGES exchange |
| **M7 — Analysis** | 10–11 | CAE-ready analysis & assemblies |
| **M8 — Production** | 12 | Performance, fuzz hardening, stable API |

---

## Definition of Done (per Iteration)

1. All new code passes `clang-tidy` and `clang-format` checks.
2. Unit tests cover ≥ 90 % of new lines (measured by `gcov`/`llvm-cov`).
3. No new Address Sanitizer or Undefined Behaviour Sanitizer errors.
4. `ctest` passes on Linux, macOS, and Windows.
5. Public API headers documented with Doxygen comments.
6. `CHANGELOG.md` entry added.
