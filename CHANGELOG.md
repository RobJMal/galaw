# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] - 2026-09-21

### Breaking Changes

- **Output-buffer interface** — `compute_fk`, `compute_link_jacobians`, `compute_link_jacobian`, and `compute_ik` no longer return allocated values. They now write into a caller-owned `GalawData<T>` created once via `model.create_galaw_data()`. Generated code uses `GeneratedGalawData<T, N, M>` in the same way.
- `galaw::parser` and `galaw::utils` modules are now crate-private. Use the re-exported `galaw::load_urdf` instead of `galaw::parser::load_urdf`.
- `GalawModel::ancestors_by_link` field is now crate-private (internal cache, not part of the public API).
- `KinematicsError::OutLengthMismatch` removed — it was never constructed.

### Added

- **Generic float support** — all APIs are now generic over `T: RealField + Copy`, accepting both `f32` and `f64`. Type-parity tests verify IK convergence across both types.
- **`GalawModel::num_links`** — convenience field alongside `num_actuated_joints`; use instead of `model.links.len()`.
- **Parallelism examples** — `examples/parallel_fk_runtime.rs` and `examples/parallel_fk_generated.rs` show rayon-based batch FK with per-thread buffer reuse.

### Performance

- **Jacobian ~2.9× faster (runtime), ~2.3× faster (generated)** — ancestor lists are now precomputed at model load time rather than walked on every call. Jacobian is now ~22.6× faster than [`k`](https://crates.io/crates/k) (was ~7.7×) and generated is ~38.2× faster (was ~16.7×).
- **Zero-allocation hot path** — FK, Jacobian, and IK no longer heap-allocate per call; all output is written into pre-allocated caller-owned buffers.
- **Generated Jacobian** — fixed redundant computation in the emitted code.

## [0.2.0] - 2026-09-07

### Added

- **Jacobian computation** — `GalawModel::compute_link_jacobians` returns a 6×N spatial Jacobian for every link; `compute_link_jacobian` computes it for a single target link.
- **Inverse kinematics** — `GalawModel::compute_ik` solves IK using a damped Levenberg-Marquardt solver, with post-convergence clamping to joint limits.
- **Generated Jacobian** — `codegen_kinematics` now emits `compute_link_jacobians` alongside `compute_fk`; returns a fixed-size array of `SMatrix<f64, 6, N>`, one per link, with no `Result` or parsing on the hot path.
- **Generated IK** — `codegen_kinematics` emits per-link `compute_ik` closures; same solver parameters and clamping behavior as the runtime version.
- Benchmark suites for Jacobian (`jacobian_speed`) and IK (`ik_speed`).
- Correctness tests for Jacobian (finite-difference) and IK (FK round-trip).

## [0.1.0] - 2026-07-28

### Added

- URDF parsing via `load_urdf` — extracts kinematic structure (links, joints, axes, limits) and validates tree topology (root detection, cycle detection, connectivity).
- **Forward kinematics** — `GalawModel::compute_fk` returns poses for all links given joint commands.
- **Generated FK** — `codegen_kinematics` binary emits an ahead-of-time `compute_fk` per robot; takes and returns fixed-size arrays, no parsing or `Result` on the hot path.
- `GalawModel::get_link_idx` / `get_joint_idx` for name-based lookup.
- Descriptive error types: `GalawError`, `UrdfParseError`, `ModelTopologyError`, `KinematicsError`.
- Benchmarks against the [`k`](https://crates.io/crates/k) crate; runtime is ~3.3× faster and generated is ~8.9× faster (geometric mean across four robots).
- Bundled URDF descriptions for Flexiv Enlight-L, ANYbotics ANYmal-D, Hello Robot Stretch 4, and Wuji Hand v1.
