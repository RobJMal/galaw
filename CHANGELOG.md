# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
