# GalawData Design Plan

## Overview

Introduce a `GalawData<T>` workspace type that holds all mutable computation buffers (FK
poses, Jacobians, IK scratch space, etc.). `GalawModel<T>` remains a pure, immutable
description of the robot topology and parameters. All kinematics methods become stateless
functions that take `(&self, joint_cmds, &mut GalawData<T>)`.

This mirrors the Pinocchio `Model` + `Data` split, mapped onto Rust's ownership model.

## Motivation

### 1. Hidden per-call allocations

Every kinematics method currently allocates internally:

```rust
// inside compute_link_jacobians today
let mut links = vec![Isometry3::identity(); self.links.len()];
```

Even with the out-param refactor, the FK scratch buffer is heap-allocated on every call.

### 2. Fragile caller contracts

`compute_link_jacobians` requires a pre-zeroed buffer because it only writes ancestor
columns. Callers who don't understand the topology silently get wrong results. There is no
way to enforce this in the type system with the current design.

### 3. Ergonomics

Callers must know the correct buffer sizes and initialization for each method. This is
error-prone and not beginner-friendly.

## Design

### `GalawData<T>`

```rust
pub struct GalawData<T: RealField> {
    pub fk_poses: Vec<Isometry3<T>>,     // FK scratch + output
    pub jacobians: Vec<Matrix6xX<T>>,    // pre-zeroed, Jacobian output
    // IK scratch space (solver state, delta_q buffer, etc.) to be added
}
```

### Construction

```rust
impl<T: RealField> GalawModel<T> {
    pub fn create_data(&self) -> GalawData<T> { ... }
}
```

`create_data()` guarantees correct sizes and initialization. Callers never size or zero
buffers manually.

### Method signatures

```rust
impl<T: RealField> GalawModel<T> {
    pub fn compute_fk(
        &self,
        joint_cmds: &[T],
        data: &mut GalawData<T>,
    ) -> Result<(), GalawError<T>>;

    pub fn compute_link_jacobians(
        &self,
        joint_cmds: &[T],
        data: &mut GalawData<T>,
    ) -> Result<(), GalawError<T>>;

    pub fn compute_ik(
        &self,
        target_link_idx: usize,
        target_pose: &Isometry3<T>,
        init_joint_cmds: &[T],
        data: &mut GalawData<T>,
    ) -> Result<Vec<T>, GalawError<T>>;
}
```

`model` is stateless — it never holds mutable state. `GalawData` is the single source of
mutable workspace for all computations.

## Parallelism

`GalawModel<T>` is `Sync`. `GalawData<T>` is `Send`. The user creates a pool and rayon
handles the rest — Rust's borrow checker enforces disjoint access with no unsafe:

```rust
use rayon::prelude::*;

let mut data_pool: Vec<GalawData<f64>> = (0..n_poses)
    .map(|_| model.create_data())
    .collect();

cmds_batch.par_iter()
    .zip(data_pool.par_iter_mut())
    .for_each(|(cmds, data)| {
        model.compute_link_jacobians(cmds, data).unwrap();
    });

let jacobians = &data_pool[0].jacobians;
```

For memory-constrained cases, use one `GalawData` per thread instead of one per pose, via
rayon's `for_each_with` (requires `Clone` on `GalawData`).

## Benefits

- **Zero per-call allocations.** All workspace is pre-allocated in `GalawData`.
- **Correct by construction.** `create_data()` initializes buffers correctly. The
  zeroing contract and size-mismatch footguns disappear.
- **Trivially parallel.** One `GalawData` per task. No coordination required.
- **Extensible.** New computations (dynamics, velocity propagation) add fields to
  `GalawData` without changing method signatures of existing computations.

## Downsides / Open Questions

- **Memory cost.** A pool of N `GalawData` instances allocates N × (all buffers). For
  large N, one-per-thread is more memory-efficient but less ergonomic.
- **Generated code.** Generated functions use fixed-size stack arrays and won't use
  `GalawData` — the runtime and generated APIs remain separate.
- **Field visibility.** `fk_poses`, `jacobians`, etc. are public for ergonomics, but
  their contents are only valid after a corresponding `compute_*` call. This should be
  clearly documented.
- **IK scratch space.** The IK solver has its own internal state (Jacobian for a single
  link, delta_q buffer, damping scalars). These need to be specified before implementation.

## Scope

This is a new feature and a breaking refactor of all public kinematics methods. It should
be its own PR after the zero-alloc out-param work is merged.

### Out of scope for this PR

- Rayon dependency and batch convenience methods
- Generated code integration
- Dynamics / velocity / acceleration buffers
