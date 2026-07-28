# `galaw` - A Rust-based kinematics library
*galaw* (pronounced gah-LOW, rhymes with "cow") is the Tagalog word that means movement or motion. This library is for computing kinematics. 

## Features

- **Stateless** — `compute_fk` takes joint commands, returns fresh poses. No mutable state, no setup step, safe to call concurrently.
- **Code-generated (optional)** — ahead-of-time `compute_fk` per robot, no parsing or `Result` on the hot path, ~1.6-4.5x faster than runtime.
- **Correctness-tested** — checked against [`k`](https://crates.io/crates/k) across randomized joint configs within limits.
- **Named lookups** — command joints/links by name, never by assumed index.
- **Descriptive errors** — malformed URDFs fail with a specific cause, not a panic.

## Quick Start

`galaw` has two APIs for computing forward kinematics, with different performance/flexibility tradeoffs:

- **Runtime** — parses a URDF at runtime, works with *any* robot.
- **Generated** — ahead-of-time code generation, fixed to *one* robot at compile time. ~1.6-4.5x faster, no parsing or `Result` handling on the hot path.

### Runtime

```rust
use galaw::{error::GalawError, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Command each actuated joint by name — never by assumed position.
    let mut joint_cmds = vec![0.0; model.num_actuated_joints];
    let shoulder_idx = model.get_joint_idx("shoulder_joint").expect("shoulder_joint exists in URDF");
    let elbow_idx = model.get_joint_idx("elbow_joint").expect("elbow_joint exists in URDF");
    joint_cmds[shoulder_idx] = 0.5;
    joint_cmds[elbow_idx] = -0.3;

    let poses = model.compute_fk(&joint_cmds)?;
    println!("{:?}", poses);
    Ok(())
}
```

Full runnable version: [`examples/basic_fk.rs`](examples/basic_fk.rs) — `cargo run --example basic_fk`

### Generated

Ahead of time, generate fixed FK code for a specific robot (code for the robots shipped with this repo already exists under `src/generated/` — see `galaw::generated`):

```
# 1st arg: urdf_path, 2nd arg: out_path
cargo run --bin codegen_fk -- assets/urdf/custom/simple_arm_2dof.urdf src/generated/simple_arm_2dof.rs
```

Then call the generated function directly — no `GalawModel`, no `Result`, no parsing at call time:

```rust
use galaw::generated::simple_arm_2dof;

let joint_cmds: [f64; 2] = [0.5, -0.3];
let poses = simple_arm_2dof::compute_fk(&joint_cmds);
```

Full runnable version: [`examples/generated_fk.rs`](examples/generated_fk.rs) — `cargo run --example generated_fk`

### Which one should I use?

**Runtime** if you need to support arbitrary URDFs at runtime — e.g. a robot chosen by a user, or loaded from a file you don't control at compile time. **Generated** if you know the robot ahead of time and want the fastest possible FK, at the cost of a codegen step and one generated file per robot.

## Performance

`galaw` outperforms [`k`](https://crates.io/crates/k) overall. Averaged (geometric mean) across the four benchmarked robots: `galaw-runtime` is **~3.3x** faster than `k`, and `galaw-generated` is **~8.9x** faster than `k`.

Benchmarked on: Intel Core i7-10750H @ 2.60GHz (6C/12T, boost up to 5.0GHz), 16GB RAM, Ubuntu 22.04.5 LTS (kernel 6.8).

![FK latency scaling](img/scaling_ns_per_call.png)
![FK throughput](img/throughput_mcalls.png)

Reproduce with `cargo bench` followed by `cargo run --release --example plot_bench` (see `benches/fk_speed.rs` / `examples/plot_bench.rs`).

## Attributions

This repository incorporates robot descriptions (URDF files) from various open-source projects. Each is used in compliance with its original license:

> **Note:** these robots' visual/collision mesh files (STL/DAE geometry) have been removed from this repo since `galaw` only parses a URDF's kinematic structure. The URDF files still reference `meshes/...` paths for compatibility with other tools. If the actual mesh geometry is needed, get it from the original project linked below.

* **Enlight-L (Flexiv)** – Derived from [flexiv_description](https://github.com/flexivrobotics/flexiv_description). Licensed under the **Apache License 2.0** (see `LICENSE` in the `Flexiv_Enlight-L` directory or the original notice for details). *Note: Modified locally to update mesh resource paths.*
  * *Local Changes:* Repackaged URDF into a flat `Flexiv_Enlight-L/` directory; modified mesh resource paths to be relative to `meshes/`; mesh files themselves removed (see note above).
  * *License Copy:* Located at `Flexiv_Enlight-L/LICENSE.md`
* **ANYmal D (ANYbotics)** – Derived from the [anymal_d_simple_description](https://github.com/ANYbotics/anymal_d_simple_description?tab=BSD-3-Clause-1-ov-file) project. Licensed under the **BSD 3-Clause License**. 
  * *Local Changes:* Repackaged URDF into a flat `ANYbotics_ANYmal-D/` directory; modified mesh resource paths to be relative to `meshes/`; mesh files themselves removed (see note above).
  * *License Copy:* Located at `ANYbotics_ANYmal-D/LICENSE.md`
* **Wuji Hand (Wuji Technology)** – Derived from the [wuji-description](https://github.com/wuji-technology/wuji-description) project. Licensed under the **MIT License**.
  * *Local Changes:* Repackaged URDF into a flat `Wuji-Technology_Wuji-Hand/` directory; modified mesh resource paths to be relative to `meshes/`; mesh files themselves removed (see note above).
  * *License Copy:* Located at `Wuji-Technology_Wuji-Hand/LICENSE.md`
* **Stretch 4 (Hello Robot)** – Derived from the [stretch4_urdf](https://github.com/hello-robot/stretch4_urdf) project. Licensed under the **Clear BSD License**.
  * *Local Changes:* Repackaged URDF into a flat `Hello-Robot_Stretch4/` directory; modified mesh resource paths to be relative to `meshes/`; mesh files themselves removed (see note above).
  * *License Copy:* Located at `Hello-Robot_Stretch4/LICENSE.md`


Copies of the original licenses and any accompanying `NOTICE` files are preserved in the root directory or alongside the respective robot package folders.
