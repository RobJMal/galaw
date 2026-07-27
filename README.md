# galaw - A fast kinematics library
*galaw* (pronounced gah-LOW, rhymes with "cow") is the Tagalog word that means movement or motion. 

## Quick Start

`galaw` has two APIs for computing forward kinematics, with different performance/flexibility tradeoffs:

- **Runtime** — parses a URDF at runtime, works with *any* robot.
- **Generated** — ahead-of-time code generation, fixed to *one* robot at compile time. ~2-14x faster, no parsing or `Result` handling on the hot path.

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

## Attributions

This repository incorporates assets and robot descriptions from various open-source projects. Each asset is used in compliance with its original license:

* **Enlight-L (Flexiv)** – Derived from [flexiv_description](https://github.com/flexivrobotics/flexiv_description). Licensed under the **Apache License 2.0** (see `LICENSE` in the `Flexiv_Enlight-L` directory or the original notice for details). *Note: Modified locally to update mesh resource paths.*
  * *Local Changes:* Repackaged URDF and meshes into a flat `Flexiv_Enlight-L/` directory; modified mesh resource paths to be relative to `meshes/`.
  * *License Copy:* Located at `Flexiv_Enlight-L/LICENSE.md`
* **ANYmal D (ANYbotics)** – Derived from the [anymal_d_simple_description](https://github.com/ANYbotics/anymal_d_simple_description?tab=BSD-3-Clause-1-ov-file) project. Licensed under the **BSD 3-Clause License**. 
  * *Local Changes:* Repackaged URDF and meshes into a flat `ANYbotics_ANYmal-D/` directory; modified mesh resource paths to be relative to `meshes/`.
  * *License Copy:* Located at `ANYbotics_ANYmal-D/LICENSE.md`
* **Wuji Hand (Wuji Technology)** – Derived from the [wuji-description](https://github.com/wuji-technology/wuji-description) project. Licensed under the **MIT License**.
  * *Local Changes:* Repackaged URDF and meshes into a flat `Wuji-Technology_Wuji-Hand/` directory; modified mesh resource paths to be relative to `meshes/`.
  * *License Copy:* Located at `Wuji-Technology_Wuji-Hand/LICENSE.md`
* **Stretch 4 (Hello Robot)** – Derived from the [stretch4_urdf](https://github.com/hello-robot/stretch4_urdf) project. Licensed under the **Clear BSD License**.
  * *Local Changes:* Repackaged URDF and meshes into a flat `Hello-Robot_Stretch4/` directory; modified mesh resource paths to be relative to `meshes/`.
  * *License Copy:* Located at `Hello-Robot_Stretch4/LICENSE.md`


Copies of the original licenses and any accompanying `NOTICE` files are preserved in the root directory or alongside the respective robot package folders.
