# galaw — Benchmarking Plan: `rapier` and `pinocchio-rs`

Plan for adding two new comparison targets to the `fk_speed` bench suite:
`rapier` (native Rust, reduced-coordinates multibody) and `pinocchio-rs`
(Rust wrapper around C++ Pinocchio). Dependency integration is **not** done
here — both have real build/dependency implications that need to be handled
carefully, and are deliberately deferred. This is documentation only.

A note on confidence before anything else: some of the specifics below came
from fetching crates.io/lib.rs pages through a web-fetch tool that, earlier
in this investigation, fabricated plausible-sounding details for a crate
page it couldn't actually load. Where that happened it's been caught and
corrected, but treat every claim below as **"verify by reading the actual
repo/Cargo.toml before depending on it,"** not as settled fact — especially
anything about build systems, since that's exactly what determines how much
work the "dependencies need to be managed properly" work later actually is.

## Target 1: `rapier`

- **What it is**: [rapier](https://rapier.rs) is dimforge's Rust physics
  engine. Most people know it for maximal-coordinate rigid-body/contact
  simulation, but it also has a `MultibodyJointSet` using the
  **reduced-coordinates formalism** — the same family of approach as
  Pinocchio and `rigidbody-rs` — documented at
  [rapier.rs/docs/user_guides/rust/joint_constraints](https://rapier.rs/docs/user_guides/rust/joint_constraints/).
  There's also a dedicated [`rapier3d-urdf`](https://crates.io/crates/rapier3d-urdf)
  crate ([docs.rs](https://docs.rs/rapier3d-urdf/latest/rapier3d_urdf/)) that
  loads a URDF directly into that system.
- **Why it's a fair comparison**: it's the architectural peer to
  `rigidbody-rs`/Pinocchio (reduced coordinates), not the "physics engine,
  therefore irrelevant" reaction the name might trigger — and it's pure
  Rust, no FFI, matching galaw's own "no C++ toolchain" positioning.
- **The one real caveat**: a `MultibodyJointSet` sits inside rapier's
  broader simulation pipeline. Benchmarking must isolate whatever pose/FK
  query it exposes — **not** call `step()` on the whole simulation, which
  would also run constraint solving and measure a different thing entirely.
  This needs to be confirmed by reading `rapier3d`'s actual API before
  wiring up a bench, not assumed from the docs page alone.
- **Dependency shape**: pure Rust crates (`rapier3d`, `rapier3d-urdf`) —
  no C/C++ toolchain, no build script beyond normal Cargo. Lowest-risk of
  the two integrations.

## Target 2: `pinocchio-rs`

- **What it is**: [github.com/BertrandBev/pinocchio-rs](https://github.com/BertrandBev/pinocchio-rs)
  — a Rust wrapper around the C++ [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
  library (URDF loading, forward kinematics, dynamics simulation via RK4
  and semi-implicit Euler integration). Published on crates.io as
  `pinocchio_rs`.
- **Why it's the one the existing landscape write-up wanted**: this is a
  genuine "Rust via FFI bindings" incumbent — the comparison this doc has
  wanted since it first listed Pinocchio as the gold standard, previously
  marked as "no mature Rust binding exists."  That line needs updating once
  this is confirmed integrated, not before.
- **Dependency shape — the part that needs real care**: per its README, it
  ships **pre-built binaries for Linux/macOS**, or falls back to full
  source compilation requiring **`xmake`** installed (not plain `cargo
  build` — a second build tool in the chain). It wraps C++ Pinocchio itself,
  so the underlying C++ library's own dependencies (likely Eigen, Boost, or
  similar — confirm from Pinocchio's own build docs) are transitively in
  play too.
- **Implications for integrating this into `galaw`'s own build**:
  - This should almost certainly be gated behind an **optional Cargo
    feature** (e.g. `bench-pinocchio`), not a plain `[dev-dependencies]`
    entry — otherwise every contributor running `cargo test`/`cargo bench`
    on a machine without `xmake` (or on an unsupported OS for the
    prebuilt-binary path) gets a broken build for something unrelated to
    their change.
  - CI would need either `xmake` installed on the bench runner, or to only
    run this comparison on the OSes the prebuilt binaries cover.
  - Worth checking whether the prebuilt-binary path is good enough for
    benchmarking at all — a prebuilt binary compiled with unknown
    optimization flags on someone else's machine is a confound for a
    performance comparison; the source-build (`xmake`) path may be the only
    one that gives a fair, controlled comparison.

## What "handled properly" means when this is picked back up

Before wiring either into `benches/fk_speed.rs`:
1. Confirm rapier's isolated FK/pose-query API (not `step()`) by reading
   `rapier3d`'s actual source/docs, not this summary.
2. Confirm `pinocchio_rs`'s exact build requirements by reading its
   `Cargo.toml`/`build.rs` directly, and decide feature-gating before
   adding it as a dependency at all.
3. Re-apply the same fairness discipline as the existing `k`/`rigidbody-rs`
   comparisons: release build, single-threaded, identical robot + joint
   configs, warm FK call only (exclude model/URDF loading from the timed
   region).
