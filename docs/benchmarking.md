# galaw — Benchmarking Plan & Competitive Landscape

A native Rust forward-kinematics solver. Goal: match or beat existing solvers
for a *specialized* robot (via "compile the URDF" code generation), while
staying pure-cargo — no C++ toolchain, no FFI.

## Rust FK comparison targets

| Library | What it is | Why benchmark against it | Links |
|---|---|---|---|
| **k** | General URDF FK/IK on nalgebra; de-facto standard | Accessible baseline — if we can't beat `k`, we can't beat pinocchio | [GitHub](https://github.com/openrr/k) · [crates.io](https://crates.io/crates/k) · [docs.rs](https://docs.rs/k) |
| **rigidbody-rs** | Featherstone-based URDF kinematics + dynamics | Closest to pinocchio's *algorithm* in pure Rust — the credible "am I competitive?" proxy | [GitHub](https://github.com/khaninger/rigidbody-rs) |
| **rs-opw-kinematics** | Closed-form FK/IK for 6-axis ortho-parallel/spherical-wrist robots | Not general, but the *specialization ceiling* — the extreme our codegen approaches | [GitHub](https://github.com/bourumir-wyngs/rs-opw-kinematics) · [crates.io](https://crates.io/crates/rs-opw-kinematics) |

## Adjacent / future (IK — not FK, but know they exist)

| Library | What it is | Links |
|---|---|---|
| **optik** | Fast optimization-based IK (SE(3), analytic gradients, parallel restarts) | [GitHub](https://github.com/kylc/optik) |
| **ik-geo** | Geometric/analytical IK | [crates.io](https://crates.io/crates/ik-geo) |
| **relaxed_ik** | Relaxed IK core | [site](https://pages.graphics.cs.wisc.edu/relaxed_ik_core/) |

## Pinocchio — the gold standard (two separate comparisons)

[Pinocchio](https://github.com/stack-of-tasks/pinocchio) is the C++ reference implementation.
Benchmark it **two ways**, because they answer different questions:

| Comparison | Question it answers | Notes |
|---|---|---|
| **C++ native** (its own `bench/timings.cpp`, or a Python `pin` + `timeit` harness) | Algorithmic ceiling — is our *approach* fundamentally competitive? | Excludes FFI overhead. Honest measure of pinocchio's raw speed. Pinocchio also has a CppADCodeGen path — the specialized-vs-specialized rival. |
| **Rust via FFI bindings** (would need building with `cxx`) | Realistic incumbent — what a Rust dev *actually* pays today | Includes per-call FFI overhead. Likely where `galaw` wins even without beating raw C++. No mature Rust binding exists — its absence is itself part of our value prop. |

### The "bindings tax" (qualitative, but real for adoption)
Beyond ns/op, a native crate saves Rust users: no C++ toolchain, no `unsafe`
FFI boundary, smaller binaries, simpler cross-compilation and deployment.

## Fairness checklist (applies to every comparison)

- [ ] **Same operation** — FK for *all* frames vs. a single frame; recompute vs. cached.
- [ ] **Exclude model loading** — measure only the warm FK call.
- [ ] **Release build**, `-O3` / `target-cpu=native` on both sides.
- [ ] **Single-threaded** unless explicitly measuring throughput.
- [ ] **Identical robot + identical joint configurations.**
- [ ] **State single-query vs. batched** explicitly — pinocchio's edge is partly SIMD/batch.

## Foundational (not competitors)

- **urdf-rs** — URDF parser under `k` — [GitHub](https://github.com/openrr/urdf-rs) · [crates.io](https://crates.io/crates/urdf-rs)
- **nalgebra / parry / rapier** ([dimforge](https://dimforge.com)) — math & physics backbone.