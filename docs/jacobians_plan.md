# galaw — Jacobian Implementation Plan

Plan for adding geometric-Jacobian computation to both APIs galaw already
has for FK: a runtime `GalawModel` method (`src/kinematics.rs`) and an
ahead-of-time codegen'd function per robot (`src/bin/codegen_fk.rs` →
`src/generated/*.rs`). Also covers how to check correctness, including a
comparison against [`k`](https://crates.io/crates/k) — the same crate
`compute_fk` is already checked against in `tests/fk_correctness.rs`.

This is documentation only. No code has been changed yet; open questions
below (marked **DECISION**) should be settled before writing any of it.

## What a Jacobian means here

For a chosen target link, the geometric Jacobian `J` is the 6×N matrix
(N = number of actuated joints on the path from root to that link) mapping
joint velocities to that link's spatial velocity, expressed in the world
frame:

```
[v; ω] = J * qdot
```

`v` (rows 0-2) is linear velocity of the target link's origin, `ω`
(rows 3-5) is angular velocity. Each column comes from one actuated joint
`i` on the path from root to the target link, using its *current* (i.e.
post-FK, not URDF-rest) world-frame position `p_i` and axis `a_i`:

| joint type | linear column (rows 0-2) | angular column (rows 3-5) |
|---|---|---|
| revolute/continuous | `a_i × (p_target − p_i)` | `a_i` |
| prismatic | `a_i` | `0` |
| fixed | — (not actuated, no column) | — |

where `p_target` is the target link's current world-frame origin, `p_i` is
joint `i`'s current world-frame origin, and `a_i` is joint `i`'s axis
rotated into the current world frame. This is exactly what `k::jacobian`
computes (confirmed by reading `k`'s source, see below) — same convention,
same frame, same row ordering — which is what makes a direct comparison
possible.

**Confirmed by reading `k` 0.32.0's source** (`~/.cargo/registry/.../k-0.32.0/src/funcs.rs`,
`pub fn jacobian<T>(arm: &SerialChain<T>) -> DMatrix<T>`):
- Free function, not a method — takes a `k::SerialChain`, not a general
  (possibly branching) `k::Chain`. `k::SerialChain::from_end(&node)` builds
  the root→node chain for a given end link/joint.
- Returns a `6 × dof` matrix, `dof` = number of movable (non-fixed) joints
  on that chain. Rows 0-2 = linear, rows 3-5 = angular — matches the table
  above exactly, formula-for-formula.
- `iter_joints()` (used internally) already skips `Fixed` joints — it
  iterates `movable_nodes`, so galaw's own `cmd_idx.is_some()` filter lines
  up with it directly.

## Column ordering matches galaw's `cmd_idx` for free

galaw assigns `cmd_idx` via a whole-tree DFS pre-order walk
(`parser.rs::resolve_joint_order`), and that function's own doc comment
already establishes (and `tests/fk_correctness.rs` already relies on) that
`k::Chain` numbers its DOFs the same way. A DFS pre-order walk assigns a
node's index *before* descending into its subtree, so for any node, the
`cmd_idx` values of its ancestors are visited in increasing order along the
root→node path — i.e. **the subsequence of `cmd_idx` values for the joints
on the path to a given link is already in the same relative order `k`'s
`SerialChain` would report for that link.**

Practical upshot: galaw's Jacobian can be a full **N = `num_actuated_joints`
wide**, zero-padded matrix (nonzero only in the columns of joints actually
on the path to the target link) rather than a narrower per-chain matrix.
That keeps the shape uniform and directly `qdot`-multipliable regardless of
which link is targeted, and it's still trivial to compare against `k`'s
narrower per-`SerialChain` matrix: `k`'s column `j` for a given link is
galaw's column at that ancestor joint's `cmd_idx`, in order, with galaw's
matrix having zeros everywhere else.

## Runtime implementation (`src/kinematics.rs`)

New method, next to `compute_fk`:

```rust
pub fn compute_jacobian(
    &self,
    joint_cmds: &[f64],
    target_link_idx: usize,
) -> Result<Matrix6xX<f64>, GalawError>
```

(`nalgebra::Matrix6xX<f64>` — dynamic column count, fixed 6 rows — mirrors
what `k::jacobian` returns structurally, without depending on `k` from
non-test code.)

Steps:
1. Validate `joint_cmds.len()` (existing check, reused) and
   `target_link_idx < self.links.len()` — new error variant needed, e.g.
   `KinematicsError::LinkIdxOutOfBounds { num_links, requested }`, same
   shape as the existing `JointCmdLengthMismatch`.
2. Run the same walk `compute_fk` already does, but retain per-joint data
   instead of discarding it: for each joint, its post-FK world transform
   (giving `p_i`) and its world-frame axis (`joint_world_transform.rotation
   * axis`). **DECISION**: either (a) have `compute_jacobian` call
   `compute_fk` and then redo a lightweight pass to recover per-joint axes
   from the joint list + returned poses, or (b) factor `compute_fk`'s loop
   body into a shared private helper both methods call, returning
   `(links: Vec<Isometry3<f64>>, joint_world_axes: Vec<Option<Vector3<f64>>>)`
   so the work isn't duplicated. (b) is the less wasteful option and keeps
   `compute_fk`'s hot path unchanged (it still returns just `Vec<Isometry3>`).
3. Walk from `target_link_idx` back to the root to find which joints are
   ancestors. `Joint` currently has `parent_link_idx`/`child_link_idx` but
   there's no existing `link → its incoming joint` map. Build one
   (`child_link_idx → joint index`, O(joints), can be a local `HashMap` built
   once per call, or precomputed once in `GalawModel` at parse time if this
   method ends up being called in a hot loop — start with local, optimize
   later if benchmarks show it matters). Walk parent-wards from
   `target_link_idx` until hitting a link with no entry (the root).
4. For each ancestor joint with `cmd_idx.is_some()`, fill in its column at
   that `cmd_idx` per the table above, using `p_target` (target link's world
   translation) and that joint's `p_i`/`a_i`. All other columns stay zero.

## Generated implementation (`src/bin/codegen_fk.rs` or a sibling binary)

**DECISION**: extend `codegen_fk` to also emit a `compute_jacobian` per
robot, or add a new `codegen_jacobian` binary/module. Given the generated
Jacobian needs the same per-joint `link_X` variables the FK codegen already
emits (see below), extending the existing generator (same output file,
`src/generated/<robot>.rs`) is the more natural fit and avoids parsing the
URDF twice — lean towards that unless it makes `codegen_fk.rs` unwieldy
enough to warrant splitting `generate_fk_fn_code`/`generate_jacobian_fn_code`
into separate files sharing the parsed `GalawModel`.

Key reuse: `generate_fk_fn_code` already emits, per joint, a
`let link_<child> = link_<parent> * ...;` binding whose value **is** that
joint's post-FK world transform — exactly `p_i` (its `.translation`) and the
frame needed to rotate the joint's (compile-time-constant) local axis into
world space (`.rotation * Vector3::new(axis_x, axis_y, axis_z)`). So Jacobian
codegen doesn't need a new runtime traversal at all — for each link L, walk
its ancestor joints (known at *codegen* time, on the host, from
`galaw_model.joints`/`parent_link_idx`, same way `codegen_fk` already knows
the tree shape) and emit one column expression per ancestor referencing the
already-bound `link_<...>` variables, e.g.:

```rust
// ancestor joint j (revolute), contributing to link L's Jacobian column j.cmd_idx
let axis_world_j = link_<j.child>.rotation * Vector3::new(ax, ay, az);
// column = axis_world_j.cross(&(link_<L>.translation.vector - link_<j.child>.translation.vector))
```

Signature and **DECISION** on shape — two options:
- **All links at once**, mirroring `compute_fk`'s existing "compute
  everything, index by link" design:
  `pub fn compute_jacobian(joint_cmds: &[f64; N]) -> [SMatrix<f64, 6, N>; M]`.
  Consistent with the crate's existing stateless/no-target-param style, but
  O(links × joints) work and generated code size, vs FK's O(joints) — most
  of that cost is dead weight for callers who only ever want one link's
  Jacobian (e.g. an end-effector).
- **Per-link, parameterized at codegen time**: emit `compute_jacobian_<link
  name>(joint_cmds: &[f64; N]) -> SMatrix<f64, 6, N>` only for links a
  robot's author actually cares about (or all of them, still N functions).
  Cheaper per call, more generated surface area, and a different calling
  convention than every other function in `generated/`.

Recommendation: start with **all-links-at-once** for API consistency with
`compute_fk`, note the cost tradeoff in the doc comment, and revisit if a
benchmark (same pattern as `benches/fk_speed.rs`) shows it's a problem for
any of the larger corpus robots (anymal_d, stretch4).

## Verification plan

Mirror `tests/fk_correctness.rs`'s two-pronged structure in a new
`tests/jacobian_correctness.rs`:

**1. Against `k`, per link, over randomized in-limit joint configs**
(the direct answer to "compare it to k"):
```rust
let k_serial = k::SerialChain::from_end(&k_chain.find_link(&link.name).unwrap());
k_chain.set_joint_positions(joint_cmd)?;
k_chain.update_transforms();
let k_jac = k::jacobian(&k_serial); // 6 x (ancestors of this link)
```
Compare column-by-column: for the i-th ancestor joint in `k_jac`'s columns
(in order), find that joint's `cmd_idx`, and assert it matches galaw's
column at that index; assert all of galaw's other columns are exactly zero.
Run over the same zero-pose + `NUM_POSES` random-pose loop the FK tests
already use, for every URDF in the existing `fk_correctness_tests!` list —
reuses `setup_kinematic_models` as-is.

**2. Generated vs. runtime**, symmetric to `check_generated_matches_dynamic`:
same `for_each_generated_robot!` registry, compares the codegen'd
`compute_jacobian` against `GalawModel::compute_jacobian` over random poses.
Catches codegen bugs independent of whether the runtime formula itself is
right.

**3. Finite-difference self-check (independent of both `k` and of galaw's
own closed-form code path)** — worth adding because (1) and (2) can't catch
a bug present in *both* `k` and galaw, or a codegen bug that happens to
mirror a runtime bug. For each actuated joint `i`, perturb `joint_cmds[i]`
by `±ε` and re-run `compute_fk`:
- linear velocity column ≈ `(p(q+ε·e_i) − p(q−ε·e_i)) / (2ε)`
- angular velocity column ≈ `(R(q+ε·e_i) * R(q−ε·e_i).inverse()).axis_angle()`
  (via `UnitQuaternion::axis_angle()`, scaled by `angle / (2ε)`, zero if the
  angle is ~0) — standard central-difference approach for rotations, since
  you can't finite-difference a quaternion component-wise and get an
  angular-velocity vector directly.

This is pure `compute_fk`-based, no `k` dependency, and doubles as a good
one-off way to sanity-check the closed-form formula while implementing it,
before wiring up the `k` comparison at all.

## Open decisions to settle before implementing

1. Shared traversal: factor `compute_fk`'s per-joint loop into a helper
   both `compute_fk` and `compute_jacobian` call (recommended), vs.
   `compute_jacobian` re-deriving per-joint world axes from `compute_fk`'s
   output + the joint list.
2. Generated API shape: all-links-at-once `[SMatrix<f64,6,N>; M]`
   (recommended, consistent with `compute_fk`) vs. per-link functions.
3. Where the new error variant(s) live: extend `KinematicsError` with
   `LinkIdxOutOfBounds` (and anything else `compute_jacobian` needs to
   reject).
4. One `codegen_fk.rs` emitting both functions per robot, vs. splitting
   Jacobian codegen into its own module/binary.
5. Whether the ancestor `child_link_idx → joint` map used by the runtime
   method is built fresh per call (simple, start here) or precomputed once
   on `GalawModel` at parse time (only worth it if profiling shows
   `compute_jacobian` is called in a hot loop the way `compute_fk` is).

## Files touched (once decisions above are made)

- `src/kinematics.rs` — `compute_jacobian` method (+ possible shared helper
  factored out of `compute_fk`).
- `src/error.rs` — new `KinematicsError` variant(s).
- `src/bin/codegen_fk.rs` — new `generate_jacobian_fn_code` (or a sibling
  file), wired into `main()`'s existing per-robot codegen pass.
- `src/generated/*.rs` — regenerated via `scripts/codegen_all_urdfs.sh` once
  codegen support exists.
- `tests/jacobian_correctness.rs` — new file, three test groups above.
- `README.md` — extend the feature list / quick-start once both APIs exist,
  same way FK's runtime-vs-generated sections read today.
