# Support real-world joint types + fix a latent ordering bug in the URDF parser

## Context

`src/parser.rs`'s `load_urdf` currently assumes every joint is `revolute`: it hard-requires `<axis>` and `<limit>` on every joint and never reads the `type` attribute at all. Real robot URDFs (which the user is about to load) routinely use `fixed` joints (sensor/frame mounts) and `continuous` joints (wheels), and often `prismatic` (grippers, slides) — all of which will fail to parse today. Separately, while investigating how to make command-vector indexing scale to real (possibly branching) robots, I found that `k::Chain` (the crate's own dependency, already used in `tests/fk_correctness.rs` as an independent ground truth) orders its movable joints via **DFS pre-order** (confirmed by reading `k`'s `iterator.rs`/`chain.rs`), while galaw's parser resolves joint order via **BFS**. For today's non-branching chain fixtures the two orders coincide by accident; for any branching real robot they will diverge, silently misassigning commands to the wrong joint. This plan fixes both problems together, since the joint-type work already requires touching the same code path (`cmd_idx` assignment).

User confirmed scope: add support for **revolute, continuous, prismatic, and fixed** joint types (the four that appear in effectively all real robot URDFs).

## Design

### 1. `src/types.rs` — replace parallel `Option`s with an enum

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum JointKinematics {
    Rotational { axis: Unit<Vector3<f64>>, limits: Option<(f64, f64)> }, // revolute (Some) or continuous (None)
    Prismatic  { axis: Unit<Vector3<f64>>, limits: Option<(f64, f64)> },
    Fixed,
}
```

- `Joint` drops `axis`/`limit_lower`/`limit_upper` in favor of a single `kinematics: JointKinematics` field — this rules out illegal states (e.g. `Fixed` with an axis) at compile time instead of via convention. This mirrors `k`'s own `JointType` design (it also collapses revolute/continuous into one `Rotational{axis}` variant distinguished only by an optional limit) — useful precedent since we cross-validate against `k` directly in tests.
- `cmd_idx: usize` → `cmd_idx: Option<usize>` — `Fixed` joints have 0 DOF and must not consume a slot in `joint_cmds` (matches how `k::Chain::dof()`/`movable_nodes` already exclude `Fixed`).
- Add `GalawModel::num_actuated_joints(&self) -> usize` (count of `cmd_idx.is_some()`), used to size `joint_cmds` and in `compute_fk`'s length check.
- `joint_name_to_idx` (built in parser.rs) is populated via `filter_map` over joints with `cmd_idx.is_some()`, so `get_joint_idx("some_fixed_joint")` naturally returns `None` — no change needed to the accessor itself, just what gets inserted.

### 2. `src/parser.rs` — split `load_urdf`, fix ordering, support all 4 types

Split the current 170-line function into:
- **`parse_link(node) -> Result<Link, _>`**
- **`parse_joint(node) -> Result<Joint, _>`** — reads the `type` attribute and maps it: `revolute`/`continuous` → `Rotational` (limits `Some` for revolute, `None` for continuous — regardless of whether a `<limit>` tag with only `effort`/`velocity` is present), `prismatic` → `Prismatic` (limits required), `fixed` → `Fixed` (no axis/limit read at all). Any other value (`floating`, `planar`, `spherical` — the remaining URDF spec types) is a clear error naming all three as unsupported. Also makes `<origin>` default to identity and `<axis>` default to `(1,0,0)` when omitted, per URDF spec — both are commonly omitted in real files, and requiring them today would fail on well-formed real URDFs.
- **`resolve_joint_order(links, joints) -> Result<Vec<Joint>, _>`** — root-finding (logic unchanged) + build an adjacency map `HashMap<usize, Vec<usize>>` from `parent_link_idx` → joint indices **in file-declaration order** (built once, O(E); confirmed this matches how `k` itself builds its tree — it iterates `robot.joints` in file order per parent) + a **recursive DFS pre-order** walk (not the current BFS) assigning `cmd_idx = Some(counter)` to non-`Fixed` joints as they're visited, `None` to `Fixed`. Recursion depth is bounded by robot depth (never realistically more than a few dozen), so no stack-depth concern. This replaces the current O(V·E) re-filter-per-pop traversal with O(V+E) and — critically — makes galaw's joint order match `k`'s, so `joint_cmds` built by iterating `model.joints` positionally (as `main.rs`/tests/benches already do) lines up correctly for branching robots, not just chains.
- `load_urdf` becomes a short orchestrator: parse XML → collect links/joints via the two parse functions → `resolve_joint_order` → build `link_name_to_idx`/`joint_name_to_idx` → construct `GalawModel`.

**Explicitly not doing** (flagging so it's a deliberate choice, not an oversight):
- No custom error enum — repo has zero precedent (everything is ad-hoc `Box<dyn std::error::Error>`); introducing one now is scope creep beyond what was asked.
- No submodule split (`parser/xml.rs` etc.) — the file stays single, ~250-300 lines with clearly separated private functions; that's not large enough yet to warrant module boundaries.
- `<mimic>` is not read/honored. Confirmed `k` itself still gives a mimicked joint a real `Rotational`/`Prismatic` type (it still consumes a DOF slot) — so an unhandled `<mimic>` joint parses and runs fine under this design, it's just controlled independently rather than slaved to its driving joint. `<safety_controller>`/`<calibration>` are metadata-only (confirmed `k` never reads them either) — safe to silently ignore.
- No cycle detection in `resolve_joint_order`. A malformed URDF where a link is the `child` of two different joints (not a valid tree) could in principle cause unbounded recursion — this is a pre-existing gap (the old BFS had the same non-termination risk), not something introduced by this change, and out of scope for "support real, well-formed robot URDFs."

### 3. `src/kinematics.rs` — branch on joint kinematics

`compute_fk`'s length check uses `self.num_actuated_joints()` instead of `self.joints.len()`. The per-joint loop matches on `joint.kinematics`:
- `Fixed` → `joint_local = joint.transform` (no command consumed)
- `Rotational { axis, .. }` → existing rotation-via-axis-angle logic, reading `joint_cmds[joint.cmd_idx.unwrap()]`
- `Prismatic { axis, .. }` → translation along `axis` scaled by the command value, using `Isometry3::from_parts(Translation3::from(axis.into_inner() * cmd), UnitQuaternion::identity())`, instead of rotation

### 4. Update callers: `src/main.rs`, `tests/fk_correctness.rs`, `benches/fk_speed.rs`

All three currently do `vec![0.0; galaw_model.joints.len()]` and/or iterate `galaw_model.joints` using `j.limit_lower..j.limit_upper` to build random commands. Change to:
- Size vectors with `galaw_model.num_actuated_joints()`.
- Filter to `j.cmd_idx.is_some()` (equivalently `joint.kinematics != Fixed`) when building per-joint random values, matching on `joint.kinematics` to get a `(lower, upper)` range — for `Rotational` with `limits: None` (continuous), fall back to a fixed test range like `-PI..PI` since there's no file-declared bound to sample.

### 5. New fixtures + tests (kept as two separate, isolated fixtures so a failure points at one concern)

- **`assets/mixed_joint_types.urdf`** — a single non-branching chain (like today's fixtures) but including one `fixed` link (e.g. a sensor mount), one `continuous` joint, and one `prismatic` joint alongside revolute joints. Isolates joint-type parsing/kinematics-math correctness.
- **`assets/branching_robot.urdf`** — a root link splitting into two child chains, plain revolute joints only. Isolates the BFS→DFS ordering fix with no confounding joint-type variables.
- Add both to the existing `fk_correctness_tests!` macro in `tests/fk_correctness.rs` (one line each) — this gets full end-to-end validation against `k::Chain` for free, for both new concerns.
- New `parser.rs` unit tests: fixed joint parses with no axis/limit; continuous joint has axis but `limits: None`; unsupported joint type (`floating`) errors clearly and names the three unsupported types; `cmd_idx` is `None` for fixed joints and contiguous-from-0 in DFS order for actuated joints; omitted `<origin>`/`<axis>` default correctly.

## Verification

- `cargo test` — the two new fixture-driven tests exercise the full pipeline against `k::Chain` as ground truth (per-link transform comparison), which is the strongest signal that both the joint-type math and the DFS ordering fix are correct.
- `cargo test --lib` (parser unit tests) for the narrower parsing-only cases.
- `cargo bench` (optional, user's call) to confirm the adjacency-map rewrite doesn't regress `fk_speed` — should improve or hold steady given O(V+E) vs O(V·E).
- Per your standing preference, I'll make the edits and explain them; you run `cargo build`/`cargo test`/`cargo bench` yourself.
