# Generated Jacobian Optimization (Future Work)

## Background

The generated kinematics path uses `GeneratedGalawData<T, const NUM_JOINTS, const NUM_LINKS>` with
stack-allocated, fixed-size arrays. For FK and IK, the generated path is uniformly faster than the
runtime path. For Jacobians, it is only faster for small robots and slower for large ones.

Benchmark data (geometric mean across 4 robots, 100-pose batch, µs):

| Operation     | galaw-runtime | galaw-generated |
|---------------|--------------|-----------------|
| FK            | 70           | 29              |
| Jacobian      | 144          | 152             |
| IK            | 608          | 269             |

Per-robot Jacobian breakdown (median, µs):

| Robot         | Links | galaw-runtime | galaw-generated | Winner  |
|---------------|-------|--------------|-----------------|---------|
| Enlight-L     | ~10   | 56           | 35              | generated |
| wujihand-right| ~26   | 126          | 108             | generated |
| stretch4      | ~54   | 219          | 273             | runtime |
| anymal-D      | ~96   | 278          | 512             | runtime |

The crossover is between 26 and 54 links.

## Root Causes

### Small robots: LLVM SSA regression

The old generated Jacobian (before the `GeneratedGalawData` refactor) called FK into a **local stack
array**:

```rust
let mut links = [Isometry3::identity(); M];
compute_fk(joint_cmds, &mut links);
let axis_world_1 = links[2].rotation * ...;
```

The current generated Jacobian calls FK through the external `data` pointer:

```rust
compute_fk(joint_cmds, data);
let axis_world_1 = data.link_poses[2].rotation * ...;
```

With a local array, LLVM can represent all FK results as SSA values after inlining — no memory
round-trips for the subsequent Jacobian reads. With an external pointer, LLVM must emit actual
memory stores to `data.link_poses` (because the caller can observe them), then reload from memory.

For small robots where the FK result fits in or near the register file (~10 links × 56 bytes = 560
bytes), the register/memory difference is measurable. For large robots, the data spills regardless.

Concretely, for Enlight-L, the current generated Jacobian is ~42% slower vs the pre-`GeneratedGalawData`
baseline, while the runtime path shows no regression.

### Large robots: instruction cache pressure

The generated Jacobian fully unrolls all link Jacobians into a single function. For anymal-D
(96 links, 14 actuated joints), this produces ~96 separate code blocks, each initializing a local
`SMatrix<f64, 6, 14>`, filling up to 14 columns, and assigning. The resulting function exceeds
the L1 instruction cache (32 KB on the i7-10750H), causing I-cache thrashing on every call.

The runtime Jacobian uses a compact loop (`fill_jacobian_columns`) that stays resident in L1-I
regardless of robot size.

## Proposed Solutions

### Option 1: Local FK buffer in `compute_link_jacobians` (low complexity)

Generate a private `compute_fk_impl` that writes to `&mut [Isometry3<T>; M]`, then have the
public `compute_fk` and `compute_link_jacobians` use it differently:

```rust
#[inline]
fn compute_fk_impl(joint_cmds: &[f64; N], poses: &mut [Isometry3<f64>; M]) {
    // ... actual FK logic ...
}

pub fn compute_fk(joint_cmds: &[f64; N], data: &mut GeneratedGalawData<f64, N, M>) {
    compute_fk_impl(joint_cmds, &mut data.link_poses);
}

pub fn compute_link_jacobians(joint_cmds: &[f64; N], data: &mut GeneratedGalawData<f64, N, M>) {
    let mut links = [Isometry3::identity(); M];  // local — LLVM keeps in SSA/registers
    compute_fk_impl(joint_cmds, &mut links);
    // ... Jacobian computation using links[i] ...
    data.link_poses = links;  // cheap memcopy at the end (560 bytes for Enlight-L)
}
```

**Effect:**
- Fixes the Enlight-L ~42% regression vs baseline.
- Negligible overhead for large robots (the final memcopy is 5.4 KB for anymal-D).
- Does not fix the large-robot I-cache problem.

**Complexity:** Low — localized change to `generate_fk_fn_code` and `generate_jacobian_fn_code`.

### Option 2: Loop-based Jacobian with static ancestor tables (higher complexity)

Instead of unrolling all link Jacobians, emit a `const` ancestor table and a compact loop:

```rust
const ANCESTOR_TABLE: [&[usize]; NUM_LINKS] = [&[], &[0], &[0, 1], ...];
const JOINT_CHILD_LINK: [usize; TOTAL_JOINTS] = [...];
const JOINT_AXIS: [[f64; 3]; TOTAL_JOINTS] = [...];
const JOINT_IS_ROT: [bool; TOTAL_JOINTS] = [...];

pub fn compute_link_jacobians(joint_cmds: &[f64; N], data: &mut GeneratedGalawData<f64, N, M>) {
    compute_fk(joint_cmds, data);
    for link_idx in 0..NUM_LINKS {
        if ANCESTOR_TABLE[link_idx].is_empty() {
            data.link_jacobians[link_idx].fill(0.0);
            continue;
        }
        let target_pos = data.link_poses[link_idx].translation;
        let mut jac = SMatrix::<f64, 6, N>::zeros();
        for &joint_idx in ANCESTOR_TABLE[link_idx] {
            // cross product using static axis data and data.link_poses
            jac.set_column(...);
        }
        data.link_jacobians[link_idx] = jac;
    }
}
```

**Effect:**
- Fixes the large-robot I-cache problem: the function is compact regardless of robot size.
- The `const` tables live in `.rodata` (always warm in cache, no heap pointer chasing vs
  the runtime's `Vec<Vec<usize>>` ancestor data).
- For small robots (≤~26 links), LLVM may still unroll the outer loop, recovering most of
  the optimization. Net effect vs current: roughly neutral to slightly better.
- For large robots, likely matches or beats the runtime Jacobian.

**Complexity:** Medium — codegen emits static tables rather than per-link code blocks. Also
reduces compile time and binary size for large robots significantly.

### Recommended approach: Option 1 + Option 2

Combining both gives consistent wins across all sizes:
- Option 1 restores the LLVM SSA optimization for FK results in the Jacobian pass.
- Option 2 keeps the function compact for I-cache friendliness at all sizes.

Expected outcome: generated Jacobian beats runtime for all tested robots, instead of the current
split where it wins on small robots and loses on large ones.

## Related Context

- The generated FK and IK already beat runtime uniformly. FK wins because joint transforms are
  baked as constants, eliminating loop + dispatch overhead. IK wins because heap allocations
  inside the iterative solver (`DVector`, `Matrix6xX`, `Vec`) are replaced with stack-allocated
  `SMatrix`/`SVector`.
- The Jacobian doesn't get the same structural simplification: the per-ancestor cross-product
  is identical whether generated or runtime. The only structural benefit of the generated path
  (knowing ancestors at compile time) is not enough to overcome the unrolling penalty for
  large robots.
