# galaw — crates.io Release Plan

Preparing `galaw` for a crates.io publish, now that the `codegen_fk` math
optimization work (see `docs/codegen_fk_math_optimization.md`) is done. Three
areas: packaging/metadata, error handling, and API documentation/examples.
Grounded in the actual repo state as of this writing, not generic advice.

## 1. crates.io packaging readiness

- **No LICENSE file, and no `license`/`license-file` in `Cargo.toml`.**
  `cargo publish` will reject the crate without one. `README.md` documents
  licenses for the *third-party URDF assets* (Apache-2.0, BSD-3-Clause, MIT,
  Clear BSD per-robot) but says nothing about galaw's own code — that's a
  decision only the maintainer can make. MIT, Apache-2.0, or dual
  MIT/Apache-2.0 (the Rust-ecosystem default) are the common choices.
- **`Cargo.toml` is missing every crates.io metadata field**: `description`,
  `license`, `repository`, `keywords`, `categories`, `readme`. `description`
  and `license`/`license-file` are hard requirements for `cargo publish`, not
  just nice-to-haves.
- **`assets/` is 38MB and nothing excludes it from the package.** Without an
  `exclude`/`include` in `Cargo.toml`, `cargo publish` bundles the whole
  thing — crates.io's default size limit is 10MB. The FK/parsing code only
  needs the `.urdf` XML files, not the mesh geometry (STL/DAE) sitting
  alongside them. Exclude mesh directories from the published tarball, or
  move them out of the crate entirely (dev-only fixtures via a git submodule
  or separate download, not shipped to consumers).
- **`nalgebra = "0.30.1"` is a tight pin on an old point release.** Check
  against current upstream before publishing — a consumer who already
  depends on a newer `nalgebra` in their own tree gets a duplicate-version
  conflict unless galaw's requirement is bumped or loosened to a wider range.
- **`edition = "2024"` with no `rust-version` field.** Edition 2024 needs a
  recent toolchain; without a documented MSRV, consumers on slightly older
  stable Rust get a confusing build failure instead of a clear "you need
  Rust X.Y+" message from Cargo itself.
- **No CI** — no `.github/workflows`. Add `cargo test` / `cargo clippy` /
  `cargo fmt --check` running on PRs before relying on external
  contributors, and run `cargo publish --dry-run` as the literal last
  pre-publish step regardless.
- **Two `[[bin]]` targets (`galaw`, `codegen_fk`) publish as installable
  binaries by default.** Decide deliberately: is `codegen_fk` meant to be a
  public-facing tool consumers `cargo install galaw` and run themselves
  (part of the "generation features" API), or an internal dev tool that
  should live under `examples/` instead so it compiles but isn't installed?
  Real API-scope decision, not just packaging cleanup.

## 2. Error messages — replace `Box<dyn Error>` + strings with a real error type

Every fallible function in `src/parser.rs` (and `src/kinematics.rs`) returns
`Result<T, Box<dyn std::error::Error>>`, with most errors built from
`format!(...)` strings via `.into()`/`ok_or_else`. Two concrete problems:

- **Consumers can't match on error kind** — only `.to_string()`, which makes
  error *text* a de facto API contract that can't safely reword later
  without breaking someone's string-matching code. Callers can't
  programmatically distinguish "file not found" from "malformed XML" from
  "missing joint attribute" from "disconnected URDF tree."
- **Some errors lose all context.** They're built from bare `&str`/`String`
  via `.into()`, which doesn't implement `source()` — the underlying error
  is erased instead of chained. Concrete example: `parser.rs:43` and `:46`,
  `.parse::<f64>()?` on a joint limit — if a URDF has `lower="not_a_number"`,
  the error a consumer sees is just `invalid float literal`, with **zero
  indication of which joint, which attribute, or which file** caused it.

### Fix: a `thiserror`-based error enum

`thiserror` is tiny (proc-macro only, no runtime deps) and is the standard
choice for *library* errors — as opposed to `anyhow`/`Box<dyn Error>`, which
are for *application* code. Keep the latter fine in `main.rs`/
`codegen_fk.rs` (they're binaries), but the library itself should expose
something matchable:

```rust
#[derive(Debug, thiserror::Error)]
pub enum GalawError {
    #[error("failed to read URDF file '{path}'")]
    Io { path: String, #[source] source: std::io::Error },

    #[error("failed to parse XML in '{path}'")]
    XmlParse { path: String, #[source] source: roxmltree::Error },

    #[error("joint '{joint}' is missing required attribute '{attribute}'")]
    MissingAttribute { joint: String, attribute: String },

    #[error("joint '{joint}' has invalid '{attribute}' value '{value}': {source}")]
    InvalidNumber {
        joint: String,
        attribute: String,
        value: String,
        #[source] source: std::num::ParseFloatError,
    },

    #[error("joint '{joint}' has unknown type '{found}' (expected one of: revolute, prismatic, fixed, continuous)")]
    UnknownJointType { joint: String, found: String },

    #[error("URDF has invalid topology: {0}")] // disconnected / cycle / multiple roots
    InvalidTopology(String),

    #[error("expected {expected} joint_cmds, got {actual}")]
    JointCmdLengthMismatch { expected: usize, actual: usize },
}
```

This is a mechanical rewrite of every `format!(...).into()` / `ok_or_else`
site in `parser.rs` and `kinematics.rs` into a specific variant — every
message keeps (or gains) the joint/attribute/file context it should have
had, every error becomes matchable, and `#[source]` keeps the original
`std::io::Error`/`roxmltree::Error`/`ParseFloatError` chained instead of
discarded.

## 3. API + examples

Two APIs exist today with the same shape but very different
performance/flexibility tradeoffs, and nothing currently explains that to a
reader — worth documenting explicitly, not just adding snippets:

- **Dynamic**: `load_urdf(path)? → GalawModel::compute_fk(&joint_cmds)` —
  works for *any* URDF at runtime, ~2-14x slower than the generated path
  (per this session's benchmarks).
- **Generated**: run the `codegen_fk` binary once per robot ahead of time,
  then call the free function `generated::<robot>::compute_fk(&joint_cmds)`
  — fixed at compile time to one robot, much faster, no `Result`/parsing at
  call time at all.

Concrete gaps:

- **README has zero usage content** — it's 100% third-party asset
  attributions right now. Needs a `## Quick start` with a copy-pasteable
  snippet for both APIs, and the tradeoff explanation above.
- **`examples/` only has `plot_bench.rs`** (a benchmark-charting dev tool,
  not a tutorial). `main.rs`'s existing demo (load URDF, look up a joint by
  name, compute FK, print poses) is solid content — promote it into a real
  `examples/basic_fk.rs`, plus a new `examples/codegen.rs` showing the
  ahead-of-time path specifically, since that's the half of the API nothing
  currently demonstrates end-to-end.
- **A crate-level doctest in `lib.rs`'s `//!` block** — the highest-visibility
  example (first thing shown on docs.rs), and doctests get compiled and run
  by `cargo test`, so it can never silently go stale the way a hand-written
  README snippet can.
- **`///` docs with `# Examples` on the actually-public surface**:
  `load_urdf`, `GalawModel::compute_fk`, `get_link_idx`/`get_joint_idx`, and
  each generated `compute_fk` (codegen could even emit a doc comment on the
  function it generates, tying back into the math-optimization PR's work).
- **Zero doc comments on any public item today.** `types.rs` — `Link`,
  `Joint`, `JointType`, `GalawModel`, every field — has no `///` anywhere,
  and `lib.rs` has no crate-level `//!`. docs.rs will render an essentially
  blank API reference. Turning on `#![warn(missing_docs)]` in `lib.rs`
  (temporarily even `deny`) is the fastest way to find every gap
  mechanically rather than auditing by hand.

## Suggested rollout order

1. **Error enum (`thiserror`)** — most mechanical, lowest-risk, and
   everything else reads better once error messages are actually good.
2. **Doc comments + `#![warn(missing_docs)]`** — mechanically finds every
   undocumented public item; write real docs for each as you go.
3. **README quick-start + `examples/basic_fk.rs` + `examples/codegen.rs`** —
   now backed by accurate error messages and doc comments to draw from.
4. **Cargo.toml metadata + LICENSE + package exclude/include + MSRV** —
   packaging cleanup, do last since it's independent of the code changes
   above and easy to get wrong if rushed.
5. **CI workflow + `cargo publish --dry-run`** — final gate before an actual
   publish.
