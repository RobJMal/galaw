#![warn(missing_docs)]

//! `galaw` is a robot kinematics library.
//!
//! Two APIs: [`load_urdf`] + [`types::GalawModel::compute_fk`] work for any
//! URDF at runtime; the `codegen_fk` binary generates a fixed, faster
//! `compute_fk` per robot ahead of time (see [`generated`]).
//!
//! ```
//! # fn main() -> Result<(), galaw::error::GalawError> {
//! let model = galaw::load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;
//! let poses = model.compute_fk(&vec![0.0; model.num_actuated_joints])?;
//! # Ok(())
//! # }
//! ```

/// Error types returned by this crate's public functions.
pub mod error;
/// URDF fixtures shared by the benchmark suite and its chart generator.
pub mod fixtures;
/// Ahead-of-time generated `compute_fk` implementations, one per robot.
///
/// Machine-written by `codegen_fk` (see `scripts/codegen_all_urdfs.sh`) —
/// exempt from `missing_docs` since files are auto-generated.
#[allow(missing_docs)]
pub mod generated;
/// Forward-kinematics computation.
pub mod kinematics;
/// URDF parsing.
pub mod parser;
/// Core data types: links, joints, and the parsed robot model.
pub mod types;
/// Small parsing helpers shared across the crate.
pub mod utils;

/// Parses a URDF file into a [`types::GalawModel`]. See [`parser::load_urdf`].
pub use parser::load_urdf;
