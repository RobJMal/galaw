#![warn(missing_docs)]

//! `galaw` is a robot kinematics library.
//!
//! Two APIs for FK, Jacobians, and IK: [`load_urdf`] + [`types::GalawModel`] work
//! for any URDF at runtime; the `codegen_kinematics` binary generates fixed,
//! faster implementations per robot ahead of time (see [`generated`]).
//!
//! Both APIs write results into caller-owned output buffers ([`types::GalawData`]
//! or [`types::GeneratedGalawData`]) to eliminate per-call heap allocation.
//!
//! ```
//! # fn main() -> Result<(), galaw::error::GalawError<f64>> {
//! let model = galaw::load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf")?;
//! let mut data = model.create_galaw_data();
//! model.compute_fk(&vec![0.0; model.num_actuated_joints], &mut data)?;
//! model.compute_link_jacobians(&vec![0.0; model.num_actuated_joints], &mut data)?;
//! # Ok(())
//! # }
//! ```

/// Error types returned by this crate's public functions.
pub mod error;
/// URDF fixtures shared by the benchmark suite and its chart generator.
#[doc(hidden)]
pub mod fixtures;
/// Ahead-of-time generated `compute_fk` implementations, one per robot.
///
/// Machine-written by `codegen_kinematics` (see `scripts/codegen_all_urdfs.sh`) —
/// exempt from `missing_docs` and `clippy` since files are auto-generated,
/// not hand-maintained.
#[allow(missing_docs)]
#[allow(clippy::all)]
pub mod generated;
/// Forward-kinematics computation.
pub mod kinematics;
pub(crate) mod parser;
/// Core data types: links, joints, and the parsed robot model.
pub mod types;
pub(crate) mod utils;

/// Parses a URDF file at `path` into a [`types::GalawModel`].
pub use parser::load_urdf;
