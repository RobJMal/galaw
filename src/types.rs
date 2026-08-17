use std::collections::HashMap;

use nalgebra::{Isometry3, Unit, Vector3};

/// A rigid body in the robot's kinematic tree.
#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub struct Link {
    /// The link's name, from the URDF's `<link name="...">` attribute.
    pub name: String,
}

/// Represents joint types found in URDFs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    /// Rotates around a fixed axis, bounded by `limit_lower`/`limit_upper`.
    Revolute,
    /// Translates along a fixed axis, bounded by `limit_lower`/`limit_upper`.
    Prismatic,
    /// Rigid connection; no relative motion between parent and child.
    Fixed,
    /// Rotates around a fixed axis, unbounded (no joint limits).
    Continuous,
}

impl std::str::FromStr for JointType {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "revolute" => Ok(JointType::Revolute),
            "prismatic" => Ok(JointType::Prismatic),
            "fixed" => Ok(JointType::Fixed),
            "continuous" => Ok(JointType::Continuous),
            other => Err(other.to_string()),
        }
    }
}

/// A connection between two [`Link`]s.
#[derive(Debug, Clone)]
pub struct Joint {
    /// The joint's name, from the URDF's `<joint name="...">` attribute.
    pub name: String,
    /// The kind of motion this joint allows.
    pub joint_type: JointType,
    /// Name of the parent link.
    pub parent: String,
    /// Index of the parent link in [`GalawModel::links`].
    pub parent_link_idx: usize,
    /// Name of the child link.
    pub child: String,
    /// Index of the child link in [`GalawModel::links`].
    pub child_link_idx: usize,
    /// Fixed offset from the parent link to this joint's origin, from the URDF's `<origin>`.
    pub transform: Isometry3<f64>,
    /// Translation axis, for [`JointType::Prismatic`] joints. `None` otherwise.
    pub lin_axis: Option<Unit<Vector3<f64>>>, // Option since Unit doesn't allow zero-vector
    /// Rotation axis, for [`JointType::Revolute`]/[`JointType::Continuous`] joints. `None` otherwise.
    pub rot_axis: Option<Unit<Vector3<f64>>>, // Option since Unit doesn't allow zero-vector
    /// Lower joint limit. `None` for joints without a limit.
    pub limit_lower: Option<f64>,
    /// Upper joint limit. `None` for joints without a limit.
    pub limit_upper: Option<f64>,
    /// Index into the `joint_cmds` slice passed to `compute_fk`. `None` for [`JointType::Fixed`] joints, which take no command.
    pub cmd_idx: Option<usize>,
}

/// A parsed robot model, ready for forward-kinematics computation.
#[derive(Debug)]
pub struct GalawModel {
    /// The robot's name, from the URDF's `<robot name="...">` attribute.
    pub name: String,
    /// All links in the robot, in URDF file declaration order.
    pub links: Vec<Link>,
    /// All joints in the robot, ordered via DFS pre-order from the root (see `resolve_joint_order`).
    pub joints: Vec<Joint>,
    /// Maps a link's name to its index in [`GalawModel::links`].
    pub link_name_to_idx: HashMap<String, usize>,
    /// Maps an actuated joint's name to its `cmd_idx` (its position in a `joint_cmds` slice).
    pub joint_name_to_idx: HashMap<String, usize>,
    /// Maps a link's index to index of joint that connectes it to parent (edge in tree)
    pub link_idx_to_parent_joint_idx: HashMap<usize, usize>,
    /// Number of actuated (non-`Fixed`) joints — the expected length of a `joint_cmds` slice.
    pub num_actuated_joints: usize,
}

impl GalawModel {
    /// Looks up a link's index in [`GalawModel::links`] by name.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn main() -> Result<(), galaw::error::GalawError> {
    /// let model = galaw::load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;
    /// assert!(model.get_link_idx("base_link").is_some());
    /// # Ok(())
    /// # }
    /// ```
    pub fn get_link_idx(&self, name: &str) -> Option<usize> {
        self.link_name_to_idx.get(name).copied()
    }

    /// Looks up an actuated joint's `cmd_idx` by name — its position in a `joint_cmds` slice.
    ///
    /// # Examples
    ///
    /// ```
    /// # fn main() -> Result<(), galaw::error::GalawError> {
    /// let model = galaw::load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;
    /// assert!(model.get_joint_idx("shoulder_joint").is_some());
    /// # Ok(())
    /// # }
    /// ```
    pub fn get_joint_idx(&self, name: &str) -> Option<usize> {
        self.joint_name_to_idx.get(name).copied()
    }
}
