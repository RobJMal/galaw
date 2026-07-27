use std::collections::HashMap;

use nalgebra::{Isometry3, Unit, Vector3};

#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub struct Link {
    pub name: String,
}

/// Represents joint types found in URDFs
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Revolute,
    Prismatic,
    Fixed,
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

#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub joint_type: JointType,
    pub parent: String,
    pub parent_link_idx: usize,
    pub child: String,
    pub child_link_idx: usize,
    pub transform: Isometry3<f64>,
    pub lin_axis: Option<Unit<Vector3<f64>>>, // Option since Unit doesn't allow zero-vector
    pub rot_axis: Option<Unit<Vector3<f64>>>, // Option since Unit doesn't allow zero-vector
    pub limit_lower: Option<f64>,
    pub limit_upper: Option<f64>,
    pub cmd_idx: Option<usize>,
}

#[derive(Debug)]
pub struct GalawModel {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
    pub link_name_to_idx: HashMap<String, usize>,
    pub joint_name_to_idx: HashMap<String, usize>,
    pub num_actuated_joints: usize,
}

impl GalawModel {
    pub fn get_link_idx(&self, name: &str) -> Option<usize> {
        self.link_name_to_idx.get(name).copied()
    }

    pub fn get_joint_idx(&self, name: &str) -> Option<usize> {
        self.joint_name_to_idx.get(name).copied()
    }
}
