use std::collections::HashMap;

use nalgebra::{Isometry3, Matrix6xX, Translation3, UnitQuaternion};

use crate::{
    error::{GalawError, KinematicsError},
    types::GalawModel,
};

impl GalawModel {
    /// Computes forward kinematics of a model.
    ///
    /// Returns each link's world-space pose as an `Isometry3<f64>`, indexed
    /// the same as [`GalawModel::links`]. `joint_cmds` must have length
    /// [`GalawModel::num_actuated_joints`].
    ///
    /// # Examples
    ///
    /// ```
    /// # fn main() -> Result<(), galaw::error::GalawError> {
    /// let model = galaw::load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;
    /// let poses = model.compute_fk(&vec![0.0; model.num_actuated_joints])?;
    /// assert_eq!(poses.len(), model.links.len());
    /// # Ok(())
    /// # }
    /// ```
    pub fn compute_fk(&self, joint_cmds: &[f64]) -> Result<Vec<Isometry3<f64>>, GalawError> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(KinematicsError::JointCmdLengthMismatch {
                num_actuated: self.num_actuated_joints,
                num_input: joint_cmds.len(),
            }
            .into());
        }

        let mut links: Vec<Isometry3<f64>> = vec![Isometry3::identity(); self.links.len()];

        for joint in &self.joints {
            let cmd = joint.cmd_idx.map(|idx| joint_cmds[idx]).unwrap_or(0.0);

            // Extracting rotation and translation components
            let rotation = match joint.rot_axis {
                Some(axis) => UnitQuaternion::from_axis_angle(&axis, cmd),
                None => UnitQuaternion::identity(),
            };
            let translation = match joint.lin_axis {
                Some(axis) => Translation3::from(axis.into_inner() * cmd),
                None => Translation3::identity(),
            };

            let joint_local = joint.transform * Isometry3::from_parts(translation, rotation);
            links[joint.child_link_idx] = links[joint.parent_link_idx] * joint_local;
        }

        Ok(links)
    }

    /// Computes Jacobian of a model
    pub fn compute_jacobian(
        &self,
        joint_cmds: &[f64],
        target_link_idx: usize,
    )   -> Result<Matrix6xX<f64>, GalawError> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(KinematicsError::JointCmdLengthMismatch { 
                num_actuated: self.num_actuated_joints, 
                num_input: joint_cmds.len() 
            }
            .into());
        }

        if target_link_idx >= self.links.len() {
            return Err(KinematicsError::LinkIdxOutOfBounds { 
                num_links: self.links.len(), 
                requested: target_link_idx, 
            }
            .into());
        }

        let child_to_joint: HashMap<usize, usize> = self
            .joints
            .iter()
            .enumerate()
            .map(|(joint_idx, joint)| (joint.child_link_idx, joint_idx))
            .collect();

        let mut current_link_idx = target_link_idx;
        while let Some(&joint_idx) = child_to_joint.get(&current_link_idx) {
            let joint = &self.joints[joint_idx];
            current_link_idx = joint.parent_link_idx;
            eprintln!("{}", joint.name)
        }

        Ok(Matrix6xX::zeros(self.num_actuated_joints))

    }
}
