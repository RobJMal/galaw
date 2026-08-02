use std::collections::HashMap;

use nalgebra::{Isometry3, Matrix6xX, Translation3, UnitQuaternion, Vector3, Vector6};

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

    /// Computes Jacobian of a model for a link.
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

        // Set to 0 so only joint ancestors contribute
        let mut jacobian = Matrix6xX::zeros(self.num_actuated_joints);

        // Setup tree-walking lookup
        let child_to_joint: HashMap<usize, usize> = self
            .joints
            .iter()
            .enumerate()
            .map(|(joint_idx, joint)| (joint.child_link_idx, joint_idx))
            .collect();

        let links = self.compute_fk(joint_cmds)?;
        let link_target_position = links[target_link_idx].translation;

        // Starting from target_link, work backwards into the root and 
        // identify the joints that affect the Jacobian. 
        let mut current_link_idx = target_link_idx;
        while let Some(&joint_idx) = child_to_joint.get(&current_link_idx) {
            let joint = &self.joints[joint_idx];

            if joint.cmd_idx.is_none() {
                current_link_idx = joint.parent_link_idx;
                continue;
            }

            // Position of the joint based after doing FK
            let joint_position = links[joint.child_link_idx].translation;
            // Direction of joint motion
            let local_axis = joint.rot_axis.or(joint.lin_axis).expect("actuated joint has an axis");
            let joint_motion_axis = (links[joint.child_link_idx].rotation * local_axis).into_inner();
            
            let (linear_vel, angular_vel) = if joint.rot_axis.is_some() {
                (
                    joint_motion_axis.cross(&(link_target_position.vector - joint_position.vector)),
                    joint_motion_axis,
                )
            } else {
                (joint_motion_axis, Vector3::zeros())
            };

            let cmd_idx = joint.cmd_idx.unwrap();
            jacobian.set_column(
                cmd_idx, 
                &Vector6::new(
                    linear_vel.x, linear_vel.y, linear_vel.z,
                    angular_vel.x, angular_vel.y, angular_vel.z)
            );

            current_link_idx = joint.parent_link_idx;
        }

        Ok(jacobian)

    }
}
