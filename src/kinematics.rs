use nalgebra::{Isometry3, Matrix6, Matrix6xX, Translation3, UnitQuaternion, Vector3, Vector6};

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

    /// Computes the Jacobian of every link in a model.
    pub fn compute_link_jacobians(&self, joint_cmds: &[f64]) -> Result<Vec<Matrix6xX<f64>>, GalawError> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(KinematicsError::JointCmdLengthMismatch {
                num_actuated: self.num_actuated_joints,
                num_input: joint_cmds.len(),
            }
            .into());
        }

        // Set to 0 so only joint ancestors contribute
        let mut jacobians: Vec<Matrix6xX<f64>> = (0..self.links.len())
            .map(|_| Matrix6xX::zeros(self.num_actuated_joints))
            .collect();

        // Construct each links' ancestor joints
        let mut ancestors_by_link: Vec<Vec<usize>> = vec![Vec::new(); self.links.len()];
        for (joint_idx, joint) in self.joints.iter().enumerate() {
            let mut ancestors = ancestors_by_link[joint.parent_link_idx].clone();
            if joint.cmd_idx.is_some() {
                ancestors.push(joint_idx);
            }
            ancestors_by_link[joint.child_link_idx] = ancestors;
        }

        let links = self.compute_fk(joint_cmds)?;

        for (link_idx, ancestors) in ancestors_by_link.iter().enumerate() {
            let joint_position_target = links[link_idx].translation;

            for &joint_idx in ancestors {
                let joint = &self.joints[joint_idx];
                let cmd_idx = joint.cmd_idx.unwrap();

                let joint_position = links[joint.child_link_idx].translation;
                let local_axis = joint
                    .rot_axis
                    .or(joint.lin_axis)
                    .expect("actuated joint has an axis");
                let joint_motion_axis =
                    (links[joint.child_link_idx].rotation * local_axis).into_inner();

                let (lin_vel, ang_vel) = if joint.rot_axis.is_some() {
                    (
                        joint_motion_axis
                            .cross(&(joint_position_target.vector - joint_position.vector)),
                        joint_motion_axis,
                    )
                } else {
                    (joint_motion_axis, Vector3::zeros())
                };

                jacobians[link_idx].set_column(
                    cmd_idx,
                    &Vector6::new(
                        lin_vel.x, lin_vel.y, lin_vel.z, ang_vel.x, ang_vel.y, ang_vel.z,
                    ),
                );
            }
        }

        Ok(jacobians)
    }

    /// Computes inverse kinematics of a model.
    pub fn compute_ik(&self, 
        target_link_idx: usize, 
        target_pose: &Isometry3<f64>, 
        initial_joint_cmds: &[f64],
    ) -> Result<Vec<f64>, GalawError> {

        // IK solver params
        const ERROR_TOLERANCE: f64 = 1e-5; 
        const DAMPING_FACTOR: f64 = 1e-4;
        const STEP_SIZE: f64 = 1.0;
        const MAX_ITERATIONS: usize = 1000;

        // Helper to compute error
        let compute_error = |joint_cmds: &[f64]| -> Result<Vector6<f64>, GalawError> {
            let current_pose = self.compute_fk(joint_cmds)?[target_link_idx];
            let error_position = target_pose.translation.vector - current_pose.translation.vector;
            let rotation_error = target_pose.rotation * current_pose.rotation.inverse();
            let error_rotation = rotation_error.scaled_axis();
            Ok(Vector6::new(
                error_position.x, error_position.y, error_position.z, 
                error_rotation.x, error_rotation.y, error_rotation.z,
            ))
        };
        
        let mut joint_cmds_candidate = initial_joint_cmds.to_vec(); 
        let mut error = compute_error(&joint_cmds_candidate)?;
        let mut iterations: usize = 0;

        // Applies the Levenberg-Marquardt approach
        while error.norm() > ERROR_TOLERANCE {
            if iterations >= MAX_ITERATIONS {
                return Err(KinematicsError::IkDidNotConverge { 
                    iterations, 
                    final_error: error.norm(), 
                }
                .into());
            }

            let jac = self.compute_link_jacobians(&joint_cmds_candidate)?[target_link_idx].clone();
            let jjt_damped = &jac * jac.transpose() + DAMPING_FACTOR * Matrix6::identity();
            let x = jjt_damped
                .cholesky()
                .expect("J*J^T + damping*I is always positive definite for damping > 0")
                .solve(&error);
            let dq = jac.transpose() * x;
            for (q, dq_i) in joint_cmds_candidate.iter_mut().zip(dq.iter()) {
                *q += STEP_SIZE * dq_i;
            }

            error = compute_error(&joint_cmds_candidate)?;
            iterations += 1;
        }

        Ok(joint_cmds_candidate)
    }
}
