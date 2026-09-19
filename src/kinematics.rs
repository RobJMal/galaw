use nalgebra::{
    DVector, Isometry3, Matrix6, Matrix6xX, RealField, Translation3, UnitQuaternion, Vector3,
    Vector6,
};

use crate::{
    error::{GalawError, KinematicsError},
    types::GalawModel,
};

impl<T: RealField + Copy> GalawModel<T> {
    /// Fills columns of `jacobian` for each joint in `ancestors`.
    #[inline]
    fn fill_jacobian_columns(
        &self,
        links: &[Isometry3<T>],
        ancestors: &[usize],
        target_position: &Translation3<T>,
        jacobian: &mut Matrix6xX<T>,
    ) {
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
                    joint_motion_axis.cross(&(target_position.vector - joint_position.vector)),
                    joint_motion_axis,
                )
            } else {
                (joint_motion_axis, Vector3::zeros())
            };

            jacobian.set_column(
                cmd_idx,
                &Vector6::new(
                    lin_vel.x, lin_vel.y, lin_vel.z, ang_vel.x, ang_vel.y, ang_vel.z,
                ),
            );
        }
    }

    /// Computes forward kinematics of a model.
    ///
    /// Computes forward kinematics of a model, writing each link's world-space
    /// pose into `poses`. `poses` must have length [`GalawModel::links`]`.len()`.
    /// `joint_cmds` must have length [`GalawModel::num_actuated_joints`].
    ///
    /// # Examples
    ///
    /// ```
    /// # fn main() -> Result<(), galaw::error::GalawError<f64>> {
    /// use nalgebra::Isometry3;
    /// let model = galaw::load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf")?;
    /// let mut poses = vec![Isometry3::identity(); model.links.len()];
    /// model.compute_fk(&vec![0.0; model.num_actuated_joints], &mut poses)?;
    /// assert_eq!(poses.len(), model.links.len());
    /// # Ok(())
    /// # }
    /// ```
    pub fn compute_fk(&self, joint_cmds: &[T], poses: &mut [Isometry3<T>]) -> Result<(), GalawError<T>> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(KinematicsError::JointCmdLengthMismatch {
                num_actuated: self.num_actuated_joints,
                num_input: joint_cmds.len(),
            }
            .into());
        }

        // Root link is identity since it serves as global base frame and is not
        // set by the joint loop. Therefore, it needs to be set. 
        let root_idx = self.joints.first().map_or(0, |j| j.parent_link_idx);
        poses[root_idx] = Isometry3::identity();

        for joint in &self.joints {
            let cmd = joint
                .cmd_idx
                .map(|idx| joint_cmds[idx])
                .unwrap_or(T::zero());

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
            poses[joint.child_link_idx] = poses[joint.parent_link_idx] * joint_local;
        }

        Ok(())
    }

    /// Computes the Jacobian of every link in a model.
    ///
    /// `jacobians` must have length [`GalawModel::links`]`.len()`, with each matrix
    /// pre-allocated to `6 × num_actuated_joints` and pre-zeroed. Only ancestor
    /// columns are written per link, so a buffer initialized with
    /// `Matrix6xX::zeros(num_actuated_joints)` can be reused across calls without
    /// re-zeroing, provided the model topology is unchanged.
    pub fn compute_link_jacobians(
        &self,
        joint_cmds: &[T],
        jacobians: &mut [Matrix6xX<T>],
    ) -> Result<(), GalawError<T>> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(KinematicsError::JointCmdLengthMismatch {
                num_actuated: self.num_actuated_joints,
                num_input: joint_cmds.len(),
            }
            .into());
        }
        if jacobians.len() != self.links.len() {
            return Err(KinematicsError::OutLengthMismatch {
                expected: self.links.len(),
                actual: jacobians.len(),
            }
            .into());
        }

        let mut links = vec![Isometry3::identity(); self.links.len()];
        self.compute_fk(joint_cmds, &mut links)?;

        for (link_idx, ancestors) in self.ancestors_by_link.iter().enumerate() {
            self.fill_jacobian_columns(
                &links,
                ancestors,
                &links[link_idx].translation,
                &mut jacobians[link_idx],
            );
        }

        Ok(())
    }

    /// Computes the Jacobian for a single link of a model.
    ///
    /// Primarily, this is used for computations where we only need specific links.
    /// `jacobian` must be pre-allocated to `6 × num_actuated_joints` and pre-zeroed.
    /// Only ancestor columns are written, so the buffer can be reused across calls.
    pub fn compute_link_jacobian(
        &self,
        joint_cmds: &[T],
        target_link_idx: usize,
        jacobian: &mut Matrix6xX<T>,
    ) -> Result<(), GalawError<T>> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(KinematicsError::JointCmdLengthMismatch {
                num_actuated: self.num_actuated_joints,
                num_input: joint_cmds.len(),
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

        let mut links = vec![Isometry3::identity(); self.links.len()];
        self.compute_fk(joint_cmds, &mut links)?;
        self.fill_jacobian_columns(
            &links,
            &self.ancestors_by_link[target_link_idx],
            &links[target_link_idx].translation,
            jacobian,
        );

        Ok(())
    }

    /// Computes the pose of one link along a precomputed chain, and fills
    /// `jacobian` with that link's Jacobian in place.
    ///
    /// Primarily, this is used by `compute_ik`'s loop, which needs both values
    /// every iteration without recomputing the chain or walking the whole model.
    #[inline]
    fn compute_restricted_pose_and_fill_jacobian(
        &self,
        chain: &[usize],
        joint_cmds: &[T],
        chain_poses: &mut Vec<Isometry3<T>>,
        jacobian: &mut Matrix6xX<T>,
    ) -> Isometry3<T> {
        chain_poses.clear();
        let mut pose = Isometry3::identity();

        for &joint_idx in chain {
            let joint = &self.joints[joint_idx];
            let cmd = joint
                .cmd_idx
                .map(|idx| joint_cmds[idx])
                .unwrap_or(T::zero());

            let rotation = match joint.rot_axis {
                Some(axis) => UnitQuaternion::from_axis_angle(&axis, cmd),
                None => UnitQuaternion::identity(),
            };
            let translation = match joint.lin_axis {
                Some(axis) => Translation3::from(axis.into_inner() * cmd),
                None => Translation3::identity(),
            };

            let joint_local = joint.transform * Isometry3::from_parts(translation, rotation);
            pose *= joint_local;
            chain_poses.push(pose);
        }

        let target_position = pose.translation;
        jacobian.fill(T::zero());

        for (i, &joint_idx) in chain.iter().enumerate() {
            let joint = &self.joints[joint_idx];
            let Some(cmd_idx) = joint.cmd_idx else {
                continue;
            };

            let joint_position = chain_poses[i].translation;
            let local_axis = joint
                .rot_axis
                .or(joint.lin_axis)
                .expect("actuated joint has an axis");
            let joint_motion_axis = (chain_poses[i].rotation * local_axis).into_inner();

            let (lin_vel, ang_vel) = if joint.rot_axis.is_some() {
                (
                    joint_motion_axis.cross(&(target_position.vector - joint_position.vector)),
                    joint_motion_axis,
                )
            } else {
                (joint_motion_axis, Vector3::zeros())
            };

            jacobian.set_column(
                cmd_idx,
                &Vector6::new(
                    lin_vel.x, lin_vel.y, lin_vel.z, ang_vel.x, ang_vel.y, ang_vel.z,
                ),
            );
        }

        pose
    }

    /// Computes inverse kinematics of a model.
    pub fn compute_ik(
        &self,
        target_link_idx: usize,
        target_pose: &Isometry3<T>,
        initial_joint_cmds: &[T],
    ) -> Result<Vec<T>, GalawError<T>> {
        // IK solver params
        let error_tolerance: T = nalgebra::convert(1e-5_f64);
        let damping_factor: T = nalgebra::convert(1e-4_f64);
        let step_size: T = nalgebra::convert(1.0_f64);
        const MAX_ITERATIONS: usize = 1000;

        let chain = &self.chain_by_link[target_link_idx];

        // Helper to compute pose error
        let compute_error = |current_pose: &Isometry3<T>| -> Result<Vector6<T>, GalawError<T>> {
            let error_position = target_pose.translation.vector - current_pose.translation.vector;
            let rotation_error = target_pose.rotation * current_pose.rotation.inverse();
            let error_rotation = rotation_error.scaled_axis();
            Ok(Vector6::new(
                error_position.x,
                error_position.y,
                error_position.z,
                error_rotation.x,
                error_rotation.y,
                error_rotation.z,
            ))
        };

        let mut joint_cmds_candidate = initial_joint_cmds.to_vec();
        let mut chain_poses: Vec<Isometry3<T>> = Vec::with_capacity(chain.len());
        let damping_matrix = Matrix6::<T>::identity() * damping_factor;
        let mut jac: Matrix6xX<T> = Matrix6xX::zeros(self.num_actuated_joints);
        let mut dq: DVector<T> = DVector::zeros(self.num_actuated_joints);

        let mut current_pose = self.compute_restricted_pose_and_fill_jacobian(
            chain,
            &joint_cmds_candidate,
            &mut chain_poses,
            &mut jac,
        );
        let mut error = compute_error(&current_pose)?;
        let mut iterations: usize = 0;

        // Applies the Levenberg-Marquardt approach
        while error.norm() > error_tolerance {
            if iterations >= MAX_ITERATIONS {
                return Err(KinematicsError::IkDidNotConverge {
                    iterations,
                    final_error: error.norm(),
                }
                .into());
            }

            let jjt_damped = &jac * jac.transpose() + damping_matrix;
            let x = jjt_damped
                .cholesky()
                .expect("J*J^T + damping*I is always positive definite for damping > 0")
                .solve(&error);
            jac.tr_mul_to(&x, &mut dq);
            for (q, dq_i) in joint_cmds_candidate.iter_mut().zip(dq.iter()) {
                *q += step_size * *dq_i;
            }

            current_pose = self.compute_restricted_pose_and_fill_jacobian(
                chain,
                &joint_cmds_candidate,
                &mut chain_poses,
                &mut jac,
            );
            error = compute_error(&current_pose)?;
            iterations += 1;
        }

        // Clamped converged solution to joint limits
        // No null-space steering
        for (i, cmd) in joint_cmds_candidate.iter_mut().enumerate() {
            *cmd = cmd.clamp(self.joint_limit_lower[i], self.joint_limit_upper[i]);
        }

        let clamped_pose = self.compute_restricted_pose_and_fill_jacobian(
            chain,
            &joint_cmds_candidate,
            &mut chain_poses,
            &mut jac,
        );
        let clamped_error = compute_error(&clamped_pose)?;
        if clamped_error.norm() > error_tolerance {
            return Err(KinematicsError::IkDidNotConverge {
                iterations,
                final_error: clamped_error.norm(),
            }
            .into());
        }

        Ok(joint_cmds_candidate)
    }
}
