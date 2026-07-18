use nalgebra::{Isometry3, Translation3, UnitQuaternion};

use crate::types::GalawModel;

impl GalawModel {
    /// Computes forward kinematics of a model
    pub fn compute_fk(
        &self,
        joint_cmds: &[f64],
    ) -> Result<Vec<Isometry3<f64>>, Box<dyn std::error::Error>> {
        if joint_cmds.len() != self.num_actuated_joints {
            return Err(format!(
                "expected {} joint_cmds, got {}",
                self.joints.len(),
                joint_cmds.len()
            )
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

            let joint_local =
                joint.transform * Isometry3::from_parts(translation, rotation);
            links[joint.child_link_idx] = links[joint.parent_link_idx] * joint_local;
        }

        Ok(links)
    }
}
