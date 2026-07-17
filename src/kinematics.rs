use k::{Isometry3, Translation3, UnitQuaternion};

use crate::types::GalawModel;

impl GalawModel {
    /// Computes forward kinematics of a model
    pub fn compute_fk(
        &self,
        joint_cmds: &[f64],
    ) -> Result<Vec<Isometry3<f64>>, Box<dyn std::error::Error>> {
        if joint_cmds.len() != self.joints.len() {
            return Err(format!(
                "expected {} joint_cmds, got {}",
                self.joints.len(),
                joint_cmds.len()
            )
            .into());
        }

        let mut links: Vec<Isometry3<f64>> = vec![Isometry3::identity(); self.links.len()];

        for joint in &self.joints {
            // Extracting info about the joint command
            let joint_rotation =
                UnitQuaternion::from_axis_angle(&joint.axis, joint_cmds[joint.cmd_idx]);
            let joint_local =
                joint.transform * Isometry3::from_parts(Translation3::identity(), joint_rotation);
            links[joint.child_link_idx] = links[joint.parent_link_idx] * joint_local;
        }

        Ok(links)
    }
}
