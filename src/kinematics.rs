use k::{Isometry3, Translation3, UnitQuaternion};

use crate::types::{RobotModel};

impl RobotModel {
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

        let mut links: Vec<Isometry3<f64>> = Vec::with_capacity(self.joints.len() + 1);
        links.push(Isometry3::identity());

        for (i, joint) in self.joints.iter().enumerate() {
            // Extracting info about the joint command
            let joint_rotation = UnitQuaternion::from_axis_angle(&joint.axis, joint_cmds[i]);
            let joint_local = joint.transform * Isometry3::from_parts(Translation3::identity(), joint_rotation);
            links.push(links[i] * joint_local);
        }

        Ok(links)
    }
}
