use crate::types::{Position3D, Quaternion, RobotModel, Transform};

impl RobotModel {
    pub fn compute_fk(
        &self,
        angles: &[f64],
    ) -> Result<Vec<Transform>, Box<dyn std::error::Error>> {
        /* Computes forward kinematics of a model */
        if angles.len() != self.joints.len() {
            return Err(format!(
                "expected {} angles, got {}",
                self.joints.len(),
                angles.len()
            )
            .into());
        }

        let mut links: Vec<Transform> = Vec::new();
        let base_link_transform: Transform = Transform {
            position: Position3D {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion::identity(),
        };
        links.push(base_link_transform);

        for (i, joint) in self.joints.iter().enumerate() {
            // Extracting info about the joint command
            let joint_axis = &joint.axis;
            let joint_axis_norm: f64 =
                f64::sqrt(joint_axis.x.powi(2) + joint_axis.y.powi(2) + joint_axis.z.powi(2));
            let joint_axis_normalized = Position3D {
                x: joint_axis.x / joint_axis_norm,
                y: joint_axis.y / joint_axis_norm,
                z: joint_axis.z / joint_axis_norm,
            };

            let quat_joint: Quaternion = Quaternion {
                x: joint_axis_normalized.x * f64::sin(angles[i] / 2.0),
                y: joint_axis_normalized.y * f64::sin(angles[i] / 2.0),
                z: joint_axis_normalized.z * f64::sin(angles[i] / 2.0),
                w: f64::cos(angles[i] / 2.0),
            };

            let quat_local: Quaternion = joint.transform.orientation.multiply(&quat_joint);

            // let link_prev_quat = links[&self.links[i]].orientation;
            let link_prev_quat = links[i].orientation;
            let link_n_quat = link_prev_quat.multiply(&quat_local);

            let quat_joint_translation = Quaternion {
                x: joint.transform.position.x,
                y: joint.transform.position.y,
                z: joint.transform.position.z,
                w: 0.0,
            };

            let rot_transform = link_prev_quat
                .multiply(&quat_joint_translation.multiply(&link_prev_quat.inverse()));
            let rot_position = Position3D {
                x: rot_transform.x,
                y: rot_transform.y,
                z: rot_transform.z,
            };
            let link_n_position = links[i].position + rot_position;

            links.push(Transform { position: link_n_position, orientation: link_n_quat });
        }

        Ok(links)
    }
}
