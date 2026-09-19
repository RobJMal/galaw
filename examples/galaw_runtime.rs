use galaw::{error::GalawError, load_urdf, types::GalawModel};
use k::Isometry3;
use nalgebra::Matrix6xX;

fn main() -> Result<(), GalawError<f64>> {
    let model: GalawModel<f64> = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Command each actuated joint by name — never by assumed position.
    let mut joint_cmds = vec![0.0; model.num_actuated_joints];
    let shoulder_idx = model
        .get_joint_idx("shoulder_joint")
        .expect("shoulder_joint exists in URDF");
    let elbow_idx = model
        .get_joint_idx("elbow_joint")
        .expect("elbow_joint exists in URDF");
    let forearm_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");
    joint_cmds[shoulder_idx] = 0.5;
    joint_cmds[elbow_idx] = -0.3;

    // --- Forward kinematics ---
    let mut poses = vec![Isometry3::identity(); model.links.len()];
    model.compute_fk(&joint_cmds, &mut poses)?;
    println!("=== FK ===");
    println!("forearm pose: {:?}", poses[forearm_idx]);

    // --- Jacobian ---
    let mut jacobians = vec![Matrix6xX::zeros(model.num_actuated_joints); model.links.iter().len()];
    model.compute_link_jacobians(&joint_cmds, &mut jacobians)?;
    println!("\n=== Jacobian ===");
    println!("forearm jacobian:\n{}", jacobians[forearm_idx]);

    // --- Inverse kinematics ---
    // Use the FK result above as the target, then solve from a zero initial guess.
    let target_pose = poses[forearm_idx];
    let init_joint_cmds = vec![0.0; model.num_actuated_joints];
    let mut solved_joint_cmds = vec![0.0f64; model.num_actuated_joints];
    model.compute_ik(
        forearm_idx,
        &target_pose,
        &init_joint_cmds,
        &mut solved_joint_cmds,
    )?;
    let mut solved_poses = vec![Isometry3::identity(); model.links.len()];
    model.compute_fk(&solved_joint_cmds, &mut solved_poses)?;
    println!("\n=== IK ===");
    println!("target pose:  {:?}", target_pose);
    println!("solved cmds:  {:?}", solved_joint_cmds);
    println!("solved pose:  {:?}", solved_poses[forearm_idx]);

    Ok(())
}
