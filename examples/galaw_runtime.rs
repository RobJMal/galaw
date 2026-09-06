use galaw::{error::GalawError, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

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
    let poses = model.compute_fk(&joint_cmds)?;
    println!("=== FK ===");
    println!("forearm pose: {:?}", poses[forearm_idx]);

    // --- Jacobian ---
    let jacobians = model.compute_link_jacobians(&joint_cmds)?;
    println!("\n=== Jacobian ===");
    println!("forearm jacobian:\n{}", jacobians[forearm_idx]);

    // --- Inverse kinematics ---
    // Use the FK result above as the target, then solve from a zero initial guess.
    let target_pose = poses[forearm_idx];
    let init_joint_cmds = vec![0.0; model.num_actuated_joints];
    let solved_joint_cmds = model.compute_ik(forearm_idx, &target_pose, &init_joint_cmds)?;
    let solved_pose = model.compute_fk(&solved_joint_cmds)?[forearm_idx];
    println!("\n=== IK ===");
    println!("target pose:  {:?}", target_pose);
    println!("solved cmds:  {:?}", solved_joint_cmds);
    println!("solved pose:  {:?}", solved_pose);

    Ok(())
}
