use galaw::{error::GalawError, generated::simple_arm_2dof, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    // Load the model only to resolve joint/link names to indices.
    // The generated functions themselves take fixed-size arrays and need no GalawModel.
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Command each actuated joint by name (array, not Vec, since the DOF count is known).
    let mut joint_cmds: [f64; 2] = [0.0; 2];
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
    let poses = simple_arm_2dof::compute_fk(&joint_cmds);
    println!("=== FK ===");
    println!("forearm pose: {:?}", poses[forearm_idx]);

    // --- Jacobian ---
    let jacobians = simple_arm_2dof::compute_link_jacobians(&joint_cmds);
    println!("\n=== Jacobian ===");
    println!("forearm jacobian:\n{}", jacobians[forearm_idx]);

    // --- Inverse kinematics ---
    // Use the FK result above as the target, then solve from a zero initial guess.
    let target_pose = poses[forearm_idx];
    let init_joint_cmds: [f64; 2] = [0.0; 2];
    let solved_joint_cmds =
        simple_arm_2dof::compute_ik(forearm_idx, &target_pose, &init_joint_cmds)?;
    println!("\n=== IK ===");
    println!("target pose:       {:?}", target_pose);
    println!("solved joint cmds: {:?}", solved_joint_cmds);

    Ok(())
}
