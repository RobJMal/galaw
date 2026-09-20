use galaw::{error::GalawError, load_urdf, types::GalawData, types::GalawModel};

fn main() -> Result<(), GalawError<f64>> {
    let model: GalawModel<f64> = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf")?;
    let mut data: GalawData<f64> = model.create_galaw_data();

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
    model.compute_fk(&joint_cmds, &mut data)?;
    println!("=== FK ===");
    println!("forearm pose: {:?}", data.link_poses[forearm_idx]);

    // --- Jacobian ---
    model.compute_link_jacobians(&joint_cmds, &mut data)?;
    println!("\n=== Jacobian ===");
    println!("forearm jacobian:\n{}", data.link_jacobians[forearm_idx]);

    // --- Inverse kinematics ---
    // Use the FK result above as the target, then solve from a zero initial guess.
    let target_pose = data.link_poses[forearm_idx];
    let init_joint_cmds = vec![0.0; model.num_actuated_joints];
    model.compute_ik(forearm_idx, &target_pose, &init_joint_cmds, &mut data)?;
    let solved_joint_cmds = data.solved_joint_cmds.clone();
    model.compute_fk(&solved_joint_cmds, &mut data)?;
    println!("\n=== IK ===");
    println!("target pose:  {:?}", target_pose);
    println!("solved cmds:  {:?}", data.solved_joint_cmds);
    println!("solved pose:  {:?}", data.link_poses[forearm_idx]);

    Ok(())
}
