use galaw::{error::GalawError, generated::simple_arm_2dof, load_urdf, types::GalawModel};
use nalgebra::{Isometry3, SMatrix};

fn main() -> Result<(), GalawError<f64>> {
    // Load the model only to resolve joint/link names to indices.
    // The generated functions themselves take fixed-size arrays and need no GalawModel.
    let model: GalawModel<f64> = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Constants of the model since we know them ahead of time
    const NUM_JOINTS: usize = 2;
    const NUM_LINKS: usize = 3;

    // Command each actuated joint by name (array, not Vec, since the DOF count is known).
    let mut joint_cmds: [f64; 2] = [0.0; NUM_JOINTS];
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
    let mut poses = [Isometry3::identity(); NUM_LINKS];
    simple_arm_2dof::compute_fk(&joint_cmds, &mut poses);
    println!("=== FK ===");
    println!("forearm pose: {:?}", poses[forearm_idx]);

    // --- Jacobian ---
    let mut jacobians: [SMatrix<f64, 6, NUM_JOINTS>; NUM_LINKS] =
        std::array::from_fn(|_| SMatrix::zeros());
    simple_arm_2dof::compute_link_jacobians(&joint_cmds, &mut jacobians);
    println!("\n=== Jacobian ===");
    println!("forearm jacobian:\n{}", jacobians[forearm_idx]);

    // --- Inverse kinematics ---
    // Use the FK result above as the target, then solve from a zero initial guess.
    let target_pose = poses[forearm_idx];
    let init_joint_cmds: [f64; 2] = [0.0; 2];
    let mut solved_joint_cmds = [0.0f64; NUM_JOINTS];
    simple_arm_2dof::compute_ik(
        forearm_idx,
        &target_pose,
        &init_joint_cmds,
        &mut solved_joint_cmds,
    )?;
    println!("\n=== IK ===");
    println!("target pose:       {:?}", target_pose);
    println!("solved joint cmds: {:?}", solved_joint_cmds);

    Ok(())
}
