use galaw::{error::GalawError, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Command each actuated joint by name
    let mut joint_cmds = vec![0.0; model.num_actuated_joints];
    let shoulder_idx = model
        .get_joint_idx("shoulder_joint")
        .expect("shoulder_joint exists in URDF");
    let elbow_idx = model
        .get_joint_idx("elbow_joint")
        .expect("elbow_joint exists in URDF");
    joint_cmds[shoulder_idx] = 0.0;
    joint_cmds[elbow_idx] = 0.0;

    // Run Jacobian computation
    let forearm_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");
    let jacobian = model.compute_jacobian(&joint_cmds, forearm_idx)?;

    println!("jacobian:\n{}", jacobian);

    Ok(())
}
