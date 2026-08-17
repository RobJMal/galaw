use galaw::{error::GalawError, generated::simple_arm_2dof, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Command each actuated joint by name (note: we pass
    // the array NOT vec since we know the params of the model)
    let mut joint_cmds: [f64; 2] = [0.0; 2];
    let shoulder_idx = model
        .get_joint_idx("shoulder_joint")
        .expect("shoulder_joint exists in URDF");
    let elbow_idx = model
        .get_joint_idx("elbow_joint")
        .expect("elbow_joint exists in URDF");
    joint_cmds[shoulder_idx] = 0.5;
    joint_cmds[elbow_idx] = -0.3;

    // Run Jacobian computation
    let forearm_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");
    let jacobian = simple_arm_2dof::compute_link_jacobians(&joint_cmds);

    println!("jacobian:\n{}", jacobian[forearm_idx]);

    Ok(())
}
