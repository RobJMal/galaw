use galaw::{error::GalawError, generated::simple_arm_2dof, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Command each actuated joint by name (note: we pass
    // the array NOT vec since we know the params of the model)
    let mut known_joint_cmds: [f64; 2] = [0.0; 2];
    let shoulder_idx = model
        .get_joint_idx("shoulder_joint")
        .expect("shoulder_joint exists in URDF");
    let elbow_idx = model
        .get_joint_idx("elbow_joint")
        .expect("elbow_joint exists in URDF");
    known_joint_cmds[shoulder_idx] = 0.5;
    known_joint_cmds[elbow_idx] = -0.3;

    // Run FK computation using codegenerated file to get a target pose
    let forearm_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");
    let target_pose = simple_arm_2dof::compute_fk(&known_joint_cmds)[forearm_idx];

    // Solve IK from a different initial guess using codegenerated file
    let init_joint_cmds: [f64; 2] = [0.0; 2];
    let solved_joint_cmds =
        simple_arm_2dof::compute_ik(forearm_idx, &target_pose, &init_joint_cmds)?;

    println!("target pose: {:?}", target_pose);
    println!("solved joint cmds: {:?}", solved_joint_cmds);

    Ok(())
}
