use galaw::{
    load_urdf,
    error::GalawError,  
    types::GalawModel
};

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
    joint_cmds[shoulder_idx] = 0.5;
    joint_cmds[elbow_idx] = -0.3;

    // Run FK computation
    let poses = model.compute_fk(&joint_cmds)?;

    // Extract pose of a link
    let forearm_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");
    println!("forearm pose: {:?}", poses[forearm_idx]);

    Ok(())
}
