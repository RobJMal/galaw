use galaw::{
    load_urdf,
    error::GalawError, 
    generated::simple_arm_2dof,
    types::GalawModel,
};

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

    // Run FK computation using codegenerated file
    let poses = simple_arm_2dof::compute_fk(&joint_cmds);
    
    // Extract pose of a link
    let forearm_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");
    println!("forearm pose: {:?}", poses[forearm_idx]);

    Ok(())
}

