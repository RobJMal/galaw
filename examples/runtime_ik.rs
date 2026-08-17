use galaw::{error::GalawError, load_urdf, types::GalawModel};

fn main() -> Result<(), GalawError> {
    let model: GalawModel = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    let shoulder_joint_idx = model
        .get_joint_idx("shoulder_joint")
        .expect("shoulder_joint exists in URDF");
    let elbow_joint_idx = model
        .get_joint_idx("elbow_joint")
        .expect("elbow_joint exists in URDF");
    let forearm_link_idx = model
        .get_link_idx("forearm")
        .expect("forearm link exists in URDF");

    // Known solution
    let mut known_joint_cmds = vec![0.0; model.num_actuated_joints];
    known_joint_cmds[shoulder_joint_idx] = 0.5;
    known_joint_cmds[elbow_joint_idx] = -0.3;
    let target_pose = model.compute_fk(&known_joint_cmds)?[forearm_link_idx];

    // Start IK from different guess
    let init_joint_cmds = vec![0.0; model.num_actuated_joints];
    let solved_joint_cmds = model.compute_ik(forearm_link_idx, &target_pose, &init_joint_cmds)?;

    let solved_pose = model.compute_fk(&solved_joint_cmds)?[forearm_link_idx];

    println!("target_pose: {:?}", target_pose);
    println!("solved cmds: {:?}", solved_joint_cmds);
    println!("solved pose: {:?}", solved_pose);

    Ok(())
}
