use galaw::load_urdf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let galaw_model = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf")?;

    // Information about the robot
    println!("robot name: {:?}", galaw_model.name);
    println!("number of links: {:?}", galaw_model.links.len());
    println!("number of joints: {:?}", galaw_model.joints.len());

    for link in galaw_model.links.iter() {
        println!("Link: {:?}", link.name);
    }

    for joint in galaw_model.joints.iter() {
        let look_up = galaw_model.get_joint_idx(&joint.name);
        println!(
            "Joint: {:?} -> cmd_idx {:?} (lookup: {:?})",
            joint.name, joint.cmd_idx, look_up
        );
    }

    // Building joint_cmd
    let mut joint_cmds = vec![0.0; galaw_model.num_actuated_joints];
    let shoulder_joint_idx = galaw_model
        .get_joint_idx("shoulder_joint")
        .ok_or("no shoulder_joint")?;
    let elbow_joint_idx = galaw_model
        .get_joint_idx("elbow_joint")
        .ok_or("no elbow_joint")?;
    joint_cmds[shoulder_joint_idx] = 0.5;
    joint_cmds[elbow_joint_idx] = -0.3;

    // Demo with galaw compute_fk
    match galaw_model.compute_fk(&joint_cmds) {
        Ok(links) => {
            for (i, link) in galaw_model.links.iter().enumerate() {
                println!("{:?}, {:?}", link, links[i])
            }
            // Look up a specific link's pose by name, not by assumed position
            let forearm_idx = galaw_model
                .get_link_idx("forearm")
                .ok_or("no forearm link")?;
            println!("forearm pose (via get_link_idx): {:?}", links[forearm_idx]);
        }
        Err(e) => eprintln!("Error: {}", e),
    };

    Ok(())
}
