use taligalaw::load_urdf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_model = load_urdf("assets/simple_robot.urdf")?;

    // Information about the robot
    println!("robot name: {:?}", robot_model.name);
    println!("number of links: {:?}", robot_model.links.len());
    println!("number of joints: {:?}", robot_model.joints.len());

    for link in robot_model.links.iter() {
        println!("Link: {:?}", link.name);
    }

    for joint in robot_model.joints.iter() {
        println!("Joint: {:?}", joint.name);
    }

    // Test commands
    let joint_cmds = [0.0, 0.0];

    // Demo with taligalaw compute_fk
    match robot_model.compute_fk(&joint_cmds) {
        Ok(links) => {
            for link in robot_model.links.iter() {
                println!("{:?}, {:?}", link, links[link])
            }
        }
        Err(e) => eprintln!("Error: {}", e),
    };

    Ok(())
}
