use taligalaw::load_urdf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_model =  load_urdf("assets/simple_robot.urdf")?;
    
    println!("robot name: {:?}", robot_model.name);
    println!("number of links: {:?}", robot_model.links.len());
    println!("number of joints: {:?}", robot_model.joints.len());

    for link in robot_model.links.iter() {
        println!("Link: {:?}", link.name);
    }

    for joint in robot_model.joints.iter() {
        println!("Joint: {:?}", joint.name);
    }

    Ok(())
}
