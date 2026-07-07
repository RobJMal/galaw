use taligalaw::load_urdf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let robot_model =  load_urdf("assets/simple_robot.urdf")?;
    
    println!("robot name: {:?}", robot_model.name);
    println!("number of joints: {:?}", robot_model.joints.len());

    Ok(())
}
