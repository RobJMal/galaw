use core::f64;
use std::fs;

use taligalaw::{EulerRPY, Joint, Position3D, Transform};

fn parse_vec3_str(input_str: &str) -> Result<(f64, f64, f64), Box<dyn std::error::Error>> {
    // Parses and extracts values from string. Assumes will contain 3 values.
    let vals: Vec<f64> = input_str
        .split_whitespace()
        .map(|n| n.parse::<f64>())
        .collect::<Result<Vec<f64>, _>>()?;

    if vals.len() != 3 {
        return Err("expected exactly 3 values".into());
    }

    Ok((vals[0], vals[1], vals[2]))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let content: String = fs::read_to_string("assets/simple_robot.urdf")?;
    let doc = roxmltree::Document::parse(&content)?;

    for node in doc.descendants() {
        if node.tag_name().name() == "joint" {
            let name: String = node
                .attribute("name")
                .ok_or("joint missing name attribute")?
                .to_string();
            let parent: String = node
                .children()
                .find(|n| n.tag_name().name() == "parent")
                .and_then(|n| n.attribute("link"))
                .ok_or("missing parent")?
                .to_string();
            let child: String = node
                .children()
                .find(|n| n.tag_name().name() == "child")
                .and_then(|n| n.attribute("link"))
                .ok_or("missing child")?
                .to_string();

            // Extracting joint XYZ and RPY
            let joint_origin = node
                                                .children()
                                                .find(|n| n.tag_name().name() == "origin")
                                                .ok_or("missing origin")?;
            
            let xyz_str: &str = joint_origin
                                .attribute("xyz")
                                .ok_or("missing XYZ")?;
            let (x, y, z) = parse_vec3_str(xyz_str)?;
            let xyz: Position3D = Position3D { x, y, z };
                                                
            let rpy_str = joint_origin
                                        .attribute("rpy")
                                        .ok_or("missing RPY")?;
            let (roll, pitch, yaw) = parse_vec3_str(rpy_str)?;
            let rpy: EulerRPY = EulerRPY { roll, pitch, yaw };

            let transform: Transform = Transform { position: xyz, orientation: rpy.to_quat() };
            
            // Extracting axis angles 
            let axis_str: &str = node
                .children()
                .find(|n| n.tag_name().name() == "axis")
                .and_then(|n| n.attribute("xyz"))
                .ok_or("missing xyz value for axis")?;
            let (axis_x, axis_y, axis_z) = parse_vec3_str(axis_str)?;
            let axis: Position3D = Position3D { x: axis_x, y: axis_y, z: axis_z };
            
            // Extracting joint limits
            let joint_limit = node
                                            .children()
                                            .find(|n| n.tag_name().name() == "limit")
                                            .ok_or("missing joint limits")?;

            let limit_lower: f64 = joint_limit
                                    .attribute("lower")
                                    .ok_or("missing joint limit lower")?
                                    .parse::<f64>()?;
            let limit_upper: f64 = joint_limit
                                    .attribute("upper")
                                    .ok_or("missing joint limit upper")?
                                    .parse::<f64>()?;
            
            // Creating joint
            let joint: Joint = Joint { name, parent, child, transform, axis, limit_lower, limit_upper };

            println!("joint: {:?}", joint);
            println!("");
        }
    }

    Ok(())
}
