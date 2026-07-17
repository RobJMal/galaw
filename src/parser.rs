use std::fs;

// Third-party
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

// Custom
use crate::types::{Joint, Link, RobotModel};
use crate::utils::parse_vec3_str;


pub fn load_urdf(urdf_path: &str) -> Result<RobotModel, Box<dyn std::error::Error>> {
    let content: String = fs::read_to_string(urdf_path)?;
    let doc = roxmltree::Document::parse(&content)?;

    let robot_name: String = doc
        .root_element()
        .attribute("name")
        .ok_or("missing robot name")?
        .to_string();
    let mut links: Vec<Link> = Vec::new();
    let mut joints: Vec<Joint> = Vec::new();

    for node in doc.descendants() {
        if node.tag_name().name() == "link" {
            let link_name: String = node
                .attribute("name")
                .ok_or("link missing name attribute")?
                .to_string();
            links.push(Link { name: link_name });
        } else if node.tag_name().name() == "joint" {
            let name: String = node
                .attribute("name")
                .ok_or("joint missing name attribute")?
                .to_string();
            let parent: String = node
                .children()
                .find(|n| n.tag_name().name() == "parent")
                .and_then(|n| n.attribute("link"))
                .ok_or_else(|| format!("missing parent link for joint {}", name))?
                .to_string();
            let child: String = node
                .children()
                .find(|n| n.tag_name().name() == "child")
                .and_then(|n| n.attribute("link"))
                .ok_or_else(|| format!("missing child link for joint {}", name))?
                .to_string();

            // Extracting joint XYZ and RPY
            let joint_origin = node
                .children()
                .find(|n| n.tag_name().name() == "origin")
                .ok_or_else(|| format!("missing origin for joint {}", name))?;

            let xyz_str: &str = joint_origin
                .attribute("xyz")
                .ok_or_else(|| format!("missing xyz for joint {}", name))?;
            let (x, y, z) = parse_vec3_str(xyz_str)?;
            let xyz = Vector3::new(x, y, z);

            let rpy_str = joint_origin
                .attribute("rpy")
                .ok_or_else(|| format!("missing rpy for joint {}", name))?;
            let (roll, pitch, yaw) = parse_vec3_str(rpy_str)?;
            let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);

            let transform = Isometry3::from_parts(Translation3::from(xyz), rotation);

            // Extracting axis angles
            let axis_str: &str = node
                .children()
                .find(|n| n.tag_name().name() == "axis")
                .and_then(|n| n.attribute("xyz"))
                .ok_or_else(|| format!("missing axis xyz value for joint {}", name))?;
            let (axis_x, axis_y, axis_z) = parse_vec3_str(axis_str)?;
            let axis = Unit::new_normalize(Vector3::new(axis_x, axis_y, axis_z));

            // Extracting joint limits
            let joint_limit = node
                .children()
                .find(|n| n.tag_name().name() == "limit")
                .ok_or_else(|| format!("missing joint limits for joint {}", name))?;

            let limit_lower: f64 = joint_limit
                .attribute("lower")
                .ok_or_else(|| format!("missing joint limit lower for joint {}", name))?
                .parse::<f64>()?;
            let limit_upper: f64 = joint_limit
                .attribute("upper")
                .ok_or_else(|| format!("missing joint limit upper for joint {}", name))?
                .parse::<f64>()?;

            // Creating joint
            let joint: Joint = Joint {
                name,
                parent,
                child,
                transform,
                axis,
                limit_lower,
                limit_upper,
            };
            joints.push(joint);
        }
    }

    Ok(RobotModel {
        name: robot_name,
        links: links,
        joints: joints,
    })
}
