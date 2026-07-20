use std::{env::args};

// Custom
use galaw::load_urdf;


fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = args().collect();

    let urdf_path = &args[1];
    let out_path = &args[2];

    println!("urdf_path: {:?}", urdf_path);
    println!("out_path: {:?}", out_path);

    let galaw_model = load_urdf(urdf_path)?;
    println!("loaded {} - {} links, {} joints", galaw_model.name, galaw_model.links.len(), galaw_model.joints.len());

    for joint in galaw_model.joints.iter() {
        let link_name: String = format!("link_{}", joint.child);
        let parent_var: String = format!("link_{}", joint.parent);

        let rotation: String = match joint.rot_axis {
            Some(axis) => {
                let vec = axis.into_inner();
                let axis_vec_str: String = format!("Vector3::new({}, {}, {})", vec.x, vec.y, vec.z);
                format!("UnitQuaternion::from_axis_angle(&{}, &joint_cmds[{}])", axis_vec_str, joint.cmd_idx.unwrap()).to_string()
            }
            None => "UnitQuaternion::identity()".to_string(),
        };
        let translation: String = match joint.lin_axis {
            Some(axis) => {
                let vec = axis.into_inner();
                let axis_vec_str: String = format!("Vector3::new({}, {}, {})", vec.x, vec.y, vec.z);
                format!("Translation3::from({} * &joint_cmds[{}]),", axis_vec_str, joint.cmd_idx.unwrap()).to_string()
            }
            None => "Translation3::identity()".to_string(),
        };
        let joint_transform_t = &joint.transform.translation;
        let joint_transform_t_str: String = format!("Translation3::new({}, {}, {})", joint_transform_t.x, joint_transform_t.y, joint_transform_t.z).to_string();
        let joint_transform_r = &joint.transform.rotation;
        let joint_transform_r_str: String = format!("UnitQuaternion::from_quaternion(Quaternion::new({}, {}, {}, {}))", joint_transform_r.w, joint_transform_r.i, joint_transform_r.j, joint_transform_r.k).to_string();

        let joint_transform: String = format!(
            "Isometry3::from_parts({}, {}))", joint_transform_t_str, joint_transform_r_str 
        );

        let joint_local: String = format!("{}*Isometry3::from_parts({}, {})", joint_transform, translation, rotation).to_string();

        let code_line: String = format!("let {} = {} * {}", link_name, parent_var, joint_local);
        println!("{}", code_line);
        println!();
    }

    Ok(())
}