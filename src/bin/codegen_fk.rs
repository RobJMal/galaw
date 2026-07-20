use std::env::{args};

// Custom
use galaw::load_urdf;


fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = args().collect();

    let urdf_path = &args[1];
    let out_path = &args[2];

    let galaw_model = load_urdf(urdf_path)?;

    // Modules/libraries that are imported
    let import_code: String = format!(
        "use nalgebra::{{Isometry3, Translation3, UnitQuaternion, Quaternion, Unit, Vector3}};"
    );
    println!("{}", import_code);

    // Function header code
    let fn_header_code: String = format!(
        "pub fn compute_fk(joint_cmds: &[f64; {}]) -> [Isometry3<f64>; {}] {{",
        galaw_model.num_actuated_joints,
        galaw_model.links.len(),
    );
    println!("{}", fn_header_code);

    let base_link_var_code: String = format!("let link_base_link = Isometry3::identity();");
    println!("{}", base_link_var_code);

    // Keeps track of the generated variables
    let mut link_vars_by_idx: Vec<Option<String>> = vec![None; galaw_model.links.len()];
    let root_link_idx = galaw_model.get_link_idx("base_link").ok_or("no base_link_found")?;
    link_vars_by_idx[root_link_idx] = Some("link_base_link".to_string());

    for joint in galaw_model.joints.iter() {
        let link_name_var: String = format!("link_{}", joint.child);
        let parent_var: String = format!("link_{}", joint.parent);

        // Using Unit::new_unchecked since already normalized in parser.rs
        let rotation: String = match joint.rot_axis {
            Some(axis) => {
                let vec = axis.into_inner();
                let axis_vec_str: String = format!("Unit::new_unchecked(Vector3::new({:?}, {:?}, {:?}))", vec.x, vec.y, vec.z);
                format!("UnitQuaternion::from_axis_angle(&{}, joint_cmds[{}])", axis_vec_str, joint.cmd_idx.unwrap()).to_string()
            }
            None => "UnitQuaternion::identity()".to_string(),
        };
        let translation: String = match joint.lin_axis {
            Some(axis) => {
                let vec = axis.into_inner();
                let axis_vec_str: String = format!("Unit::new_unchecked(Vector3::new({:?}, {:?}, {:?}))", vec.x, vec.y, vec.z);
                format!("Translation3::from({} * joint_cmds[{}])", axis_vec_str, joint.cmd_idx.unwrap()).to_string()
            }
            None => "Translation3::identity()".to_string(),
        };
        let joint_transform_t = &joint.transform.translation;
        let joint_transform_t_str: String = format!("Translation3::new({:?}, {:?}, {:?})", joint_transform_t.x, joint_transform_t.y, joint_transform_t.z).to_string();
        let joint_transform_r = &joint.transform.rotation;
        let joint_transform_r_str: String = format!("UnitQuaternion::from_quaternion(Quaternion::new({:?}, {:?}, {:?}, {:?}))", joint_transform_r.w, joint_transform_r.i, joint_transform_r.j, joint_transform_r.k).to_string();

        let joint_transform: String = format!(
            "Isometry3::from_parts({}, {})", joint_transform_t_str, joint_transform_r_str 
        );

        let joint_local: String = format!("{}*Isometry3::from_parts({}, {})", joint_transform, translation, rotation).to_string();

        let code_line: String = format!("let {} = {} * {};", link_name_var, parent_var, joint_local);
        link_vars_by_idx[joint.child_link_idx] = Some(link_name_var.clone());
        println!("{}", code_line);
        println!();
    }

    // Putting the link_vars in order in the return array
    let mut ordered_link_vars: Vec<String> = Vec::new();
    for i in 0..galaw_model.links.len() {  
        let var_name = link_vars_by_idx[i].clone().unwrap();
        ordered_link_vars.push(var_name);
    }
    let fn_return_code: String = format!("[{}]", ordered_link_vars.join(", "));
    println!("{}", fn_return_code);

    let fn_closer_code: String = format!("}}").to_string();
    println!("{}", fn_closer_code);

    println!("Generated code has been written to: {}", out_path);
    Ok(())
}