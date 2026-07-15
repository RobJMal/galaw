use taligalaw::{load_urdf, types::{self, Position3D, Quaternion}};

const TEST_TOLERANCE: f64 = 1e-10;

fn assert_close(a: f64, b: f64) {
    assert!((a - b).abs() < TEST_TOLERANCE, "expected {b}, got {a} OR not within {TEST_TOLERANCE}");
}

/// Need to do this test because quaternions double-cover rotations (q=-q are same rotation)
fn assert_orientation_close(a: &Quaternion, b: &Quaternion) {
    let dot_prod = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    assert_close(dot_prod.abs(), 1.0);
}

fn assert_position3d_close(a: &Position3D, b: &Position3D) {
    assert_close(a.x, b.x);
    assert_close(a.y, b.y);
    assert_close(a.z, b.z);
}



#[test]
fn test_zero_cmd() -> Result<(), Box<dyn std::error::Error>> {
    let urdf_file_path: String = String::from("assets/simple_robot.urdf");

    // Robot models
    let tg_robot_model = load_urdf(&urdf_file_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(&urdf_file_path).unwrap();

    // Test input
    let joint_cmd = [0.0, 0.0];

    let tg_result = tg_robot_model.compute_fk(&joint_cmd)?;
    let _  = k_chain.set_joint_positions(&joint_cmd);
    let _ = k_chain.update_transforms();

    for link in tg_robot_model.links.iter() {
        let tg_link = &tg_result[link];
        let k_link = k_chain.find_link(&link.name).unwrap().world_transform().ok_or("invalid result")?;
        let k_link_position = Position3D {
            x: k_link.translation.x, 
            y: k_link.translation.y, 
            z: k_link.translation.z
        };
        let k_link_orientation = types::Quaternion {
            x: k_link.rotation.i,
            y: k_link.rotation.j,
            z: k_link.rotation.k,
            w: k_link.rotation.w,
        };

        assert_position3d_close(&tg_link.position, &k_link_position);
        assert_orientation_close(&tg_link.orientation, &k_link_orientation);
    }

    Ok(())
}

