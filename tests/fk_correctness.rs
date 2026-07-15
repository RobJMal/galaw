// Third-party
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom 
use taligalaw::{load_urdf, types::{Position3D, Quaternion, RobotModel}};


const TEST_TOLERANCE: f64 = 1e-10;
const RNG_SEED: u64 = 42;


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

/// Converts to Position3D 
fn to_position3d(t: &k::nalgebra::Translation3<f64>) -> Position3D {
    Position3D { x: t.x, y: t.y, z: t.z }
}

/// Converts to Quaternion
fn to_quaternion(q: k::nalgebra::Quaternion<f64>) -> Quaternion {
    Quaternion { x: q.i, y: q.j, z: q.k, w: q.w }
}

fn assert_tg_fk_matches_k(tg_model: &RobotModel, k_chain: &k::Chain<f64>, joint_cmd: &[f64]) -> Result<(), Box<dyn std::error::Error>> {
    eprintln!("[input] joint_cmd = {:?}", joint_cmd);

    let tg_result = tg_model.compute_fk(joint_cmd)?;
    k_chain.set_joint_positions(joint_cmd)?;
    k_chain.update_transforms();

    for link in tg_model.links.iter() {
        let tg_link = &tg_result[link];
        let k_link = k_chain
            .find_link(&link.name).unwrap()
            .world_transform().ok_or("invalid result")?;

        assert_position3d_close(&tg_link.position, &to_position3d(&k_link.translation));
        assert_orientation_close(&tg_link.orientation, &to_quaternion(*k_link.rotation.quaternion()));
    }

    Ok(())
}


/// Because k_chain is stateful, cannot have it easily parallized and need to instantiate it for each test
fn setup_kinematic_models() -> (RobotModel, k::Chain<f64>) {
    let urdf_path = "assets/simple_robot.urdf";
    let tg_robot_model = load_urdf(urdf_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();
    (tg_robot_model, k_chain)
}


#[test]
fn test_zero_cmd() -> Result<(), Box<dyn std::error::Error>> {
    let (tg_model, k_chain) = setup_kinematic_models();
    let joint_cmd = [0.0, 0.0];
    assert_tg_fk_matches_k(&tg_model, &k_chain, &joint_cmd)
}


#[test]
fn test_random_joint_cmds () -> Result<(), Box<dyn  std::error::Error>> {
    let (tg_model, k_chain) = setup_kinematic_models();
    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    for _ in 0..128 {
        let joint_cmds: Vec<f64> = tg_model.joints
            .iter()
            .map(|j| rng.random_range(j.limit_lower..j.limit_upper))
            .collect();
        assert_tg_fk_matches_k(&tg_model, &k_chain, &joint_cmds)?;
    }

    Ok(())
}

// Test at joint limit
