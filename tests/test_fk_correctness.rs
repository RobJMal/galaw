/// Tests the correctness of the implmeented forward kinematics function
/// with Rust's k library
// Third-party
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{
    load_urdf,
    types::{Position3D, Quaternion, RobotModel, Transform},
};

// TYPES
type TestResult = Result<(), Box<dyn std::error::Error>>;

// CONSTANTS
const TEST_TOLERANCE: f64 = 1e-10;
const RNG_SEED: u64 = 42;

// HELPERS
fn assert_close(a: f64, b: f64) {
    assert!(
        (a - b).abs() < TEST_TOLERANCE,
        "expected {b}, got {a} OR not within {TEST_TOLERANCE}"
    );
}

/// Need to do this test because quaternions double-cover rotations (q=-q are same rotation)
fn assert_orientation_close(a: &Quaternion, b: &Quaternion) {
    let dot_prod = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    assert_close(dot_prod.abs(), 1.0);
}

fn assert_position3d_close(a: &Position3D, b: &Position3D) {
    assert_close(a.x, b.x);
    assert_close(a.y, b.y);
    assert_close(a.z, b.z);
}

fn assert_transform_close(galaw_transform: &Transform, k_iso: &k::nalgebra::Isometry3<f64>) {
    assert_position3d_close(&galaw_transform.position, &to_position3d(&k_iso.translation));
    assert_orientation_close(
        &galaw_transform.orientation,
        &to_quaternion(*k_iso.rotation.quaternion()),
    );
}

/// Converts to Position3D
fn to_position3d(t: &k::nalgebra::Translation3<f64>) -> Position3D {
    Position3D {
        x: t.x,
        y: t.y,
        z: t.z,
    }
}

/// Converts to Quaternion
fn to_quaternion(q: k::nalgebra::Quaternion<f64>) -> Quaternion {
    Quaternion {
        x: q.i,
        y: q.j,
        z: q.k,
        w: q.w,
    }
}

fn assert_galaw_fk_matches_k(
    galaw_model: &RobotModel,
    k_chain: &k::Chain<f64>,
    joint_cmd: &[f64],
) -> TestResult {
    eprintln!("[input] joint_cmd = {:?}", joint_cmd);

    let galaw_result = galaw_model.compute_fk(joint_cmd)?;
    k_chain.set_joint_positions(joint_cmd)?;
    k_chain.update_transforms();

    for link in galaw_model.links.iter() {
        let k_link = k_chain
            .find_link(&link.name)
            .unwrap()
            .world_transform()
            .ok_or("invalid result")?;

        assert_transform_close(&galaw_result[link], &k_link);
    }

    Ok(())
}

/// Because k_chain is stateful, cannot have it easily parallized and need to instantiate it for each test
fn setup_kinematic_models() -> (RobotModel, k::Chain<f64>) {
    let urdf_path = "assets/simple_robot.urdf";
    let galaw_robot_model = load_urdf(urdf_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();
    (galaw_robot_model, k_chain)
}

#[test]
fn test_zero_cmd() -> TestResult {
    let (galaw_model, k_chain) = setup_kinematic_models();
    let joint_cmd = [0.0, 0.0];
    assert_galaw_fk_matches_k(&galaw_model, &k_chain, &joint_cmd)
}

#[test]
fn test_random_joint_cmds() -> TestResult {
    let (galaw_model, k_chain) = setup_kinematic_models();
    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    for _ in 0..128 {
        let joint_cmds: Vec<f64> = galaw_model
            .joints
            .iter()
            .map(|j| rng.random_range(j.limit_lower..j.limit_upper))
            .collect();
        assert_galaw_fk_matches_k(&galaw_model, &k_chain, &joint_cmds)?;
    }

    Ok(())
}
