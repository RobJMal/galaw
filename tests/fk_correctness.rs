/// Tests the correctness of the implmeented forward kinematics function
/// with Rust's k library

// Third-party
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{load_urdf, types::GalawModel};

// TYPES
type TestResult = Result<(), Box<dyn std::error::Error>>;

// CONSTANTS
const TEST_TOLERANCE: f64 = 1e-10;
const RNG_SEED: u64 = 42;
const NUM_POSES: usize = 128; // Number of random robot poses to test out

// HELPERS
fn assert_close(a: f64, b: f64) {
    assert!(
        (a - b).abs() < TEST_TOLERANCE,
        "expected {b}, got {a} OR not within {TEST_TOLERANCE}"
    );
}

/// Need to do this test because quaternions double-cover rotations (q=-q are same rotation)
fn assert_orientation_close(a: &UnitQuaternion<f64>, b: &UnitQuaternion<f64>) {
    let dot_prod = a.i * b.i + a.j * b.j + a.k * b.k + a.w * b.w;
    assert_close(dot_prod.abs(), 1.0);
}

fn assert_position3d_close(a: &Translation3<f64>, b: &Translation3<f64>) {
    assert_close(a.x, b.x);
    assert_close(a.y, b.y);
    assert_close(a.z, b.z);
}

fn assert_transform_close(galaw_transform: &Isometry3<f64>, k_iso: &k::nalgebra::Isometry3<f64>) {
    assert_position3d_close(&galaw_transform.translation, &k_iso.translation);
    assert_orientation_close(&galaw_transform.rotation, &k_iso.rotation);
}

fn assert_galaw_fk_matches_k(
    galaw_model: &GalawModel,
    k_chain: &k::Chain<f64>,
    joint_cmd: &[f64],
) -> TestResult {
    eprintln!("[input] joint_cmd = {:?}", joint_cmd);

    let galaw_result = galaw_model.compute_fk(joint_cmd)?;
    k_chain.set_joint_positions(joint_cmd)?;
    k_chain.update_transforms();

    for (i, link) in galaw_model.links.iter().enumerate() {
        let k_link = k_chain
            .find_link(&link.name)
            .unwrap()
            .world_transform()
            .ok_or("invalid result")?;

        assert_transform_close(&galaw_result[i], &k_link);
    }

    Ok(())
}

/// Because k_chain is stateful, cannot have it easily parallized and need to instantiate it for each test
fn setup_kinematic_models(urdf_path: &str) -> (GalawModel, k::Chain<f64>) {
    let galaw_robot_model = load_urdf(urdf_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();
    (galaw_robot_model, k_chain)
}

/// Runs the full correctness check (zero pose + random poses) for one URDF.
/// The joint count is read from the model, so this works for any robot.
fn check_fk_for_urdf(urdf_path: &str) -> TestResult {
    eprintln!("[urdf] {urdf_path}");
    let (galaw_model, k_chain) = setup_kinematic_models(urdf_path);
    let n_joints = galaw_model.joints.len();

    // Zero pose: a vector of zeros sized to this robot (not hardcoded to 2).
    let zero_cmd = vec![0.0; n_joints];
    assert_galaw_fk_matches_k(&galaw_model, &k_chain, &zero_cmd)?;

    // Random poses within each joint's limits (deterministic via the seed).
    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds: Vec<f64> = galaw_model
            .joints
            .iter()
            .map(|j| rng.random_range(j.limit_lower..j.limit_upper))
            .collect();
        assert_galaw_fk_matches_k(&galaw_model, &k_chain, &joint_cmds)?;
    }

    Ok(())
}

/// Generates one `#[test]` per URDF. Each robot becomes its own named test, so
/// `cargo test` shows exactly which robot ran and which one failed. To cover a
/// new robot, add a single `name => "path"` line below.
macro_rules! fk_correctness_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_fk_for_urdf($path)
            }
        )*
    };
}

fk_correctness_tests! {
    simple_robot     => "assets/simple_robot.urdf",
    simple_arm_6dof  => "assets/simple_arm_6dof.urdf",
    simple_arm_10dof => "assets/simple_arm_10dof.urdf",
    simple_arm_20dof => "assets/simple_arm_20dof.urdf",
}
