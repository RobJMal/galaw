/// Tests the correctness of the implemented Jacobian computation
/// with Rust's k library
// Third-party
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use k;

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

/// Sets up the different kinematics model for testing.
/// 
/// Mainly done because k_chain is stateful and thus we'll need to 
/// instantiate it for each test.
fn setup_kinematic_models(urdf_path: &str) -> (GalawModel, k::Chain<f64>) {
    let galaw_model = load_urdf(urdf_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();
    (galaw_model, k_chain)
}

/// Compares galaw's Jacobian against k's for every link in the model at one pose.
fn asssert_galaw_jacobian_matches_k(
    galaw_model: &GalawModel,
    k_chain: &k::Chain<f64>,
    joint_cmds: &[f64],
) -> TestResult {
    k_chain.set_joint_positions(joint_cmds)?;
    k_chain.update_transforms();

    for (target_link_idx, link) in galaw_model.links.iter().enumerate() {
        let galaw_jacobian = galaw_model.compute_jacobian(&joint_cmds, target_link_idx)?;

        let k_node = k_chain.find_link(&link.name).ok_or("link missing from k chain")?;
        let k_serial = k::SerialChain::from_end(k_node);
        let k_jacobian = k::jacobian(&k_serial);

        for (k_col_idx, k_joint) in k_serial.iter_joints().enumerate() {
            let cmd_idx = galaw_model
                .get_joint_idx(&k_joint.name)
                .ok_or("joint missing from galaw model")?;
            for row in 0..6 {
                assert_close(galaw_jacobian[(row, cmd_idx)], k_jacobian[(row, k_col_idx)]);
            }
        }
    }

    Ok(())
}

/// Runs the full correctness check (zero pose + random poses) for one URDF
fn check_jacobian_for_urdf(urdf_path: &str) -> TestResult {
    let (galaw_model, k_chain) = setup_kinematic_models(urdf_path);

    let zero_joint_cmds: Vec<f64> = galaw_model
        .joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .map(|j| match (j.limit_lower, j.limit_upper) {
            (Some(lower), Some(upper)) => 0.0_f64.clamp(lower, upper),
            _ => 0.0,
        })
        .collect();
    asssert_galaw_jacobian_matches_k(&galaw_model, &k_chain, &zero_joint_cmds)?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds: Vec<f64> = galaw_model
            .joints
            .iter()
            .filter(|j| j.cmd_idx.is_some())
            .map(|j| match (j.limit_lower, j.limit_upper) {
                (Some(lower), Some(upper)) => rng.random_range(lower..upper),
                _ => rng.random_range(0.0..0.0),
            })
            .collect();
        asssert_galaw_jacobian_matches_k(&galaw_model, &k_chain, &joint_cmds)?;
    }

    Ok(())
}

/// Generates one `#[test]` per URDF.
macro_rules! jacobian_correctness_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_jacobian_for_urdf($path)
            }
        )*
    };
}

jacobian_correctness_tests! {
    simple_arm_2dof  => "assets/urdf/custom/simple_arm_2dof.urdf",
    simple_arm_2dof_flipped => "assets/urdf/custom/simple_arm_2dof_flipped.urdf",
    simple_arm_3dof_rrp => "assets/urdf/custom/simple-arm_3dof_rrp.urdf",

    // Third-party robots
    flexiv_enlight_l => "assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",
    anymal_d => "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",
    wuji_hand_v1_right => "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",
    stretch4 => "assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",
}