// Third-party
use rand::RngExt;
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{load_urdf, types::GalawModel};

// ---- TYPES ----
pub type TestResult = Result<(), Box<dyn std::error::Error>>;

// ---- CONSTANTS ----
pub const TEST_TOLERANCE: f64 = 1e-10;
pub const RNG_SEED: u64 = 42;
pub const NUM_POSES: usize = 128;

// ---- HELPERS ----
pub fn assert_close(a: f64, b: f64) {
    assert!(
        (a - b).abs() < TEST_TOLERANCE,
        "expected {b}, got {a} OR not within {TEST_TOLERANCE}"
    );
}

/// Sets up the different kinematics model for testing.
///
/// Mainly done because k_chain is stateful and thus we'll need to
/// instantiate it for each test.
pub fn setup_kinematic_models(urdf_path: &str) -> (GalawModel, k::Chain<f64>) {
    let galaw_model = load_urdf(urdf_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();
    (galaw_model, k_chain)
}

/// Generates zero joint commands.
pub fn zero_joint_cmds(model: &GalawModel) -> Vec<f64> {
    model
        .joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .map(|j| match (j.limit_lower, j.limit_upper) {
            (Some(lower), Some(upper)) => 0.0_f64.clamp(lower, upper),
            _ => 0.0,
        })
        .collect()
}

/// Generates random joint commands.
pub fn random_joint_cmds(model: &GalawModel, rng: &mut ChaCha8Rng) -> Vec<f64> {
    model
        .joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .map(|j| match (j.limit_lower, j.limit_upper) {
            (Some(lower), Some(upper)) => rng.random_range(lower..upper),
            _ => rng.random_range(0.0..0.0),
        })
        .collect()
}
