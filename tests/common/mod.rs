// Each tests/*.rs compiles as its own crate, so dead_code is checked
// per-binary — a function unused here may still be used by a sibling test.
#![allow(dead_code)]

// Third-party
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rand::RngExt;
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{load_urdf, types::GalawModel};

// ---- TYPES ----
pub type TestResult = Result<(), Box<dyn std::error::Error>>;

// ---- CONSTANTS ----
pub const RNG_SEED: u64 = 42;

// ---- HELPERS ----
pub fn assert_close(a: f64, b: f64, test_tolerance: &f64) {
    assert!(
        (a - b).abs() < *test_tolerance,
        "expected {b}, got {a} OR not within {test_tolerance}"
    );
}

/// Need to do this test because quaternions double-cover rotations (q=-q are same rotation)
fn assert_orientation_close(a: &UnitQuaternion<f64>, b: &UnitQuaternion<f64>, test_tolerance: &f64) {
    let dot_prod = a.i * b.i + a.j * b.j + a.k * b.k + a.w * b.w;
    assert_close(dot_prod.abs(), 1.0, test_tolerance);
}

fn assert_position3d_close(a: &Translation3<f64>, b: &Translation3<f64>, test_tolerance: &f64) {
    assert_close(a.x, b.x, test_tolerance);
    assert_close(a.y, b.y, test_tolerance);
    assert_close(a.z, b.z, test_tolerance);
}

pub fn assert_galaw_k_transform_close(
    galaw_transform: &Isometry3<f64>, 
    k_iso: &k::nalgebra::Isometry3<f64>,
    test_tolerance: &f64,
) {
    assert_position3d_close(&galaw_transform.translation, &k_iso.translation, test_tolerance);
    assert_orientation_close(&galaw_transform.rotation, &k_iso.rotation, test_tolerance);
}

/// Both sides are plain `nalgebra::Isometry3<f64>` here (dynamic vs codegen'd
/// FK), unlike `assert_galaw_k_transform_close` which bridges to `k`'s own re-exported
/// nalgebra type.
pub fn assert_galaw_transform_close(a: &Isometry3<f64>, b: &Isometry3<f64>, test_tolerance: &f64) {
    assert_position3d_close(&a.translation, &b.translation, test_tolerance);
    assert_orientation_close(&a.rotation, &b.rotation, test_tolerance);
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
