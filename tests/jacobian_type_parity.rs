/// Type parity checks for Jacobian computation.
///
/// Verifies that Jacobian elements are consistent across float types: a lower-precision
/// type should agree with a higher-precision type within the narrower type's precision
/// limit. Mirrors fk_type_parity.rs — same approach, applied to the full Jacobian matrix.
// Third-party
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::load_urdf;
use nalgebra::Matrix6xX;

mod common;
use common::{RNG_SEED, TestResult};

const NUM_POSES: usize = 128;
const PARITY_TOLERANCE: f64 = 1e-4;

fn check_jacobian_f32_f64_parity(urdf_path: &str) -> TestResult {
    let model_f32 = load_urdf::<f32>(urdf_path)?;
    let model_f64 = load_urdf::<f64>(urdf_path)?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let cmds_f64: Vec<f64> = model_f64
            .joints
            .iter()
            .filter(|j| j.cmd_idx.is_some())
            .map(|j| match (j.limit_lower, j.limit_upper) {
                (Some(lo), Some(hi)) => rng.random_range(lo..hi),
                _ => 0.0,
            })
            .collect();
        let cmds_f32: Vec<f32> = cmds_f64.iter().map(|&v| v as f32).collect();

        let mut jacs_f64 =
            vec![Matrix6xX::zeros(model_f64.num_actuated_joints); model_f64.links.len()];
        model_f64.compute_link_jacobians(&cmds_f64, &mut jacs_f64)?;
        let mut jacs_f32 =
            vec![Matrix6xX::zeros(model_f32.num_actuated_joints); model_f32.links.len()];
        model_f32.compute_link_jacobians(&cmds_f32, &mut jacs_f32)?;

        for link_idx in 0..model_f64.links.len() {
            for row in 0..6 {
                for col in 0..model_f64.num_actuated_joints {
                    let v32 = jacs_f32[link_idx][(row, col)] as f64;
                    let v64 = jacs_f64[link_idx][(row, col)];
                    assert!(
                        (v32 - v64).abs() < PARITY_TOLERANCE,
                        "link {link_idx} [{row},{col}]: f32={v32:.6e} f64={v64:.6e}"
                    );
                }
            }
        }
    }

    Ok(())
}

macro_rules! parity_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_jacobian_f32_f64_parity($path)
            }
        )*
    };
}

parity_tests! {
    simple_arm_2dof        => "assets/urdf/custom/simple_arm_2dof.urdf",
    simple_arm_3dof_rrp    => "assets/urdf/custom/simple-arm_3dof_rrp.urdf",
    flexiv_enlight_l       => "assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",
    anymal_d               => "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",
    wuji_hand_v1_right     => "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",
    stretch4               => "assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",
}
