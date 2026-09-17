/// Type parity checks for forward kinematics.
///
/// Verifies that FK results are consistent across float types. A lower-precision
/// type should agree with a higher-precision type within the narrower type's
/// precision limit. This protects the generic implementation from bugs that only
/// surface at specific precisions, and documents the expected precision trade-off
/// when switching types for performance.
// Third-party
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::load_urdf;

mod common;
use common::{RNG_SEED, TestResult};

const NUM_POSES: usize = 128;
// f32 has ~7 significant decimal digits; allow for accumulated floating-point
// error across a joint chain (typically <1e-4 for robot arms ~1 m in scale).
const PARITY_TOLERANCE: f64 = 1e-4;

fn check_fk_f32_f64_parity(urdf_path: &str) -> TestResult {
    let model_f32 = load_urdf::<f32>(urdf_path)?;
    let model_f64 = load_urdf::<f64>(urdf_path)?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        // Generate joint commands as f64, then narrow to f32 for the f32 model.
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

        let links_f64 = model_f64.compute_fk(&cmds_f64)?;
        let links_f32 = model_f32.compute_fk(&cmds_f32)?;

        for i in 0..model_f64.links.len() {
            let t32 = &links_f32[i].translation;
            let t64 = &links_f64[i].translation;
            assert!(
                (t32.x as f64 - t64.x).abs() < PARITY_TOLERANCE,
                "link {i} translation.x: f32={:.6e} f64={:.6e}",
                t32.x,
                t64.x
            );
            assert!(
                (t32.y as f64 - t64.y).abs() < PARITY_TOLERANCE,
                "link {i} translation.y: f32={:.6e} f64={:.6e}",
                t32.y,
                t64.y
            );
            assert!(
                (t32.z as f64 - t64.z).abs() < PARITY_TOLERANCE,
                "link {i} translation.z: f32={:.6e} f64={:.6e}",
                t32.z,
                t64.z
            );

            // Quaternions double-cover SO(3): q and -q represent the same rotation,
            // so compare via |q32 · q64| ≈ 1.
            let r32 = &links_f32[i].rotation;
            let r64 = &links_f64[i].rotation;
            let dot = ((r32.w as f64) * r64.w
                + (r32.i as f64) * r64.i
                + (r32.j as f64) * r64.j
                + (r32.k as f64) * r64.k)
                .abs();
            assert!(
                (dot - 1.0).abs() < PARITY_TOLERANCE,
                "link {i} rotation: |q32·q64|={dot:.6e}"
            );
        }
    }

    Ok(())
}

macro_rules! parity_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_fk_f32_f64_parity($path)
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
