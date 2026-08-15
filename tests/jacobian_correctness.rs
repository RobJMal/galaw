use nalgebra::SMatrix;
/// Tests the correctness of the implemented Jacobian computation
/// with Rust's k library
// Third-party
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::types::GalawModel;

mod common;
use common::{
    RNG_SEED, TestResult, assert_close, random_joint_cmds, setup_kinematic_models,
    zero_joint_cmds,
};

// ---- CONSTANTS ----
const NUM_POSES: usize = 128;
const TEST_TOLERANCE: f64 = 1e-7;
const FD_EPS: f64 = 1e-6;
const FD_TOLERANCE: f64 = 1e-5; // looser numerical error for finite difference 

/// Checks that results are close to finite-difference method.
fn assert_close_fd(a: f64, b: f64) {
    assert!(
        (a - b).abs() < FD_TOLERANCE,
        "expected {b}, got {a} OR not within {FD_TOLERANCE}"
    );
}

/// Compare's galaw's analytic Jacobian against a central-difference approximation.
///
/// Purpose is to check correctness without relying on external libraries.
fn assert_galaw_jacobian_matches_finite_difference(
    galaw_model: &GalawModel,
    joint_cmds: &[f64],
) -> TestResult {
    let jacobians = galaw_model.compute_jacobian(joint_cmds)?;

    for joint in &galaw_model.joints {
        let Some(cmd_idx) = joint.cmd_idx else {
            continue;
        };

        let mut plus = joint_cmds.to_vec();
        plus[cmd_idx] += FD_EPS;
        let mut minus = joint_cmds.to_vec();
        minus[cmd_idx] -= FD_EPS;

        let links_plus = galaw_model.compute_fk(&plus)?;
        let links_minus = galaw_model.compute_fk(&minus)?;

        for link_idx in 0..galaw_model.links.len() {
            let linear = (links_plus[link_idx].translation.vector
                - links_minus[link_idx].translation.vector)
                / (2.0 * FD_EPS);

            let relative_rotation =
                links_plus[link_idx].rotation * links_minus[link_idx].rotation.inverse();
            let angular = relative_rotation.scaled_axis() / (2.0 * FD_EPS);

            let jac = &jacobians[link_idx];
            assert_close_fd(jac[(0, cmd_idx)], linear.x);
            assert_close_fd(jac[(1, cmd_idx)], linear.y);
            assert_close_fd(jac[(2, cmd_idx)], linear.z);
            assert_close_fd(jac[(3, cmd_idx)], angular.x);
            assert_close_fd(jac[(4, cmd_idx)], angular.y);
            assert_close_fd(jac[(5, cmd_idx)], angular.z);
        }
    }

    Ok(())
}

/// Compares galaw's Jacobian against k's for every link in the model at one pose.
fn asssert_galaw_jacobian_matches_k(
    galaw_model: &GalawModel,
    k_chain: &k::Chain<f64>,
    joint_cmds: &[f64],
) -> TestResult {
    k_chain.set_joint_positions(joint_cmds)?;
    k_chain.update_transforms();

    let galaw_jacobian = galaw_model.compute_jacobian(joint_cmds)?;

    for (target_link_idx, link) in galaw_model.links.iter().enumerate() {
        let galaw_link_jacobian = &galaw_jacobian[target_link_idx];

        let k_node = k_chain
            .find_link(&link.name)
            .ok_or("link missing from k chain")?;
        let k_serial = k::SerialChain::from_end(k_node);
        let k_jacobian = k::jacobian(&k_serial);

        for (k_col_idx, k_joint) in k_serial.iter_joints().enumerate() {
            let cmd_idx = galaw_model
                .get_joint_idx(&k_joint.name)
                .ok_or("joint missing from galaw model")?;
            for row in 0..6 {
                assert_close(
                    galaw_link_jacobian[(row, cmd_idx)],
                    k_jacobian[(row, k_col_idx)],
                    &TEST_TOLERANCE,
                );
            }
        }
    }

    Ok(())
}

/// Runs correctness check of Jacobians with finite-difference method
fn check_jacobian_matches_fd_for_urdf(urdf_path: &str) -> TestResult {
    let galaw_model = galaw::load_urdf(urdf_path)?;

    assert_galaw_jacobian_matches_finite_difference(&galaw_model, &zero_joint_cmds(&galaw_model))?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds = random_joint_cmds(&galaw_model, &mut rng);
        assert_galaw_jacobian_matches_finite_difference(&galaw_model, &joint_cmds)?;
    }

    Ok(())
}

/// Runs correctness check generated `compute_jacobian` against the runtime version.
fn check_generated_jacobian_matches_dynamic<const N: usize, const M: usize>(
    urdf_path: &str,
    generated_compute_jacobian: impl Fn(&[f64; N]) -> [SMatrix<f64, 6, N>; M],
) -> TestResult {
    let galaw_model = galaw::load_urdf(urdf_path)?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds = random_joint_cmds(&galaw_model, &mut rng);
        let dynamic_jacobians = galaw_model.compute_jacobian(&joint_cmds)?;

        let joint_cmds_arr: [f64; N] = joint_cmds.clone().try_into().unwrap();
        let generated_jacobians = generated_compute_jacobian(&joint_cmds_arr);

        for link_idx in 0..galaw_model.links.len() {
            for row in 0..6 {
                for col in 0..N {
                    assert_close(
                        dynamic_jacobians[link_idx][(row, col)],
                        generated_jacobians[link_idx][(row, col)],
                        &TEST_TOLERANCE,
                    );
                }
            }
        }
    }

    Ok(())
}

/// Runs the full correctness check (zero pose + random poses) for one URDF
fn check_jacobian_for_urdf(urdf_path: &str) -> TestResult {
    let (galaw_model, k_chain) = setup_kinematic_models(urdf_path);

    let zero_joint_cmds: Vec<f64> = zero_joint_cmds(&galaw_model);
    asssert_galaw_jacobian_matches_k(&galaw_model, &k_chain, &zero_joint_cmds)?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds: Vec<f64> = random_joint_cmds(&galaw_model, &mut rng);
        asssert_galaw_jacobian_matches_k(&galaw_model, &k_chain, &joint_cmds)?;
    }

    Ok(())
}

/// Generates Jacobian finite difference tests
macro_rules! jacobian_fd_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_jacobian_matches_fd_for_urdf($path)
            }
        )*
    };
}

jacobian_fd_tests! {
    simple_arm_2dof_fd  => "assets/urdf/custom/simple_arm_2dof.urdf",
    simple_arm_2dof_flipped_fd => "assets/urdf/custom/simple_arm_2dof_flipped.urdf",
    simple_arm_3dof_rrp_fd => "assets/urdf/custom/simple-arm_3dof_rrp.urdf",
    flexiv_enlight_l_fd => "assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",
    anymal_d_fd => "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",
    wuji_hand_v1_right_fd => "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",
    stretch4_fd => "assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",
}

/// Generates Jacobian calculation comparison test (k v galaw)
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

/// Generates Jacobian runtime vs generated tests
macro_rules! jacobian_codegen_correctness_test {
    ($module:ident, $path:expr, $compute_fk:path) => {
        mod $module {
            use super::*;

            #[test]
            fn matches_dynamic() -> TestResult {
                check_generated_jacobian_matches_dynamic(
                    $path,
                    galaw::generated::$module::compute_jacobian,
                )
            }
        }
    };
}
galaw::for_each_generated_robot!(jacobian_codegen_correctness_test);

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
