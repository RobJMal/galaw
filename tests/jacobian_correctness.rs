/// Tests the correctness of the implemented Jacobian computation
/// with Rust's k library
// Third-party
use rand::{SeedableRng};
use rand_chacha::ChaCha8Rng;
use k;

// Custom
use galaw::{types::GalawModel};

mod common;
use common::{TestResult, NUM_POSES, RNG_SEED, assert_close, setup_kinematic_models, zero_joint_cmds, random_joint_cmds};

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

    let zero_joint_cmds: Vec<f64> = zero_joint_cmds(&galaw_model);
    asssert_galaw_jacobian_matches_k(&galaw_model, &k_chain, &zero_joint_cmds)?;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds: Vec<f64> = random_joint_cmds(&galaw_model, &mut rng);
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