/// Tests the correctness of the implemented inverse kinematics function
/// with Rust's k library
// Third-party
use rand::{SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{types::GalawModel};

mod common;
use common::{
    RNG_SEED, TestResult, random_joint_cmds, setup_kinematic_models,
    zero_joint_cmds,
};

use crate::common::assert_galaw_transform_close;

// ---- CONSTANTS ----
pub const TEST_TOLERANCE: f64 = 1e-10;
pub const NUM_POSES: usize = 128;

fn assert_galaw_ik_correctness(
    galaw_model: &GalawModel,
    target_joint_cmd: &[f64],
    init_joint_cmd: &[f64],
) -> TestResult {
    eprintln!("[input] target_joint_cmd = {:?}", target_joint_cmd);
    
    // Known solution
    let target_link_poses = galaw_model.compute_fk(target_joint_cmd)?;
    let target_end_effector_pose = target_link_poses[target_link_poses.len() - 1];
    let target_link_idx = target_link_poses.len() - 1;

    let solved_joint_cmds = galaw_model.compute_ik(target_link_idx, &target_end_effector_pose, init_joint_cmd)?;
    let solved_link_poses = galaw_model.compute_fk(&solved_joint_cmds)?;
    let solved_end_effector_pose = solved_link_poses[solved_link_poses.len() - 1];

    assert_galaw_transform_close(&target_end_effector_pose, &solved_end_effector_pose);

    Ok(())
}

/// Run IK check for each URDF.
fn check_ik_for_urdf(urdf_path: &str) -> TestResult {
    eprintln!("[urdf] {urdf_path}");
    let (galaw_model, _) = setup_kinematic_models(urdf_path);

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    // Zero joint cmd
    let zero_joint_cmd: Vec<f64> = zero_joint_cmds(&galaw_model);
    let init_joint_cmd: Vec<f64> = random_joint_cmds(&galaw_model, &mut rng);
    assert_galaw_ik_correctness(
        &galaw_model, 
        &zero_joint_cmd,
        &init_joint_cmd,
    );

    Ok(())
}

/// Generates one `#[test]` per URDF.
macro_rules! ik_correctness_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_ik_for_urdf($path)
            }
        )*
    };
}

ik_correctness_tests! {
    simple_arm_2dof  => "assets/urdf/custom/simple_arm_2dof.urdf",
}
