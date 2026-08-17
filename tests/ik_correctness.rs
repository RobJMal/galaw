/// Tests the correctness of the implemented inverse kinematics function
/// with Rust's k library
// Third-party
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::types::GalawModel;

mod common;
use common::{RNG_SEED, TestResult, random_joint_cmds, setup_kinematic_models, zero_joint_cmds};

use crate::common::assert_galaw_transform_close;

// ---- CONSTANTS ----
const TEST_TOLERANCE: f64 = 1e-4;
const NUM_POSES: usize = 128;
const MAX_PERTUBATION: f64 = 0.5; // radians/meters offset from target for IK initial pose

/// Returns links that are valid IK targets (leaves of the kinematic tree)
fn candidate_target_links(galaw_model: &GalawModel) -> Vec<usize> {
    let parent_indices: std::collections::HashSet<usize> = galaw_model
        .joints
        .iter()
        .map(|j| j.parent_link_idx)
        .collect();

    let mut ancestors_by_link: Vec<Vec<usize>> = vec![Vec::new(); galaw_model.links.len()];
    for (joint_idx, joint) in galaw_model.joints.iter().enumerate() {
        let mut ancestors = ancestors_by_link[joint.parent_link_idx].clone();
        if joint.cmd_idx.is_some() {
            ancestors.push(joint_idx);
        }
        ancestors_by_link[joint.child_link_idx] = ancestors;
    }

    (0..galaw_model.links.len())
        .filter(|&link_idx| {
            !parent_indices.contains(&link_idx) && !ancestors_by_link[link_idx].is_empty()
        })
        .collect()
}

/// Perturbs base joint config by small random offset per joint.
///
/// This is done to better replicate how IK is used in practice
fn perturbed_joint_cmds(
    model: &GalawModel,
    base: &[f64],
    rng: &mut ChaCha8Rng,
    max_offset: f64,
) -> Vec<f64> {
    model
        .joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .zip(base.iter())
        .map(|(j, &base_value)| {
            let offset = rng.random_range(-max_offset..max_offset);
            let perturbed = base_value + offset;
            match (j.limit_lower, j.limit_upper) {
                (Some(lower), Some(upper)) => perturbed.clamp(lower, upper),
                _ => perturbed,
            }
        })
        .collect()
}

/// Assert that IK is correct by running check with internal FK.
fn assert_galaw_ik_correctness(
    galaw_model: &GalawModel,
    target_link_idx: usize,
    target_joint_cmd: &[f64],
    init_joint_cmd: &[f64],
) -> TestResult {
    eprintln!(
        "[input] target_link_idx = {target_link_idx}, target_joint_cmd = {:?}",
        target_joint_cmd
    );

    let target_link_pose = galaw_model.compute_fk(target_joint_cmd)?[target_link_idx];

    let solved_joint_cmds =
        galaw_model.compute_ik(target_link_idx, &target_link_pose, init_joint_cmd)?;
    let solved_link_poses = galaw_model.compute_fk(&solved_joint_cmds)?[target_link_idx];

    assert_galaw_transform_close(&target_link_pose, &solved_link_poses, &TEST_TOLERANCE);

    Ok(())
}

/// Run IK check for each URDF.
fn check_ik_for_urdf(urdf_path: &str) -> TestResult {
    eprintln!("[urdf] {urdf_path}");
    let (galaw_model, _) = setup_kinematic_models(urdf_path);

    let candidates = candidate_target_links(&galaw_model);
    assert!(
        !candidates.is_empty(),
        "no valid IK target links found for {urdf_path}"
    );
    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    // Zero-ish pose
    let zero_joint_cmd: Vec<f64> = zero_joint_cmds(&galaw_model);
    let init_joint_cmd: Vec<f64> = random_joint_cmds(&galaw_model, &mut rng);
    assert_galaw_ik_correctness(
        &galaw_model,
        candidates[0],
        &zero_joint_cmd,
        &init_joint_cmd,
    )?;

    for _ in 0..NUM_POSES {
        let target_link_idx = candidates[rng.random_range(0..candidates.len())];
        let target_joint_cmd = random_joint_cmds(&galaw_model, &mut rng);
        let init_joint_cmd =
            perturbed_joint_cmds(&galaw_model, &target_joint_cmd, &mut rng, MAX_PERTUBATION);
        assert_galaw_ik_correctness(
            &galaw_model,
            target_link_idx,
            &target_joint_cmd,
            &init_joint_cmd,
        )?;
    }

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
    simple_arm_3dof_rrp => "assets/urdf/custom/simple-arm_3dof_rrp.urdf",   // Tests revolute and prismatic

    // Third-party robots
    flexiv_enlight_l => "assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",  // Tests revolute and fixed
    anymal_d => "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",     // Tests revolute and fixed
    wuji_hand_v1_right => "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",  // Tests revolute and fixed
    stretch4 => "assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",     // Tests continuous, prismatic, revolute, fixed
}
