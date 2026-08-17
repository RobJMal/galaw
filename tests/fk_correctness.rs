/// Tests the correctness of the implemented forward kinematics function
/// with Rust's k library
// Third-party
use nalgebra::Isometry3;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{load_urdf, types::GalawModel};

mod common;
use common::{
    RNG_SEED, TestResult, assert_galaw_k_transform_close, assert_galaw_transform_close,
    random_joint_cmds, setup_kinematic_models, zero_joint_cmds,
};

// ---- CONSTANTS ----
const NUM_POSES: usize = 128;
const TEST_TOLERANCE: f64 = 1e-7;

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

        assert_galaw_k_transform_close(&galaw_result[i], &k_link, &TEST_TOLERANCE);
    }

    Ok(())
}

/// Runs the full correctness check (zero pose + random poses) for one URDF.
/// The joint count is read from the model, so this works for any robot.
fn check_fk_for_urdf(urdf_path: &str) -> TestResult {
    eprintln!("[urdf] {urdf_path}");
    let (galaw_model, k_chain) = setup_kinematic_models(urdf_path);

    // Zero-ish pose: 0.0 clamped into each joint's own valid range — for most
    // joints (symmetric ranges like [-π, π]) this is still exactly zero; for
    // joints whose range doesn't include zero (e.g. a hand's finger joints,
    // which can't fully straighten), it's the closest valid value instead.
    let zero_joint_cmd: Vec<f64> = zero_joint_cmds(&galaw_model);
    assert_galaw_fk_matches_k(&galaw_model, &k_chain, &zero_joint_cmd)?;

    // Random poses within each joint's limits (deterministic via the seed).
    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds: Vec<f64> = random_joint_cmds(&galaw_model, &mut rng);
        assert_galaw_fk_matches_k(&galaw_model, &k_chain, &joint_cmds)?;
    }

    Ok(())
}

/// Compares a codegen'd `compute_fk` against the dynamic `GalawModel::compute_fk`
/// over many random poses, not just one hand-picked pose. Generic over the array
/// sizes baked into each robot's generated signature (`[f64; N] -> [Isometry3<f64>; M]`),
/// since those differ per robot and can't be unified any other way without codegen
/// itself producing a shared trait. The dynamic path is the ground truth here
/// (already checked against `k` above), so this only needs to confirm the
/// generated code agrees with it.
fn check_generated_matches_dynamic<const N: usize, const M: usize>(
    urdf_path: &str,
    generated_compute_fk: impl Fn(&[f64; N]) -> [Isometry3<f64>; M],
) -> TestResult {
    let galaw_model = load_urdf(urdf_path).unwrap();

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

        let dynamic_links = galaw_model.compute_fk(&joint_cmds)?;

        let joint_cmds_arr: [f64; N] = joint_cmds.clone().try_into().unwrap();
        let generated_links = generated_compute_fk(&joint_cmds_arr);

        for i in 0..galaw_model.links.len() {
            assert_galaw_transform_close(&dynamic_links[i], &generated_links[i], &TEST_TOLERANCE);
        }
    }

    Ok(())
}

/// Generates one `#[test]` per robot with codegen'd FK, via the shared
/// `for_each_generated_robot!` registry (src/generated/registry.rs, built by
/// scripts/codegen_all_urdfs.sh) instead of a hand-maintained list — the same
/// registry benches/fk_speed.rs uses for its own, differently-shaped purpose.
///
/// Each robot is wrapped in its own `mod $module` rather than one flat
/// `#[test] fn codegen_$module`, because a `path`/`ident` fragment already
/// captured by macro_rules can't be pasted together into a brand-new
/// identifier without a helper crate (e.g. `paste`) — nesting in a module
/// sidesteps that using only what's already in scope. The tradeoff:
/// `cargo test` now shows e.g. `simple_arm_2dof::matches_dynamic` instead of
/// a single flat `codegen_simple_arm_2dof`, and the per-robot "tests revolute
/// and fixed"-style comments that used to live on the old hand-written list
/// are gone (the registry only carries a path + module name, not comments).
macro_rules! codegen_correctness_test {
    ($module:ident, $path:expr, $compute_fk:path) => {
        mod $module {
            use super::*;

            #[test]
            fn matches_dynamic() -> TestResult {
                check_generated_matches_dynamic($path, $compute_fk)
            }
        }
    };
}
galaw::for_each_generated_robot!(codegen_correctness_test);

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
    simple_arm_2dof  => "assets/urdf/custom/simple_arm_2dof.urdf",
    simple_arm_3dof_rrp => "assets/urdf/custom/simple-arm_3dof_rrp.urdf",   // Tests revolute and prismatic

    // Third-party robots
    flexiv_enlight_l => "assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",  // Tests revolute and fixed
    anymal_d => "assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",     // Tests revolute and fixed
    wuji_hand_v1_right => "assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",  // Tests revolute and fixed
    stretch4 => "assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",     // Tests continuous, prismatic, revolute, fixed
}
