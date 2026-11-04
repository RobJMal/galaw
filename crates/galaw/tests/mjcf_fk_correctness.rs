/// Tests that `load_mjcf` + `compute_fk` produce link poses that agree with
/// the equivalent URDF model to within floating-point tolerance.
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use galaw::{load_mjcf, load_urdf};

mod common;
use common::{RNG_SEED, TestResult, assert_galaw_transform_close, random_joint_cmds};

const NUM_POSES: usize = 100;
const TOLERANCE: f64 = 1e-9;

/// Verifies that FK from an MJCF model matches FK from the equivalent URDF
/// for the same joint commands.  Link poses are matched by name.
fn check_mjcf_matches_urdf(urdf_path: &str, mjcf_path: &str) -> TestResult {
    let urdf_model = load_urdf::<f64>(urdf_path).unwrap();
    let mjcf_model = load_mjcf::<f64>(mjcf_path).unwrap();

    assert_eq!(
        urdf_model.num_actuated_joints, mjcf_model.num_actuated_joints,
        "URDF and MJCF models have different actuated joint counts"
    );

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    for _ in 0..NUM_POSES {
        // random_joint_cmds iterates joints in cmd_idx order, so the same
        // limits on the same joints produce equivalent commands for both models.
        let cmds = random_joint_cmds(&urdf_model, &mut rng);

        let mut urdf_data = urdf_model.create_galaw_data();
        urdf_model.compute_fk(&cmds, &mut urdf_data).unwrap();

        let mut mjcf_data = mjcf_model.create_galaw_data();
        mjcf_model.compute_fk(&cmds, &mut mjcf_data).unwrap();

        // Compare poses for every link that exists in both models by name.
        for link in &urdf_model.links {
            let Some(urdf_idx) = urdf_model.get_link_idx(&link.name) else {
                continue;
            };
            let Some(mjcf_idx) = mjcf_model.get_link_idx(&link.name) else {
                // MJCF model may have extra virtual links; that is fine.
                continue;
            };
            assert_galaw_transform_close(
                &urdf_data.link_poses[urdf_idx],
                &mjcf_data.link_poses[mjcf_idx],
                &TOLERANCE,
            );
        }
    }
    Ok(())
}

#[test]
fn mjcf_fk_matches_urdf_simple_arm_2dof() -> TestResult {
    check_mjcf_matches_urdf(
        "assets/urdf/custom/simple_arm_2dof.urdf",
        "assets/mjcf/custom/simple_arm_2dof.xml",
    )
}

#[test]
fn mjcf_fk_matches_urdf_simple_arm_2dof_flipped() -> TestResult {
    check_mjcf_matches_urdf(
        "assets/urdf/custom/simple_arm_2dof.urdf",
        "assets/mjcf/custom/simple_arm_2dof_flipped.xml",
    )
}

/// A minimal smoke test: the MJCF parser reports the right model name and
/// joint/link counts without running FK.
#[test]
fn mjcf_basic_model_properties() {
    let model = load_mjcf::<f64>("assets/mjcf/custom/simple_arm_2dof.xml").unwrap();
    assert_eq!(model.name, "simple_arm_2dof");
    assert_eq!(model.num_actuated_joints, 2);
    assert_eq!(model.num_links, 3);
    assert!(model.get_link_idx("base_link").is_some());
    assert!(model.get_link_idx("upper_arm").is_some());
    assert!(model.get_link_idx("forearm").is_some());
    assert!(model.get_joint_idx("shoulder_joint").is_some());
    assert!(model.get_joint_idx("elbow_joint").is_some());
}

// ── Third-party model smoke tests ─────────────────────────────────────────────
//
// These tests load real Menagerie MJCFs and verify that:
//   1. Parsing succeeds (no errors, all links/joints resolved)
//   2. Expected model metadata is correct
//   3. FK runs without panicking at all-zero joint config
//
// They do NOT compare against a reference URDF (no equivalent exists);
// correctness of the kinematics is validated separately via the custom
// fixtures above.

/// Verifies that FK runs at all-zero joints and returns the right number of
/// link poses.
fn smoke_fk(model: &galaw::types::GalawModel<f64>) {
    let cmds = vec![0.0_f64; model.num_actuated_joints];
    let mut data = model.create_galaw_data();
    model.compute_fk(&cmds, &mut data).unwrap();
    assert_eq!(data.link_poses.len(), model.num_links);
}

#[test]
fn mjcf_loads_spot() {
    let model = load_mjcf::<f64>(
        "assets/mjcf/third_party/Boston-Dynamics_Spot/spot.xml",
    )
    .unwrap();
    assert_eq!(model.name, "spot");
    assert_eq!(model.num_actuated_joints, 12);
    assert!(model.get_link_idx("body").is_some());
    assert!(model.get_joint_idx("fl_hx").is_some());
    smoke_fk(&model);
}

#[test]
fn mjcf_loads_panda() {
    let model = load_mjcf::<f64>(
        "assets/mjcf/third_party/Franka-Emika_Panda/panda.xml",
    )
    .unwrap();
    assert_eq!(model.name, "panda");
    assert_eq!(model.num_actuated_joints, 9);
    assert!(model.get_link_idx("link0").is_some());
    assert!(model.get_joint_idx("joint1").is_some());
    assert!(model.get_joint_idx("finger_joint1").is_some());
    smoke_fk(&model);
}

#[test]
fn mjcf_loads_shadow_hand() {
    let model = load_mjcf::<f64>(
        "assets/mjcf/third_party/Shadow-Robot_Hand/right_hand.xml",
    )
    .unwrap();
    assert_eq!(model.name, "right_shadow_hand");
    assert_eq!(model.num_actuated_joints, 24);
    assert!(model.get_link_idx("rh_forearm").is_some());
    assert!(model.get_joint_idx("rh_WRJ1").is_some());
    smoke_fk(&model);
}

#[test]
fn mjcf_loads_g1() {
    let model = load_mjcf::<f64>(
        "assets/mjcf/third_party/Unitree_G1/g1.xml",
    )
    .unwrap();
    assert_eq!(model.name, "g1_29dof_rev_1_0");
    assert_eq!(model.num_actuated_joints, 29);
    assert!(model.get_link_idx("pelvis").is_some());
    assert!(model.get_joint_idx("left_hip_pitch_joint").is_some());
    smoke_fk(&model);
}

/// Checks the flipped MJCF (joint declared after child body) resolves to the
/// same parent/child link names as the standard ordering.
#[test]
fn mjcf_joint_resolution_independent_of_declaration_order() {
    let normal = load_mjcf::<f64>("assets/mjcf/custom/simple_arm_2dof.xml").unwrap();
    let flipped = load_mjcf::<f64>("assets/mjcf/custom/simple_arm_2dof_flipped.xml").unwrap();

    for joint_name in ["shoulder_joint", "elbow_joint"] {
        let nj = normal.joints.iter().find(|j| j.name == joint_name).unwrap();
        let fj = flipped.joints.iter().find(|j| j.name == joint_name).unwrap();
        assert_eq!(
            normal.links[nj.parent_link_idx].name,
            flipped.links[fj.parent_link_idx].name,
            "joint {joint_name}: parent link name differs between normal and flipped"
        );
        assert_eq!(
            normal.links[nj.child_link_idx].name,
            flipped.links[fj.child_link_idx].name,
            "joint {joint_name}: child link name differs between normal and flipped"
        );
    }
}
