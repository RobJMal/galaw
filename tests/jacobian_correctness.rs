
// Third-party
use nalgebra::Vector6;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use k;

// Custom
use galaw::load_urdf;

// CONSTANTS
const TEST_TOLERANCE: f64 = 1e-10;
const RNG_SEED: u64 = 42;
const NUM_POSES: usize = 128; // Number of random robot poses to test out

// HELPERS
fn assert_close(a: f64, b: f64) {
    assert!(
        (a - b).abs() < TEST_TOLERANCE,
        "expected {b}, got {a} OR not within {TEST_TOLERANCE}"
    );
}


#[test]
fn jacobian_zero_pose_matches_hand_derivation() {
    let model = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf").unwrap();
    let forearm_idx = model.get_link_idx("forearm").unwrap();
    let joint_cmds = [0.0, 0.0];
    let jacobian = model.compute_jacobian(&joint_cmds, forearm_idx).unwrap();

    assert_eq!(jacobian.column(0), Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    assert_eq!(jacobian.column(1), Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));
}

#[test]
fn jacobian_matches_k_simple_arm_2dof() {
    let urdf_path = "assets/urdf/custom/simple_arm_2dof.urdf";
    let galaw_model = load_urdf(urdf_path).unwrap();
    let k_chain = k::Chain::<f64>::from_urdf_file(urdf_path).unwrap();

    let target_link_name = "forearm";
    let target_link_idx = galaw_model.get_link_idx(target_link_name).unwrap();

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    for _ in 0..NUM_POSES {
        let joint_cmds: Vec<f64> = galaw_model
            .joints
            .iter()
            .filter(|j| j.cmd_idx.is_some())
            .map(|j| match (j.limit_lower, j.limit_upper) {
                (Some(lower), Some(upper)) => rng.random_range(lower..upper),
                _ => 0.0,
            })
            .collect();

        let galaw_jacobian = galaw_model
            .compute_jacobian(&joint_cmds, target_link_idx)
            .unwrap();

        k_chain.set_joint_positions(&joint_cmds).unwrap();
        k_chain.update_transforms();
        let k_serial = k::SerialChain::from_end(k_chain.find_link(target_link_name).unwrap());
        let k_jacobian = k::jacobian(&k_serial);
        
        // k's columns are ordered by its own joints on this chain; galaw's matrix
        // is zero-padded to the full actuated-joint count, so map each of k's cols
        // to galaw's cmd_idx by joint_name rather by raw index. 
        for (k_col_idx, k_joint) in k_serial.iter_joints().enumerate() {
            let cmd_idx = galaw_model.get_joint_idx(&k_joint.name).unwrap();
            for row in 0..6 {
                assert_close(galaw_jacobian[(row, cmd_idx)], k_jacobian[(row, k_col_idx)]);
            }
        }
    }
}