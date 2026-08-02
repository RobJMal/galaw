use galaw::load_urdf;
use nalgebra::Vector6;

#[test]
fn jacobian_zero_pose_matches_hand_derivation() {
    let model = crate::load_urdf("assets/urdf/custom/simple_arm_2dof.urdf").unwrap();
    let forearm_idx = model.get_link_idx("forearm").unwrap();
    let joint_cmds = [0.0, 0.0];
    let jacobian = model.compute_jacobian(&joint_cmds, forearm_idx).unwrap();

    assert_eq!(jacobian.column(0), Vector6::new(0.0, 0.0, 0.0, 0.0, 0.0, 1.0));
    assert_eq!(jacobian.column(1), Vector6::new(0.0, 0.0, 0.0, 0.0, 1.0, 0.0));
}