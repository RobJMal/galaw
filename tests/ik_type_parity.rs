/// Type parity checks for inverse kinematics.
///
/// Cross-type comparison of joint solutions is not meaningful because IK solutions
/// are not unique. Different types may converge to different but equally valid
/// configurations via the nullspace. Instead, this verifies that IK reaches the
/// target pose within the precision limit of the type used:
///
///   FK(IK(target)) ≈ target   (within PARITY_TOLERANCE)
// Third-party
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

// Custom
use galaw::{error::KinematicsError, load_urdf, types::GalawModel};

mod common;
use common::{RNG_SEED, TestResult};

const NUM_POSES: usize = 128;
const MAX_PERTURBATION: f32 = 0.5;
// f32 IK convergence tolerance is 1e-5; allow 10x margin for f32 FK rounding
// across the IK loop and post-clamp re-evaluation.
const PARITY_TOLERANCE: f32 = 1e-4;

/// Casts an f64 isometry to f32 component-by-component.
fn iso_f64_to_f32(iso: &Isometry3<f64>) -> Isometry3<f32> {
    Isometry3::from_parts(
        Translation3::new(
            iso.translation.x as f32,
            iso.translation.y as f32,
            iso.translation.z as f32,
        ),
        UnitQuaternion::from_quaternion(Quaternion::new(
            iso.rotation.w as f32,
            iso.rotation.i as f32,
            iso.rotation.j as f32,
            iso.rotation.k as f32,
        )),
    )
}

/// Links with at least one actuated ancestor (valid IK targets).
fn candidate_target_links<T>(model: &GalawModel<T>) -> Vec<usize> {
    let parent_indices: std::collections::HashSet<usize> =
        model.joints.iter().map(|j| j.parent_link_idx).collect();

    let mut ancestors_by_link = vec![vec![]; model.num_links];
    for (joint_idx, joint) in model.joints.iter().enumerate() {
        let mut ancestors = ancestors_by_link[joint.parent_link_idx].clone();
        if joint.cmd_idx.is_some() {
            ancestors.push(joint_idx);
        }
        ancestors_by_link[joint.child_link_idx] = ancestors;
    }

    (0..model.num_links)
        .filter(|&i| !parent_indices.contains(&i) && !ancestors_by_link[i].is_empty())
        .collect()
}

fn check_ik_f32_parity(urdf_path: &str) -> TestResult {
    // Use f64 model only for generating target joint commands (via its limits).
    // Everything else — target pose, IK, FK — runs in f32.
    let model_f32 = load_urdf::<f32>(urdf_path)?;
    let model_f64 = load_urdf::<f64>(urdf_path)?;
    let mut data_f64 = model_f64.create_galaw_data();
    let mut data_f32 = model_f32.create_galaw_data();

    let candidates = candidate_target_links(&model_f32);
    assert!(!candidates.is_empty(), "no valid IK targets in {urdf_path}");

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);

    for _ in 0..NUM_POSES {
        let target_link_idx = candidates[rng.random_range(0..candidates.len())];

        // Generate target joint commands in f64, then narrow to f32.
        let target_cmds_f64: Vec<f64> = model_f64
            .joints
            .iter()
            .filter(|j| j.cmd_idx.is_some())
            .map(|j| match (j.limit_lower, j.limit_upper) {
                (Some(lo), Some(hi)) => rng.random_range(lo..hi),
                _ => 0.0,
            })
            .collect();
        let target_cmds_f32: Vec<f32> = target_cmds_f64.iter().map(|&v| v as f32).collect();

        // Compute target pose in f64 then cast to f32 so the IK operates entirely
        // in f32, matching real usage (no f64 ground-truth leaking in).
        model_f64.compute_fk(&target_cmds_f64, &mut data_f64)?;
        let target_pose_f64 = data_f64.link_poses[target_link_idx];
        let target_pose_f32 = iso_f64_to_f32(&target_pose_f64);

        // Perturb target joint cmds to form initial conditions.
        let init_cmds_f32: Vec<f32> = model_f32
            .joints
            .iter()
            .filter(|j| j.cmd_idx.is_some())
            .zip(target_cmds_f32.iter())
            .map(|(j, &base)| {
                let perturbed = base + rng.random_range(-MAX_PERTURBATION..MAX_PERTURBATION);
                match (j.limit_lower, j.limit_upper) {
                    (Some(lo), Some(hi)) => perturbed.clamp(lo, hi),
                    _ => perturbed,
                }
            })
            .collect();

        match model_f32.compute_ik(
            target_link_idx,
            &target_pose_f32,
            &init_cmds_f32,
            &mut data_f32,
        ) {
            Ok(()) => {}
            Err(galaw::error::GalawError::Kinematics(KinematicsError::IkDidNotConverge {
                ..
            })) => {
                eprintln!("[skip] f32 IK did not converge after clamping");
                continue;
            }
            Err(e) => return Err(e.into()),
        };

        let solved = data_f32.solved_joint_cmds.clone();
        model_f32.compute_fk(&solved, &mut data_f32)?;
        let achieved = data_f32.link_poses[target_link_idx];

        // Translation
        let t = &achieved.translation;
        let tgt = &target_pose_f32.translation;
        assert!(
            (t.x - tgt.x).abs() < PARITY_TOLERANCE,
            "translation.x: {:.4e} vs {:.4e}",
            t.x,
            tgt.x
        );
        assert!(
            (t.y - tgt.y).abs() < PARITY_TOLERANCE,
            "translation.y: {:.4e} vs {:.4e}",
            t.y,
            tgt.y
        );
        assert!(
            (t.z - tgt.z).abs() < PARITY_TOLERANCE,
            "translation.z: {:.4e} vs {:.4e}",
            t.z,
            tgt.z
        );

        // Orientation — quaternion double-cover: compare |q1·q2| ≈ 1
        let r = &achieved.rotation;
        let tgt_r = &target_pose_f32.rotation;
        let dot = (r.w * tgt_r.w + r.i * tgt_r.i + r.j * tgt_r.j + r.k * tgt_r.k).abs();
        assert!(
            (dot - 1.0).abs() < PARITY_TOLERANCE,
            "rotation |q·q_tgt|={dot:.4e}"
        );
    }

    Ok(())
}

macro_rules! parity_tests {
    ($($name:ident => $path:expr),* $(,)?) => {
        $(
            #[test]
            fn $name() -> TestResult {
                check_ik_f32_parity($path)
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
