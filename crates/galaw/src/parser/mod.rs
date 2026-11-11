use std::collections::{HashMap, HashSet};

// Third-party
use nalgebra::RealField;

// Custom
use crate::error::ModelTopologyError;
use crate::types::{Joint, JointType, Link};

mod mjcf;
mod urdf;

pub use mjcf::load_mjcf;
pub use urdf::load_urdf;

// =============================================================================
// SHARED: JOINT ORDER RESOLUTION
// =============================================================================

/// Resolved joints (in DFS order), link name -> index, and joint name -> cmd_idx.
type ResolvedJoints<T> = (
    Vec<Joint<T>>,
    HashMap<String, usize>, // link_name -> link_idx
    HashMap<String, usize>, // joint_name -> cmd_idx
    Vec<Vec<usize>>,        // ancestors_by_link
    Vec<Vec<usize>>,        // chain_by_link
    Vec<T>,                 // joint_limit_lower (indexed by cmd_idx)
    Vec<T>,                 // joint_limit_upper (indexed by cmd_idx)
);

/// Visits the different nodes in DFS
///
/// If there is a link that has been revisited, it returns
/// the index of the link as an error to hint about cycles
/// in kinematic model.
fn dfs_visit<T: RealField + Copy>(
    link_idx: usize,
    joints: &[Joint<T>],
    link_lookup: &HashMap<&str, usize>,
    children_by_link: &HashMap<usize, Vec<usize>>,
    ordered_joints: &mut Vec<Joint<T>>,
    cmd_counter: &mut usize,
    visited: &mut HashSet<usize>,
) -> Result<(), usize> {
    if !visited.insert(link_idx) {
        return Err(link_idx);
    }

    let Some(child_joint_indices) = children_by_link.get(&link_idx) else {
        return Ok(());
    };

    for &joint_idx in child_joint_indices {
        let j = &joints[joint_idx];
        let child_link_idx = link_lookup[j.child.as_str()];

        let mut resolved = j.clone();
        resolved.parent_link_idx = link_idx;
        resolved.child_link_idx = child_link_idx;
        resolved.cmd_idx = if j.joint_type == JointType::Fixed {
            None
        } else {
            let idx = *cmd_counter;
            *cmd_counter += 1;
            Some(idx)
        };
        ordered_joints.push(resolved);

        dfs_visit(
            child_link_idx,
            joints,
            link_lookup,
            children_by_link,
            ordered_joints,
            cmd_counter,
            visited,
        )?;
    }

    Ok(())
}

/// Resolves joint order for downstream functions.
///
/// Resolves the joint order via Depth-First Search (DFS) pre-order from the
/// root, so `compute_fk` can trust indices instead of file-declaration order.
/// DFS is used (not BFS) for two reasons:
///
/// 1. Joints in the same branch end up at consecutive indices (e.g. a robot
///    hand's index-finger joints land at 5, 6, 7, 8 instead of being
///    interleaved with the other fingers' joints).
/// 2. `k::Chain` — this project's own ground-truth for correctness testing —
///    numbers its DOFs via DFS pre-order (confirmed by reading its source).
fn resolve_joint_order<T: RealField + Copy>(
    links: &[Link],
    joints: &[Joint<T>],
) -> Result<ResolvedJoints<T>, ModelTopologyError> {
    let link_lookup: HashMap<&str, usize> = links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.name.as_str(), i))
        .collect();

    let mut children_by_link: HashMap<usize, Vec<usize>> = HashMap::new();
    for (joint_idx, j) in joints.iter().enumerate() {
        let parent_idx = link_lookup[j.parent.as_str()];
        children_by_link
            .entry(parent_idx)
            .or_default()
            .push(joint_idx);
    }

    let child_names: HashSet<&str> = joints.iter().map(|j| j.child.as_str()).collect();
    let root_candidates: Vec<usize> = links
        .iter()
        .enumerate()
        .filter(|(_, l)| !child_names.contains(l.name.as_str()))
        .map(|(i, _)| i)
        .collect();
    let root_idx = match root_candidates.as_slice() {
        [single] => *single,
        [] => return Err(ModelTopologyError::MissingRootLink),
        _ => {
            let names: Vec<String> = root_candidates
                .iter()
                .map(|&i| links[i].name.clone())
                .collect();
            return Err(ModelTopologyError::MultipleRootLinks(names));
        }
    };

    let mut ordered_joints: Vec<Joint<T>> = Vec::with_capacity(joints.len());
    let mut actuated_joint_counter = 0;
    let mut visited: HashSet<usize> = HashSet::new();
    dfs_visit(
        root_idx,
        joints,
        &link_lookup,
        &children_by_link,
        &mut ordered_joints,
        &mut actuated_joint_counter,
        &mut visited,
    )
    .map_err(|err| ModelTopologyError::CyclicLink(links[err].name.clone()))?;

    let visited_joints: HashSet<&str> = ordered_joints.iter().map(|j| j.name.as_str()).collect();
    let disconnected_joints: Vec<String> = joints
        .iter()
        .filter(|j| !visited_joints.contains(j.name.as_str()))
        .map(|j| j.name.clone())
        .collect();
    if !disconnected_joints.is_empty() {
        return Err(ModelTopologyError::DisconnectedJoints(disconnected_joints));
    }

    let link_name_to_idx: HashMap<String, usize> = links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.name.clone(), i))
        .collect();

    let joint_name_to_idx: HashMap<String, usize> = ordered_joints
        .iter()
        .filter_map(|j| j.cmd_idx.map(|idx| (j.name.clone(), idx)))
        .collect();

    // Precompute per-link ancestor tables. Joints are in DFS pre-order,
    // so each parent link's lists are complete before its children are processed.
    let mut ancestors_by_link: Vec<Vec<usize>> = vec![Vec::new(); links.len()];
    let mut chain_by_link: Vec<Vec<usize>> = vec![Vec::new(); links.len()];
    for (joint_idx, joint) in ordered_joints.iter().enumerate() {
        let mut ancestors = ancestors_by_link[joint.parent_link_idx].clone();
        if joint.cmd_idx.is_some() {
            ancestors.push(joint_idx);
        }
        ancestors_by_link[joint.child_link_idx] = ancestors;

        let mut chain = chain_by_link[joint.parent_link_idx].clone();
        chain.push(joint_idx);
        chain_by_link[joint.child_link_idx] = chain;
    }

    let mut joint_limit_lower = vec![T::zero(); actuated_joint_counter];
    let mut joint_limit_upper = vec![T::zero(); actuated_joint_counter];
    for joint in &ordered_joints {
        if let Some(cmd_idx) = joint.cmd_idx {
            joint_limit_lower[cmd_idx] = joint.limit_lower.unwrap_or(T::zero());
            joint_limit_upper[cmd_idx] = joint.limit_upper.unwrap_or(T::zero());
        }
    }

    Ok((
        ordered_joints,
        link_name_to_idx,
        joint_name_to_idx,
        ancestors_by_link,
        chain_by_link,
        joint_limit_lower,
        joint_limit_upper,
    ))
}

// =============================================================================
// TESTS
// =============================================================================

#[cfg(test)]
mod tests {
    use nalgebra::Isometry3;

    use crate::error::ModelTopologyError;
    use crate::types::{GalawModel, JointType, Link};

    use super::{load_mjcf, load_urdf};

    // ---- URDF ----

    /// get_link_idx must resolve to the right link, for every link, in
    /// both file orderings — guards against index/link mappings drifting
    /// out of sync.
    #[test]
    fn urdf_link_lookup_is_self_consistent() {
        for path in [
            "assets/urdf/custom/simple_arm_2dof.urdf",
            "assets/urdf/custom/simple_arm_2dof_flipped.urdf",
        ] {
            let model = load_urdf::<f64>(path).unwrap();
            for link in &model.links {
                let idx = model.get_link_idx(&link.name).unwrap_or_else(|| {
                    panic!("get_link_idx(\"{}\") returned None in {path}", link.name)
                });
                assert_eq!(
                    model.links[idx].name, link.name,
                    "get_link_idx(\"{}\") in {path} pointed at the wrong link",
                    link.name
                );
            }
        }
    }

    /// Same robot, links/joints in a different file order — resolved
    /// parent/child link names must match regardless.
    #[test]
    fn urdf_joint_resolution_is_independent_of_file_order() {
        let original = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf").unwrap();
        let flipped = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof_flipped.urdf").unwrap();

        fn parent_child_names<T: nalgebra::RealField + Copy>(
            model: &GalawModel<T>,
            joint_name: &str,
        ) -> (String, String) {
            let joint = model.joints.iter().find(|j| j.name == joint_name).unwrap();
            (
                model.links[joint.parent_link_idx].name.clone(),
                model.links[joint.child_link_idx].name.clone(),
            )
        }

        for joint_name in ["shoulder_joint", "elbow_joint"] {
            assert_eq!(
                parent_child_names(&original, joint_name),
                parent_child_names(&flipped, joint_name),
                "joint {joint_name} resolved to a different parent/child link depending on file order"
            );
        }
    }

    // ---- MJCF ----

    /// MJCF parser reports the right model name, joint/link counts, and
    /// index lookups — no FK involved, purely parser output validation.
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

    /// Flipped MJCF (joint declared after child body) resolves to the
    /// same parent/child link names as the standard ordering.
    #[test]
    fn mjcf_joint_resolution_is_independent_of_declaration_order() {
        let normal = load_mjcf::<f64>("assets/mjcf/custom/simple_arm_2dof.xml").unwrap();
        let flipped = load_mjcf::<f64>("assets/mjcf/custom/simple_arm_2dof_flipped.xml").unwrap();

        fn parent_child_names<T: nalgebra::RealField + Copy>(
            model: &GalawModel<T>,
            joint_name: &str,
        ) -> (String, String) {
            let joint = model.joints.iter().find(|j| j.name == joint_name).unwrap();
            (
                model.links[joint.parent_link_idx].name.clone(),
                model.links[joint.child_link_idx].name.clone(),
            )
        }

        for joint_name in ["shoulder_joint", "elbow_joint"] {
            assert_eq!(
                parent_child_names(&normal, joint_name),
                parent_child_names(&flipped, joint_name),
                "joint {joint_name}: parent/child link differs between normal and flipped MJCF"
            );
        }
    }

    /// Creates graph ROOT -> A -> B -> A. This ensures that dfs_visit
    /// does not have unbounded recursion error.
    #[test]
    fn resolve_joint_order_detects_reachable_cycle() {
        use crate::types::Joint;

        fn joint(name: &str, parent: &str, child: &str) -> Joint<f64> {
            Joint {
                name: name.to_string(),
                joint_type: JointType::Fixed,
                parent: parent.to_string(),
                parent_link_idx: 0,
                child: child.to_string(),
                child_link_idx: 0,
                transform: Isometry3::identity(),
                lin_axis: None,
                rot_axis: None,
                limit_lower: None,
                limit_upper: None,
                cmd_idx: None,
            }
        }

        let links = vec![
            Link {
                name: "ROOT".to_string(),
            },
            Link {
                name: "A".to_string(),
            },
            Link {
                name: "B".to_string(),
            },
        ];
        let joints = vec![
            joint("j1", "ROOT", "A"),
            joint("j2", "A", "B"),
            joint("j3", "B", "A"),
        ];

        let result = super::resolve_joint_order(&links, &joints);
        assert!(
            matches!(&result, Err(ModelTopologyError::CyclicLink(name)) if name == "A"),
            "expected CyclicLink(\"A\"), got {result:?}"
        );
    }
}
