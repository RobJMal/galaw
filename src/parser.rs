use std::collections::{HashMap, HashSet};
use std::fs;

// Third-party
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

// Custom
use crate::types::{GalawModel, Joint, JointType, Link};
use crate::utils::parse_vec3_str;

// ----- HELPER METHODS -----
/// Parses the axis information from tag
fn read_axis(
    node: roxmltree::Node<'_, '_>,
    joint_name: &str,
) -> Result<Unit<Vector3<f64>>, Box<dyn std::error::Error>> {
    // Extracting axis angles
    let axis_str: &str = node
        .children()
        .find(|n| n.tag_name().name() == "axis")
        .and_then(|n| n.attribute("xyz"))
        .ok_or_else(|| format!("missing axis xyz value for joint {}", joint_name))?;
    let (axis_x, axis_y, axis_z) = parse_vec3_str(axis_str)?;

    Ok(Unit::new_normalize(Vector3::new(axis_x, axis_y, axis_z)))
}

/// Parses the joint limit information
///
/// Returns (limit_lower, limit_upper)
fn read_joint_limits(
    node: roxmltree::Node<'_, '_>,
    joint_name: &str,
) -> Result<(f64, f64), Box<dyn std::error::Error>> {
    let joint_limit = node
        .children()
        .find(|n| n.tag_name().name() == "limit")
        .ok_or_else(|| format!("missing joint limits for joint {}", joint_name))?;

    let limit_lower: f64 = joint_limit
        .attribute("lower")
        .ok_or_else(|| format!("missing joint limit lower for joint {}", joint_name))?
        .parse::<f64>()?;
    let limit_upper: f64 = joint_limit
        .attribute("upper")
        .ok_or_else(|| format!("missing joint limit upper for joint {}", joint_name))?
        .parse::<f64>()?;

    Ok((limit_lower, limit_upper))
}

/// Parses <link> tag into a `Link`
fn parse_link(node: roxmltree::Node<'_, '_>) -> Result<Link, Box<dyn std::error::Error>> {
    let link_name: String = node
        .attribute("name")
        .ok_or("link missing name attribute")?
        .to_string();
    Ok(Link { name: link_name })
}

/// Parses <joint> tag into a `Joint`
fn parse_joint(node: roxmltree::Node<'_, '_>) -> Result<Joint, Box<dyn std::error::Error>> {
    let name: String = node
        .attribute("name")
        .ok_or("joint missing name attribute")?
        .to_string();
    let joint_type: JointType = node
        .attribute("type")
        .ok_or_else(|| format!("joint {} missing type attribute", name))?
        .parse()
        .map_err(|e| format!("joint {}: {}", name, e))?;
    let parent: String = node
        .children()
        .find(|n| n.tag_name().name() == "parent")
        .and_then(|n| n.attribute("link"))
        .ok_or_else(|| format!("missing parent link for joint {}", name))?
        .to_string();
    let child: String = node
        .children()
        .find(|n| n.tag_name().name() == "child")
        .and_then(|n| n.attribute("link"))
        .ok_or_else(|| format!("missing child link for joint {}", name))?
        .to_string();

    // Extracting joint XYZ and RPY
    let joint_origin = node
        .children()
        .find(|n| n.tag_name().name() == "origin")
        .ok_or_else(|| format!("missing origin for joint {}", name))?;

    let xyz_str: &str = joint_origin
        .attribute("xyz")
        .ok_or_else(|| format!("missing xyz for joint {}", name))?;
    let (x, y, z) = parse_vec3_str(xyz_str)?;
    let xyz = Vector3::new(x, y, z);

    let rpy_str = joint_origin
        .attribute("rpy")
        .ok_or_else(|| format!("missing rpy for joint {}", name))?;
    let (roll, pitch, yaw) = parse_vec3_str(rpy_str)?;
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);

    let transform = Isometry3::from_parts(Translation3::from(xyz), rotation);

    // Extracting axis angles
    let (rot_axis, lin_axis) = match joint_type {
        JointType::Prismatic => (None, Some(read_axis(node, &name)?)),
        JointType::Revolute | JointType::Continuous => (Some(read_axis(node, &name)?), None),
        JointType::Fixed => (None, None),
    };

    // Extracting joint limits
    let (limit_lower, limit_upper) = match joint_type {
        JointType::Revolute | JointType::Prismatic => {
            let (lower, upper) = read_joint_limits(node, &name)?;
            (Some(lower), Some(upper))
        }
        // Set to 2*PI since continous and no limits (arbitrarily set)
        JointType::Continuous => (
            Some(2.0 * -std::f64::consts::PI),
            Some(2.0 * std::f64::consts::PI),
        ),
        JointType::Fixed => (None, None),
    };

    // Creating joint
    let joint: Joint = Joint {
        name,
        joint_type,
        parent,
        parent_link_idx: 0, // Resolved in resolve_joint_order
        child,
        child_link_idx: 0, // Resolved in resolve_joint_order
        transform,
        lin_axis: lin_axis,
        rot_axis: rot_axis,
        limit_lower: limit_lower,
        limit_upper: limit_upper,
        cmd_idx: None, // Resolved in resolve_joint_order
    };

    Ok(joint)
}

/// Visits the different nodes in DFS
fn dfs_visit(
    link_idx: usize,
    joints: &[Joint],
    link_lookup: &HashMap<&str, usize>,
    children_by_link: &HashMap<usize, Vec<usize>>,
    ordered_joints: &mut Vec<Joint>,
    cmd_counter: &mut usize,
) {
    let Some(child_joint_indices) = children_by_link.get(&link_idx) else {
        return;
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
        );
    }
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
fn resolve_joint_order(
    links: &Vec<Link>,
    joints: &Vec<Joint>,
) -> Result<(Vec<Joint>, HashMap<String, usize>, HashMap<String, usize>), Box<dyn std::error::Error>>
{
    // Enforcing order to ensure indexing is accurate
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
    // Find the root
    let child_names: HashSet<&str> = joints.iter().map(|j| j.child.as_str()).collect();
    let root_candidates: Vec<usize> = links
        .iter()
        .enumerate()
        .filter(|(_, l)| !child_names.contains(l.name.as_str()))
        .map(|(i, _)| i)
        .collect();
    let root_idx = match root_candidates.as_slice() {
        [single] => *single,
        [] => return Err("no root link found - every link has a parent (cycle in URDF?)".into()),
        _ => {
            let names: Vec<&str> = root_candidates
                .iter()
                .map(|&i| links[i].name.as_str())
                .collect();
            return Err(format!(
                "multiple root-like links with no parent: {:?} - URDF may be disconnected",
                names
            )
            .into());
        }
    };

    // Walk the tree from root, resolving parent/child link indices
    let mut ordered_joints: Vec<Joint> = Vec::with_capacity(joints.len());
    let mut acutated_joint_counter = 0;
    dfs_visit(
        root_idx,
        joints,
        &link_lookup,
        &children_by_link,
        &mut ordered_joints,
        &mut acutated_joint_counter,
    );

    if ordered_joints.len() != joints.len() {
        return Err(
            "some joints are unreachable from root link (disconnected or cyclic URDF)".into(),
        );
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

    Ok((ordered_joints, link_name_to_idx, joint_name_to_idx))
}

/// Parses a URDF file into a `GalawModel`.
///
/// After XML parsing, it resolves the joint order via Breadth-First Search (BFS)
/// from the root so `compute_fk` can trust indices instead of file order.
pub fn load_urdf(urdf_path: &str) -> Result<GalawModel, Box<dyn std::error::Error>> {
    let content: String = fs::read_to_string(urdf_path)?;
    let doc = roxmltree::Document::parse(&content)?;

    let robot_name: String = doc
        .root_element()
        .attribute("name")
        .ok_or("missing robot name")?
        .to_string();
    let mut links: Vec<Link> = Vec::new();
    let mut joints: Vec<Joint> = Vec::new();

    for node in doc.descendants() {
        if node.tag_name().name() == "link" {
            let link = parse_link(node)?;
            links.push(link);
        } else if node.tag_name().name() == "joint" {
            let joint = parse_joint(node)?;
            joints.push(joint);
        }
    }

    let (ordered_joints, link_name_to_idx, joint_name_to_idx) =
        resolve_joint_order(&links, &joints)?;

    let num_actuated_joints = ordered_joints
        .iter()
        .filter(|j| j.cmd_idx.is_some())
        .count();

    Ok(GalawModel {
        name: robot_name,
        links: links,
        link_name_to_idx: link_name_to_idx,
        joints: ordered_joints,
        joint_name_to_idx: joint_name_to_idx,
        num_actuated_joints: num_actuated_joints,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// get_link_idx must resolve to the right link, for every link, in
    /// both file orderings — guards against index/link mappings drifting
    /// out of sync (not needed to catch today's bug, but the same bug
    /// class as the joint resolution issue below).
    #[test]
    fn link_lookup_is_self_consistent() {
        for path in [
            "assets/urdf/custom/simple_arm_2dof.urdf",
            "assets/urdf/custom/simple_arm_2dof_flipped.urdf",
        ] {
            let model = load_urdf(path).unwrap();
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
    fn joint_resolution_is_independent_of_file_order() {
        let original = load_urdf("assets/urdf/custom/simple_arm_2dof.urdf").unwrap();
        let flipped = load_urdf("assets/urdf/custom/simple_arm_2dof_flipped.urdf").unwrap();

        // Resolve a joint's parent/child *link names* (not raw indices —
        // those are expected to differ between the two files, since the
        // links are declared in a different order in each).
        fn parent_child_names(model: &GalawModel, joint_name: &str) -> (String, String) {
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
}
