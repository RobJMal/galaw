use std::collections::{HashMap, HashSet, VecDeque};
use std::{fs};

// Third-party
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

// Custom
use crate::types::{GalawModel, Joint, Link};
use crate::utils::parse_vec3_str;

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
            let link_name: String = node
                .attribute("name")
                .ok_or("link missing name attribute")?
                .to_string();
            links.push(Link { name: link_name });
        } else if node.tag_name().name() == "joint" {
            let name: String = node
                .attribute("name")
                .ok_or("joint missing name attribute")?
                .to_string();
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
            let axis_str: &str = node
                .children()
                .find(|n| n.tag_name().name() == "axis")
                .and_then(|n| n.attribute("xyz"))
                .ok_or_else(|| format!("missing axis xyz value for joint {}", name))?;
            let (axis_x, axis_y, axis_z) = parse_vec3_str(axis_str)?;
            let axis = Unit::new_normalize(Vector3::new(axis_x, axis_y, axis_z));

            // Extracting joint limits
            let joint_limit = node
                .children()
                .find(|n| n.tag_name().name() == "limit")
                .ok_or_else(|| format!("missing joint limits for joint {}", name))?;

            let limit_lower: f64 = joint_limit
                .attribute("lower")
                .ok_or_else(|| format!("missing joint limit lower for joint {}", name))?
                .parse::<f64>()?;
            let limit_upper: f64 = joint_limit
                .attribute("upper")
                .ok_or_else(|| format!("missing joint limit upper for joint {}", name))?
                .parse::<f64>()?;

            // Creating joint
            let joint: Joint = Joint {
                name,
                parent,
                parent_link_idx: 0, // Dummy as it will be handled in next lines
                child,
                child_link_idx: 1, // Dummy as it will be handled in next lines
                transform,
                axis,
                limit_lower,
                limit_upper,
                cmd_idx: joints.len(),
            };
            joints.push(joint);
        }
    }

    // Enforcing order to ensure indexing is accurate
    let link_index: HashMap<&str, usize> = links
        .iter()
        .enumerate()
        .map(|(i, l)| (l.name.as_str(), i))
        .collect();

    // Find the root
    let child_names: HashSet<&str> = joints.iter().map(|j| j.child.as_str()).collect();
    let root_candidates: Vec<usize> = links
        .iter()
        .enumerate()
        .filter(|(_, l)| !child_names.contains(l.name.as_str()))
        .map(|(i, _)| i)
        .collect();
    let root_index = match root_candidates.as_slice() {
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
    let mut queue: VecDeque<usize> = VecDeque::from([root_index]);
    while let Some(parent_idx) = queue.pop_front() {
        for j in joints
            .iter()
            .filter(|j| link_index[j.parent.as_str()] == parent_idx)
        {
            let child_idx = link_index[j.child.as_str()];
            let mut resolved = j.clone();
            resolved.parent_link_idx = parent_idx;
            resolved.child_link_idx = child_idx;
            ordered_joints.push(resolved);
            queue.push_back(child_idx);
        }
    }

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
        .map(|j| (j.name.clone(), j.cmd_idx))
        .collect();

    Ok(GalawModel {
        name: robot_name,
        links: links,
        link_name_to_idx: link_name_to_idx,
        joints: ordered_joints,
        joint_name_to_idx: joint_name_to_idx,
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
            "assets/simple_robot.urdf",
            "assets/simple_robot_flipped.urdf",
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
        let original = load_urdf("assets/simple_robot.urdf").unwrap();
        let flipped = load_urdf("assets/simple_robot_flipped.urdf").unwrap();

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
