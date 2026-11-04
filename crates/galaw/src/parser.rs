use std::collections::{HashMap, HashSet};
use std::fs;
use std::str::FromStr;

// Third-party
use nalgebra::{Isometry3, Matrix3, Quaternion, RealField, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};

// Custom
use crate::error::{GalawError, MjcfParseError, ModelTopologyError, UrdfParseError};
use crate::types::{GalawModel, Joint, JointType, Link};
use crate::utils::parse_vec3_str;

// ----- HELPER METHODS -----
/// Parses the axis information from tag
fn read_axis<T>(
    node: roxmltree::Node<'_, '_>,
    joint_name: &str,
) -> Result<Unit<Vector3<T>>, UrdfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    // Extracting axis angles
    let axis_str: &str = node
        .children()
        .find(|n| n.tag_name().name() == "axis")
        .and_then(|n| n.attribute("xyz"))
        .ok_or_else(|| UrdfParseError::MissingAttributeJointAxisXyz(joint_name.to_string()))?;
    let (axis_x, axis_y, axis_z) = parse_vec3_str::<T>(axis_str)?;

    Ok(Unit::new_normalize(Vector3::new(axis_x, axis_y, axis_z)))
}

/// Parses the joint limit information
///
/// Returns (limit_lower, limit_upper)
fn read_joint_limits<T>(
    node: roxmltree::Node<'_, '_>,
    joint_name: &str,
) -> Result<(T, T), UrdfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let joint_limit = node
        .children()
        .find(|n| n.tag_name().name() == "limit")
        .ok_or_else(|| UrdfParseError::MissingTagJointLimit(joint_name.to_string()))?;

    // Parsing lower joint limits
    let limit_lower_str: &str = joint_limit
        .attribute("lower")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointLimitLower(joint_name.to_string()))?;
    let limit_lower: T =
        limit_lower_str
            .parse::<T>()
            .map_err(|source| UrdfParseError::InvalidNumberFormat {
                value: limit_lower_str.to_string(),
                source,
            })?;

    // Parsing upper joint limits
    let limit_upper_str: &str = joint_limit
        .attribute("upper")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointLimitUpper(joint_name.to_string()))?;
    let limit_upper: T =
        limit_upper_str
            .parse::<T>()
            .map_err(|source| UrdfParseError::InvalidNumberFormat {
                value: limit_upper_str.to_string(),
                source,
            })?;

    Ok((limit_lower, limit_upper))
}

/// Parses <link> tag into a `Link`
fn parse_link(node: roxmltree::Node<'_, '_>) -> Result<Link, UrdfParseError> {
    let link_name: String = node
        .attribute("name")
        .ok_or(UrdfParseError::MissingAttributeLinkName)?
        .to_string();
    Ok(Link { name: link_name })
}

/// Parses <joint> tag into a `Joint`
fn parse_joint<T>(node: roxmltree::Node<'_, '_>) -> Result<Joint<T>, UrdfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let name: String = node
        .attribute("name")
        .ok_or(UrdfParseError::MissingAttributeJointName)?
        .to_string();
    let joint_type: JointType = node
        .attribute("type")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointType(name.clone()))?
        .parse()
        .map_err(|found| UrdfParseError::UnknownJointType {
            name: name.clone(),
            found,
        })?;

    // Extracting parent info
    let joint_parent: roxmltree::Node<'_, '_> = node
        .children()
        .find(|n| n.tag_name().name() == "parent")
        .ok_or_else(|| UrdfParseError::MissingTagJointParent(name.clone()))?;
    let parent: String = joint_parent
        .attribute("link")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointParentLink(name.clone()))?
        .to_string();

    // Extracting child info
    let joint_child: roxmltree::Node<'_, '_> = node
        .children()
        .find(|n| n.tag_name().name() == "child")
        .ok_or_else(|| UrdfParseError::MissingTagJointChild(name.clone()))?;
    let child: String = joint_child
        .attribute("link")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointChildLink(name.clone()))?
        .to_string();

    // Extracting joint XYZ and RPY and constructing transform
    let joint_origin = node
        .children()
        .find(|n| n.tag_name().name() == "origin")
        .ok_or_else(|| UrdfParseError::MissingTagJointOrigin(name.clone()))?;

    let xyz_str: &str = joint_origin
        .attribute("xyz")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointOriginXyz(name.clone()))?;
    let (x, y, z) = parse_vec3_str::<T>(xyz_str)?;
    let xyz = Vector3::new(x, y, z);

    let rpy_str = joint_origin
        .attribute("rpy")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointOriginRpy(name.clone()))?;
    let (roll, pitch, yaw) = parse_vec3_str::<T>(rpy_str)?;
    let rotation = UnitQuaternion::from_euler_angles(roll, pitch, yaw);

    let transform = Isometry3::from_parts(Translation3::from(xyz), rotation);

    // Extracting axis angles
    let (rot_axis, lin_axis) = match joint_type {
        JointType::Prismatic => (None, Some(read_axis::<T>(node, &name)?)),
        JointType::Revolute | JointType::Continuous => (Some(read_axis::<T>(node, &name)?), None),
        JointType::Fixed => (None, None),
    };

    // Extracting joint limits
    let (limit_lower, limit_upper) = match joint_type {
        JointType::Revolute | JointType::Prismatic => {
            let (lower, upper) = read_joint_limits::<T>(node, &name)?;
            (Some(lower), Some(upper))
        }
        // Set to 2*PI since continuous has no limits (arbitrarily set)
        JointType::Continuous => {
            let two_pi: T = nalgebra::convert(2.0 * std::f64::consts::PI);
            (Some(-two_pi), Some(two_pi))
        }
        JointType::Fixed => (None, None),
    };

    // Creating joint
    let joint: Joint<T> = Joint {
        name,
        joint_type,
        parent,
        parent_link_idx: 0, // Resolved in resolve_joint_order
        child,
        child_link_idx: 0, // Resolved in resolve_joint_order
        transform,
        lin_axis,
        rot_axis,
        limit_lower,
        limit_upper,
        cmd_idx: None, // Resolved in resolve_joint_order
    };

    Ok(joint)
}

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
    // Checks if we visited before
    if !visited.insert(link_idx) {
        return Err(link_idx);
    }

    // Retrive the link's children
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
        [] => return Err(ModelTopologyError::MissingRootLink),
        _ => {
            let names: Vec<String> = root_candidates
                .iter()
                .map(|&i| links[i].name.clone())
                .collect();
            return Err(ModelTopologyError::MultipleRootLinks(names));
        }
    };

    // Walk the tree from root, resolving parent/child link indices
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

    // Find the disconnected joints
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

/// Parses a URDF file into a [`GalawModel`].
///
/// After XML parsing, it resolves the joint order via Depth-First Search (DFS)
/// from the root so `compute_fk` can trust indices instead of file order.
///
/// # Examples
///
/// ```
/// # fn main() -> Result<(), galaw::error::GalawError<f64>> {
/// let model = galaw::load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf")?;
/// assert_eq!(model.name, "simple_arm_2dof");
/// # Ok(())
/// # }
/// ```
pub fn load_urdf<T>(urdf_path: &str) -> Result<GalawModel<T>, GalawError<T>>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError> + std::fmt::Display,
{
    let content: String = fs::read_to_string(urdf_path).map_err(|err| UrdfParseError::Io {
        path: urdf_path.to_string(),
        source: err,
    })?;
    let doc = roxmltree::Document::parse(&content).map_err(|err| UrdfParseError::XmlParse {
        path: urdf_path.to_string(),
        source: err,
    })?;

    let robot_name: String = doc
        .root_element()
        .attribute("name")
        .ok_or(UrdfParseError::MissingAttributeRobotName)?
        .to_string();
    let mut links: Vec<Link> = Vec::new();
    let mut joints: Vec<Joint<T>> = Vec::new();

    for node in doc.descendants() {
        if node.tag_name().name() == "link" {
            let link = parse_link(node)?;
            links.push(link);
        } else if node.tag_name().name() == "joint" {
            let joint = parse_joint::<T>(node)?;
            joints.push(joint);
        }
    }

    let (
        ordered_joints,
        link_name_to_idx,
        joint_name_to_idx,
        ancestors_by_link,
        chain_by_link,
        joint_limit_lower,
        joint_limit_upper,
    ) = resolve_joint_order(&links, &joints)?;

    Ok(GalawModel {
        name: robot_name,
        num_links: links.len(),
        links,
        link_name_to_idx,
        joints: ordered_joints,
        joint_name_to_idx,
        num_actuated_joints: joint_limit_lower.len(),
        ancestors_by_link,
        chain_by_link,
        joint_limit_lower,
        joint_limit_upper,
    })
}

// =============================================================================
// MJCF PARSER
// =============================================================================

/// Settings from the MJCF `<compiler>` element that affect how attributes are
/// interpreted throughout the rest of the file.
struct CompilerSettings {
    /// True when angles are in degrees (the MJCF default); false for radians.
    angle_in_degrees: bool,
    /// Intrinsic Euler-angle sequence (e.g. `"xyz"`). Default `"xyz"`.
    eulerseq: String,
}

impl Default for CompilerSettings {
    fn default() -> Self {
        Self {
            angle_in_degrees: true,
            eulerseq: "xyz".to_string(),
        }
    }
}

impl CompilerSettings {
    fn to_radians<T: RealField + Copy>(&self, angle: T) -> T {
        if self.angle_in_degrees {
            angle * nalgebra::convert(std::f64::consts::PI / 180.0)
        } else {
            angle
        }
    }
}

/// Per-class joint-attribute defaults from `<default class="..."><joint .../>`.
/// Outer key: class name; inner: attribute name → value string.
type MjcfDefaults = HashMap<String, HashMap<String, String>>;

fn parse_mjcf_defaults(root: roxmltree::Node<'_, '_>) -> MjcfDefaults {
    let mut map: MjcfDefaults = HashMap::new();
    if let Some(def_root) = root.children().find(|n| n.tag_name().name() == "default") {
        collect_default_classes(def_root, "__root__", &mut map);
    }
    map
}

fn collect_default_classes(
    node: roxmltree::Node<'_, '_>,
    class_name: &str,
    map: &mut MjcfDefaults,
) {
    let mut joint_attrs: HashMap<String, String> = HashMap::new();
    for child in node.children() {
        if child.tag_name().name() == "joint" {
            for attr in child.attributes() {
                joint_attrs.insert(attr.name().to_string(), attr.value().to_string());
            }
        }
    }
    map.insert(class_name.to_string(), joint_attrs);
    for child in node.children() {
        if child.tag_name().name() == "default" {
            if let Some(name) = child.attribute("class") {
                collect_default_classes(child, name, map);
            }
        }
    }
}

/// Returns the effective attributes for a `<joint>` node after applying class
/// defaults and then overriding with the element's own explicit attributes.
fn effective_joint_attrs(
    jnode: roxmltree::Node<'_, '_>,
    defaults: &MjcfDefaults,
) -> HashMap<String, String> {
    let mut attrs: HashMap<String, String> = HashMap::new();
    if let Some(cls) = jnode.attribute("class") {
        if let Some(defs) = defaults.get(cls) {
            attrs.extend(defs.iter().map(|(k, v)| (k.clone(), v.clone())));
        }
    }
    for attr in jnode.attributes() {
        attrs.insert(attr.name().to_string(), attr.value().to_string());
    }
    attrs
}

/// Parses a whitespace-separated string into a `Vec<T>`.
fn parse_floats_mjcf<T>(s: &str) -> Result<Vec<T>, MjcfParseError>
where
    T: Copy + FromStr<Err = std::num::ParseFloatError>,
{
    s.split_whitespace()
        .map(|tok| {
            tok.parse::<T>().map_err(|source| MjcfParseError::InvalidNumberFormat {
                value: tok.to_string(),
                source,
            })
        })
        .collect()
}

/// Builds an `Isometry3` from the pos/rotation attributes on a body or joint node.
fn parse_mjcf_transform<T>(
    node: roxmltree::Node<'_, '_>,
    settings: &CompilerSettings,
    name: &str,
) -> Result<Isometry3<T>, MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let translation = if let Some(s) = node.attribute("pos") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 3 {
            return Err(MjcfParseError::InvalidVector3Len(format!("{} pos", name), v.len()));
        }
        Translation3::new(v[0], v[1], v[2])
    } else {
        Translation3::identity()
    };

    let rot_count = ["quat", "euler", "axisangle", "xyaxes", "zaxis"]
        .iter()
        .filter(|&&a| node.attribute(a).is_some())
        .count();
    if rot_count > 1 {
        return Err(MjcfParseError::MultipleRotationFormats { body: name.to_string() });
    }

    let rotation: UnitQuaternion<T> = if let Some(s) = node.attribute("quat") {
        // "w x y z"
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 4 {
            return Err(MjcfParseError::InvalidVector4Len(format!("{} quat", name), v.len()));
        }
        UnitQuaternion::new_normalize(Quaternion::new(v[0], v[1], v[2], v[3]))
    } else if let Some(s) = node.attribute("euler") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 3 {
            return Err(MjcfParseError::InvalidVector3Len(format!("{} euler", name), v.len()));
        }
        let (a, b, c) = (settings.to_radians(v[0]), settings.to_radians(v[1]), settings.to_radians(v[2]));
        apply_mjcf_euler(a, b, c, &settings.eulerseq)
    } else if let Some(s) = node.attribute("axisangle") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 4 {
            return Err(MjcfParseError::InvalidVector4Len(format!("{} axisangle", name), v.len()));
        }
        let axis = Unit::new_normalize(Vector3::new(v[0], v[1], v[2]));
        let angle = settings.to_radians(v[3]);
        UnitQuaternion::from_axis_angle(&axis, angle)
    } else if let Some(s) = node.attribute("xyaxes") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 6 {
            return Err(MjcfParseError::InvalidVector4Len(
                format!("{} xyaxes (expected 6 values)", name),
                v.len(),
            ));
        }
        let xv = Vector3::new(v[0], v[1], v[2]);
        let yv = Vector3::new(v[3], v[4], v[5]);
        let zv = xv.cross(&yv);
        let mat = Matrix3::from_columns(&[xv, yv, zv]);
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(mat))
    } else if let Some(s) = node.attribute("zaxis") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 3 {
            return Err(MjcfParseError::InvalidVector3Len(format!("{} zaxis", name), v.len()));
        }
        let z = Unit::new_normalize(Vector3::new(v[0], v[1], v[2]));
        UnitQuaternion::rotation_between_axis(&Vector3::z_axis(), &z).unwrap_or_else(|| {
            UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                nalgebra::convert(std::f64::consts::PI),
            )
        })
    } else {
        UnitQuaternion::identity()
    };

    Ok(Isometry3::from_parts(translation, rotation))
}

/// Intrinsic Euler rotation: apply each angle around the corresponding body-frame axis in
/// left-to-right order (e.g. "xyz" → Rx(a) * Ry(b) * Rz(c)).
fn apply_mjcf_euler<T: RealField + Copy>(a: T, b: T, c: T, seq: &str) -> UnitQuaternion<T> {
    let chars: Vec<char> = seq.chars().take(3).collect();
    let angles = [a, b, c];
    let mut q = UnitQuaternion::identity();
    for (&ch, &ang) in chars.iter().zip(angles.iter()) {
        let r = match ch {
            'x' | 'X' => UnitQuaternion::from_axis_angle(&Vector3::x_axis(), ang),
            'y' | 'Y' => UnitQuaternion::from_axis_angle(&Vector3::y_axis(), ang),
            'z' | 'Z' => UnitQuaternion::from_axis_angle(&Vector3::z_axis(), ang),
            _ => UnitQuaternion::identity(),
        };
        q = q * r;
    }
    q
}

/// Builds a zero-DOF (Fixed) `Joint` between two links.
fn mjcf_fixed_joint<T: RealField + Copy>(
    name: &str,
    parent: &str,
    child: &str,
    transform: Isometry3<T>,
) -> Joint<T> {
    Joint {
        name: name.to_string(),
        joint_type: JointType::Fixed,
        parent: parent.to_string(),
        parent_link_idx: 0,
        child: child.to_string(),
        child_link_idx: 0,
        transform,
        rot_axis: None,
        lin_axis: None,
        limit_lower: None,
        limit_upper: None,
        cmd_idx: None,
    }
}

/// Parses the joint range; returns `None` for unlimited joints.
/// `is_angular` controls whether the range values are converted from degrees.
fn parse_mjcf_range<T>(
    attrs: &HashMap<String, String>,
    settings: &CompilerSettings,
    is_angular: bool,
) -> Result<Option<(T, T)>, MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    if attrs.get("limited").map(|s| s.as_str()) == Some("false") {
        return Ok(None);
    }
    if let Some(s) = attrs.get("range") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() >= 2 {
            let (lo, hi) = if is_angular {
                (settings.to_radians(v[0]), settings.to_radians(v[1]))
            } else {
                (v[0], v[1])
            };
            return Ok(Some((lo, hi)));
        }
    }
    Ok(None)
}

/// Builds an actuated `Joint` for a `hinge` or `slide` MJCF joint.
fn build_actuated_joint<T>(
    joint_name: &str,
    type_str: &str,
    attrs: &HashMap<String, String>,
    transform: Isometry3<T>,
    parent: &str,
    child: &str,
    settings: &CompilerSettings,
) -> Result<Joint<T>, MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let is_hinge = type_str == "hinge";

    let axis_str = attrs.get("axis").map(|s| s.as_str()).unwrap_or("0 0 1");
    let av = parse_floats_mjcf::<T>(axis_str)?;
    if av.len() != 3 {
        return Err(MjcfParseError::InvalidVector3Len(
            format!("{} axis", joint_name),
            av.len(),
        ));
    }
    let axis = Unit::new_normalize(Vector3::new(av[0], av[1], av[2]));

    let range = parse_mjcf_range::<T>(attrs, settings, is_hinge)?;

    let (joint_type, rot_axis, lin_axis, limit_lower, limit_upper) = if is_hinge {
        match range {
            Some((lo, hi)) => (JointType::Revolute, Some(axis), None, Some(lo), Some(hi)),
            None => {
                let two_pi: T = nalgebra::convert(2.0 * std::f64::consts::PI);
                (JointType::Continuous, Some(axis), None, Some(-two_pi), Some(two_pi))
            }
        }
    } else {
        // slide
        match range {
            Some((lo, hi)) => (JointType::Prismatic, None, Some(axis), Some(lo), Some(hi)),
            None => {
                let big: T = nalgebra::convert(1.0e6_f64);
                (JointType::Prismatic, None, Some(axis), Some(-big), Some(big))
            }
        }
    };

    Ok(Joint {
        name: joint_name.to_string(),
        joint_type,
        parent: parent.to_string(),
        parent_link_idx: 0,
        child: child.to_string(),
        child_link_idx: 0,
        transform,
        rot_axis,
        lin_axis,
        limit_lower,
        limit_upper,
        cmd_idx: None,
    })
}

/// Emits 3 joints + 2 virtual links for `joint.pos ≠ 0`.
///
/// MuJoCo FK formula with a non-zero joint anchor `p`:
/// ```text
/// B(θ) = P · T(body) · T(p) · R(θ) · T(−p)
/// ```
/// We represent this as:
/// ```text
/// P ──[Fixed: T(body)·T(p)]──> pre (virtual)
///   ──[hinge/slide: I, axis]──> post (virtual)
///   ──[Fixed: T(−p)]──────────> B
/// ```
#[allow(clippy::too_many_arguments)]
fn emit_offset_joint<T>(
    joint_name: &str,
    type_str: &str,
    attrs: &HashMap<String, String>,
    body_transform: Isometry3<T>,
    joint_pos: Vector3<T>,
    parent: &str,
    body_link: &str,
    links: &mut Vec<Link>,
    joints: &mut Vec<Joint<T>>,
    settings: &CompilerSettings,
) -> Result<(), MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let pre = format!("{}__{}_pre", body_link, joint_name);
    let post = format!("{}__{}_post", body_link, joint_name);
    links.push(Link { name: pre.clone() });
    links.push(Link { name: post.clone() });

    let t_p = Isometry3::from_parts(Translation3::from(joint_pos), UnitQuaternion::identity());
    joints.push(mjcf_fixed_joint(
        &format!("{}_pre", joint_name),
        parent,
        &pre,
        body_transform * t_p,
    ));

    let actuated =
        build_actuated_joint(joint_name, type_str, attrs, Isometry3::identity(), &pre, &post, settings)?;
    joints.push(actuated);

    let t_neg_p = Isometry3::from_parts(Translation3::from(-joint_pos), UnitQuaternion::identity());
    joints.push(mjcf_fixed_joint(&format!("{}_post", joint_name), &post, body_link, t_neg_p));

    Ok(())
}

/// Emits N joints + (N−1) virtual links for a body that contains multiple joints.
///
/// Chain: `parent → virt_0 → … → virt_(N−2) → body`
/// The first joint carries `body_transform`; subsequent joints use `Identity`.
#[allow(clippy::too_many_arguments)]
fn emit_multi_joint_body<T>(
    joint_nodes: &[roxmltree::Node<'_, '_>],
    body_transform: Isometry3<T>,
    parent: &str,
    body_link: &str,
    links: &mut Vec<Link>,
    joints: &mut Vec<Joint<T>>,
    settings: &CompilerSettings,
    defaults: &MjcfDefaults,
) -> Result<(), MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let n = joint_nodes.len();
    let virt: Vec<String> = (0..n - 1).map(|i| format!("{}__vj{}", body_link, i)).collect();
    for v in &virt {
        links.push(Link { name: v.clone() });
    }

    for (i, jnode) in joint_nodes.iter().enumerate() {
        let attrs = effective_joint_attrs(*jnode, defaults);
        let type_str = attrs.get("type").map(|s| s.as_str()).unwrap_or("hinge").to_string();
        if type_str == "free" {
            continue;
        }
        if type_str == "ball" {
            let jname = attrs.get("name").cloned().unwrap_or_else(|| format!("{}_j{}", body_link, i));
            return Err(MjcfParseError::UnsupportedJointType {
                joint: jname,
                joint_type: "ball".to_string(),
            });
        }
        let joint_name = attrs
            .get("name")
            .cloned()
            .unwrap_or_else(|| format!("{}_j{}", body_link, i));

        let (this_parent, this_child, this_transform) = if i == 0 {
            let child = virt.first().map(|s| s.as_str()).unwrap_or(body_link);
            (parent.to_string(), child.to_string(), body_transform)
        } else {
            let p = virt[i - 1].clone();
            let c = virt.get(i).cloned().unwrap_or_else(|| body_link.to_string());
            (p, c, Isometry3::identity())
        };

        // Note: joint.pos ≠ 0 within multi-joint bodies is not yet fully supported;
        // the offset is approximated as zero (joint.pos ignored).
        let j = build_actuated_joint(
            &joint_name,
            &type_str,
            &attrs,
            this_transform,
            &this_parent,
            &this_child,
            settings,
        )?;
        joints.push(j);
    }
    Ok(())
}

/// Recursively processes an MJCF `<body>` node.
///
/// `parent_link_name` is `None` only for direct children of `<worldbody>` (root bodies).
#[allow(clippy::too_many_arguments)]
fn visit_body<T>(
    node: roxmltree::Node<'_, '_>,
    parent_link_name: Option<&str>,
    links: &mut Vec<Link>,
    joints: &mut Vec<Joint<T>>,
    settings: &CompilerSettings,
    defaults: &MjcfDefaults,
) -> Result<(), MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    for child in node.children() {
        if child.is_element() && child.tag_name().name() == "include" {
            return Err(MjcfParseError::IncludeNotSupported);
        }
    }

    let body_name = node
        .attribute("name")
        .ok_or_else(|| MjcfParseError::MissingAttributeBodyName {
            body_path: parent_link_name
                .map(|p| format!("child of {}", p))
                .unwrap_or_else(|| "worldbody child".to_string()),
        })?
        .to_string();

    let body_transform = parse_mjcf_transform::<T>(node, settings, &body_name)?;
    links.push(Link { name: body_name.clone() });

    // Collect all direct <joint> children, excluding <freejoint>/<free>
    let all_joint_nodes: Vec<_> = node
        .children()
        .filter(|n| n.is_element() && n.tag_name().name() == "joint")
        .collect();

    // Validate types early
    for jn in &all_joint_nodes {
        let t = jn.attribute("type").unwrap_or("hinge");
        if t == "ball" {
            let jname = jn.attribute("name").unwrap_or("unknown").to_string();
            return Err(MjcfParseError::UnsupportedJointType {
                joint: jname,
                joint_type: "ball".to_string(),
            });
        }
        if !matches!(t, "hinge" | "slide" | "free") {
            let jname = jn.attribute("name").unwrap_or("unknown").to_string();
            return Err(MjcfParseError::UnknownJointType {
                joint: jname,
                found: t.to_string(),
            });
        }
    }

    // Keep only hinge/slide (skip free)
    let actuated: Vec<_> = all_joint_nodes
        .iter()
        .filter(|jn| {
            let t = jn.attribute("type").unwrap_or("hinge");
            t != "free"
        })
        .copied()
        .collect();

    match (parent_link_name, actuated.as_slice()) {
        (None, _) => {
            // Root body: no parent joint is needed regardless of joints present.
        }
        (Some(parent), []) => {
            // No actuated joints → rigid attachment to parent.
            joints.push(mjcf_fixed_joint(
                &format!("{}_to_{}", parent, body_name),
                parent,
                &body_name,
                body_transform,
            ));
        }
        (Some(parent), [jn]) => {
            // Single actuated joint → one joint from parent to this body.
            let attrs = effective_joint_attrs(*jn, defaults);
            let type_str =
                attrs.get("type").map(|s| s.as_str()).unwrap_or("hinge").to_string();
            let joint_name = attrs
                .get("name")
                .cloned()
                .unwrap_or_else(|| format!("{}_joint", body_name));

            let joint_pos: Vector3<T> = if let Some(s) = attrs.get("pos") {
                let v = parse_floats_mjcf::<T>(s)?;
                if v.len() != 3 {
                    return Err(MjcfParseError::InvalidVector3Len(
                        format!("{} pos", joint_name),
                        v.len(),
                    ));
                }
                Vector3::new(v[0], v[1], v[2])
            } else {
                Vector3::zeros()
            };

            let eps: T = nalgebra::convert(1e-10_f64);
            if joint_pos.norm() < eps {
                let j = build_actuated_joint(
                    &joint_name,
                    &type_str,
                    &attrs,
                    body_transform,
                    parent,
                    &body_name,
                    settings,
                )?;
                joints.push(j);
            } else {
                emit_offset_joint(
                    &joint_name,
                    &type_str,
                    &attrs,
                    body_transform,
                    joint_pos,
                    parent,
                    &body_name,
                    links,
                    joints,
                    settings,
                )?;
            }
        }
        (Some(parent), jns) => {
            emit_multi_joint_body(
                jns,
                body_transform,
                parent,
                &body_name,
                links,
                joints,
                settings,
                defaults,
            )?;
        }
    }

    // Recurse into child bodies
    for child in node.children() {
        if child.is_element() && child.tag_name().name() == "body" {
            visit_body(child, Some(&body_name), links, joints, settings, defaults)?;
        }
    }
    Ok(())
}

/// Parses an MJCF file at `path` into a [`GalawModel`].
///
/// MJCF (MuJoCo XML) encodes the kinematic tree as nested `<body>` elements inside
/// `<worldbody>`. Each `<joint>` within a body connects that body to its parent.
///
/// Supported joint types: `hinge` (→ `Revolute`/`Continuous`) and `slide` (→ `Prismatic`).
/// `free` joints are silently treated as fixed (useful for floating-base simulation models).
/// `ball` joints return an error.
///
/// # Examples
///
/// ```
/// # fn main() -> Result<(), galaw::error::GalawError<f64>> {
/// let model = galaw::load_mjcf::<f64>("assets/mjcf/custom/simple_arm_2dof.xml")?;
/// assert_eq!(model.name, "simple_arm_2dof");
/// assert_eq!(model.num_actuated_joints, 2);
/// # Ok(())
/// # }
/// ```
pub fn load_mjcf<T>(path: &str) -> Result<GalawModel<T>, GalawError<T>>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError> + std::fmt::Display,
{
    let content = fs::read_to_string(path).map_err(|source| MjcfParseError::Io {
        path: path.to_string(),
        source,
    })?;
    let doc = roxmltree::Document::parse(&content).map_err(|source| MjcfParseError::XmlParse {
        path: path.to_string(),
        source,
    })?;

    let root = doc.root_element(); // <mujoco>
    let model_name = root.attribute("model").unwrap_or("").to_string();

    // Read <compiler> settings (use defaults if absent)
    let mut settings = CompilerSettings::default();
    if let Some(compiler) = root.children().find(|n| n.tag_name().name() == "compiler") {
        if let Some(angle) = compiler.attribute("angle") {
            settings.angle_in_degrees = angle != "radian";
        }
        if let Some(seq) = compiler.attribute("eulerseq") {
            settings.eulerseq = seq.to_string();
        }
    }

    let defaults = parse_mjcf_defaults(root);

    let Some(worldbody) = root.children().find(|n| n.tag_name().name() == "worldbody") else {
        // No worldbody → empty model is valid but unusual; just return empty.
        return Ok(GalawModel {
            name: model_name,
            num_links: 0,
            links: Vec::new(),
            link_name_to_idx: HashMap::new(),
            joints: Vec::new(),
            joint_name_to_idx: HashMap::new(),
            num_actuated_joints: 0,
            ancestors_by_link: Vec::new(),
            chain_by_link: Vec::new(),
            joint_limit_lower: Vec::new(),
            joint_limit_upper: Vec::new(),
        });
    };

    // Check for <include> in worldbody
    for child in worldbody.children() {
        if child.is_element() && child.tag_name().name() == "include" {
            return Err(MjcfParseError::IncludeNotSupported.into());
        }
    }

    let mut links: Vec<Link> = Vec::new();
    let mut joints: Vec<Joint<T>> = Vec::new();

    for child in worldbody.children() {
        if child.is_element() && child.tag_name().name() == "body" {
            visit_body(child, None, &mut links, &mut joints, &settings, &defaults)?;
        }
    }

    let (
        ordered_joints,
        link_name_to_idx,
        joint_name_to_idx,
        ancestors_by_link,
        chain_by_link,
        joint_limit_lower,
        joint_limit_upper,
    ) = resolve_joint_order(&links, &joints)?;

    Ok(GalawModel {
        name: model_name,
        num_links: links.len(),
        links,
        link_name_to_idx,
        joints: ordered_joints,
        joint_name_to_idx,
        num_actuated_joints: joint_limit_lower.len(),
        ancestors_by_link,
        chain_by_link,
        joint_limit_lower,
        joint_limit_upper,
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
    fn joint_resolution_is_independent_of_file_order() {
        let original = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof.urdf").unwrap();
        let flipped = load_urdf::<f64>("assets/urdf/custom/simple_arm_2dof_flipped.urdf").unwrap();

        // Resolve a joint's parent/child *link names* (not raw indices —
        // those are expected to differ between the two files, since the
        // links are declared in a different order in each).
        fn parent_child_names<T: RealField + Copy>(
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

    /// Creates graph ROOT -> A -> B -> A. This ensures that dfs_visit
    /// does not have unbounded recursion error.
    #[test]
    fn resolve_joint_order_detects_reachable_cycle() {
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
            joint("j3", "B", "A"), // closes the cycle back to A
        ];

        let result = resolve_joint_order(&links, &joints);

        assert!(
            matches!(&result, Err(ModelTopologyError::CyclicLink(name)) if name == "A"),
            "expected CyclicLink(\"A\"), got {result:?}"
        );
    }
}
