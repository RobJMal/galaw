use std::fs;
use std::str::FromStr;

// Third-party
use nalgebra::{Isometry3, RealField, Translation3, Unit, UnitQuaternion, Vector3};

// Custom
use crate::error::{GalawError, UrdfParseError};
use crate::types::{GalawModel, Joint, JointType, Link};
use crate::utils::parse_vec3_str;

/// Parses the axis information from tag
fn read_axis<T>(
    node: roxmltree::Node<'_, '_>,
    joint_name: &str,
) -> Result<Unit<Vector3<T>>, UrdfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
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

    let joint_parent: roxmltree::Node<'_, '_> = node
        .children()
        .find(|n| n.tag_name().name() == "parent")
        .ok_or_else(|| UrdfParseError::MissingTagJointParent(name.clone()))?;
    let parent: String = joint_parent
        .attribute("link")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointParentLink(name.clone()))?
        .to_string();

    let joint_child: roxmltree::Node<'_, '_> = node
        .children()
        .find(|n| n.tag_name().name() == "child")
        .ok_or_else(|| UrdfParseError::MissingTagJointChild(name.clone()))?;
    let child: String = joint_child
        .attribute("link")
        .ok_or_else(|| UrdfParseError::MissingAttributeJointChildLink(name.clone()))?
        .to_string();

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

    let (rot_axis, lin_axis) = match joint_type {
        JointType::Prismatic => (None, Some(read_axis::<T>(node, &name)?)),
        JointType::Revolute | JointType::Continuous => (Some(read_axis::<T>(node, &name)?), None),
        JointType::Fixed => (None, None),
    };

    let (limit_lower, limit_upper) = match joint_type {
        JointType::Revolute | JointType::Prismatic => {
            let (lower, upper) = read_joint_limits::<T>(node, &name)?;
            (Some(lower), Some(upper))
        }
        JointType::Continuous => {
            let two_pi: T = nalgebra::convert(2.0 * std::f64::consts::PI);
            (Some(-two_pi), Some(two_pi))
        }
        JointType::Fixed => (None, None),
    };

    Ok(Joint {
        name,
        joint_type,
        parent,
        parent_link_idx: 0,
        child,
        child_link_idx: 0,
        transform,
        lin_axis,
        rot_axis,
        limit_lower,
        limit_upper,
        cmd_idx: None,
    })
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
    ) = super::resolve_joint_order(&links, &joints)?;

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
