use std::collections::HashMap;
use std::fs;
use std::str::FromStr;

// Third-party
use nalgebra::{
    Isometry3, Matrix3, Quaternion, RealField, Rotation3, Translation3, Unit, UnitQuaternion,
    Vector3,
};

// Custom
use crate::error::{GalawError, MjcfParseError};
use crate::types::{GalawModel, Joint, JointType, Link};

// =============================================================================
// COMPILER SETTINGS
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

// =============================================================================
// DEFAULT CLASS INHERITANCE
// =============================================================================

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
///
/// Priority (lowest → highest):
/// 1. `inherited_class` defaults (from the nearest ancestor body's `childclass`)
/// 2. The joint's own `class` attribute defaults
/// 3. The joint's explicit attributes
fn effective_joint_attrs(
    jnode: roxmltree::Node<'_, '_>,
    defaults: &MjcfDefaults,
    inherited_class: Option<&str>,
) -> HashMap<String, String> {
    let mut attrs: HashMap<String, String> = HashMap::new();
    if let Some(cls) = inherited_class {
        if let Some(defs) = defaults.get(cls) {
            attrs.extend(defs.iter().map(|(k, v)| (k.clone(), v.clone())));
        }
    }
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

// =============================================================================
// TRANSFORM PARSING
// =============================================================================

/// Parses a whitespace-separated string into a `Vec<T>`.
fn parse_floats_mjcf<T>(s: &str) -> Result<Vec<T>, MjcfParseError>
where
    T: Copy + FromStr<Err = std::num::ParseFloatError>,
{
    s.split_whitespace()
        .map(|tok| {
            tok.parse::<T>()
                .map_err(|source| MjcfParseError::InvalidNumberFormat {
                    value: tok.to_string(),
                    source,
                })
        })
        .collect()
}

/// Builds an `Isometry3` from the pos/rotation attributes on a body or joint node.
///
/// Supported rotation formats: `quat` (wxyz), `euler`, `axisangle`, `xyaxes`, `zaxis`.
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
            return Err(MjcfParseError::InvalidVector3Len(
                format!("{} pos", name),
                v.len(),
            ));
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
        return Err(MjcfParseError::MultipleRotationFormats {
            body: name.to_string(),
        });
    }

    let rotation: UnitQuaternion<T> = if let Some(s) = node.attribute("quat") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 4 {
            return Err(MjcfParseError::InvalidVector4Len(
                format!("{} quat", name),
                v.len(),
            ));
        }
        UnitQuaternion::new_normalize(Quaternion::new(v[0], v[1], v[2], v[3]))
    } else if let Some(s) = node.attribute("euler") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 3 {
            return Err(MjcfParseError::InvalidVector3Len(
                format!("{} euler", name),
                v.len(),
            ));
        }
        let (a, b, c) = (
            settings.to_radians(v[0]),
            settings.to_radians(v[1]),
            settings.to_radians(v[2]),
        );
        apply_mjcf_euler(a, b, c, &settings.eulerseq)
    } else if let Some(s) = node.attribute("axisangle") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 4 {
            return Err(MjcfParseError::InvalidVector4Len(
                format!("{} axisangle", name),
                v.len(),
            ));
        }
        let axis = Unit::new_normalize(Vector3::new(v[0], v[1], v[2]));
        UnitQuaternion::from_axis_angle(&axis, settings.to_radians(v[3]))
    } else if let Some(s) = node.attribute("xyaxes") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 6 {
            return Err(MjcfParseError::InvalidVector4Len(
                format!("{} xyaxes (expected 6 values)", name),
                v.len(),
            ));
        }
        let mat = Matrix3::from_columns(&[
            Vector3::new(v[0], v[1], v[2]),
            Vector3::new(v[3], v[4], v[5]),
            Vector3::new(v[0], v[1], v[2]).cross(&Vector3::new(v[3], v[4], v[5])),
        ]);
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(mat))
    } else if let Some(s) = node.attribute("zaxis") {
        let v = parse_floats_mjcf::<T>(s)?;
        if v.len() != 3 {
            return Err(MjcfParseError::InvalidVector3Len(
                format!("{} zaxis", name),
                v.len(),
            ));
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
        q *= r;
    }
    q
}

// =============================================================================
// JOINT CONSTRUCTION
// =============================================================================

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
                (
                    JointType::Continuous,
                    Some(axis),
                    None,
                    Some(-two_pi),
                    Some(two_pi),
                )
            }
        }
    } else {
        // slide
        match range {
            Some((lo, hi)) => (JointType::Prismatic, None, Some(axis), Some(lo), Some(hi)),
            None => {
                let big: T = nalgebra::convert(1.0e6_f64);
                (
                    JointType::Prismatic,
                    None,
                    Some(axis),
                    Some(-big),
                    Some(big),
                )
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

// =============================================================================
// BODY TREE TRAVERSAL
// =============================================================================

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

    let actuated = build_actuated_joint(
        joint_name,
        type_str,
        attrs,
        Isometry3::identity(),
        &pre,
        &post,
        settings,
    )?;
    joints.push(actuated);

    let t_neg_p = Isometry3::from_parts(Translation3::from(-joint_pos), UnitQuaternion::identity());
    joints.push(mjcf_fixed_joint(
        &format!("{}_post", joint_name),
        &post,
        body_link,
        t_neg_p,
    ));

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
    inherited_class: Option<&str>,
) -> Result<(), MjcfParseError>
where
    T: RealField + Copy + FromStr<Err = std::num::ParseFloatError>,
{
    let n = joint_nodes.len();
    let virt: Vec<String> = (0..n - 1)
        .map(|i| format!("{}__vj{}", body_link, i))
        .collect();
    for v in &virt {
        links.push(Link { name: v.clone() });
    }

    for (i, jnode) in joint_nodes.iter().enumerate() {
        let attrs = effective_joint_attrs(*jnode, defaults, inherited_class);
        let type_str = attrs
            .get("type")
            .map(|s| s.as_str())
            .unwrap_or("hinge")
            .to_string();
        if type_str == "free" {
            continue;
        }
        if type_str == "ball" {
            let jname = attrs
                .get("name")
                .cloned()
                .unwrap_or_else(|| format!("{}_j{}", body_link, i));
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
            let c = virt
                .get(i)
                .cloned()
                .unwrap_or_else(|| body_link.to_string());
            (p, c, Isometry3::identity())
        };

        // Note: joint.pos ≠ 0 within multi-joint bodies is not yet fully supported;
        // the offset is approximated as zero (joint.pos ignored).
        joints.push(build_actuated_joint(
            &joint_name,
            &type_str,
            &attrs,
            this_transform,
            &this_parent,
            &this_child,
            settings,
        )?);
    }
    Ok(())
}

/// Recursively processes an MJCF `<body>` node.
///
/// `parent_link_name` is `None` only for direct children of `<worldbody>` (root bodies).
/// `inherited_class` is the nearest ancestor body's `childclass` attribute, if any.
#[allow(clippy::too_many_arguments)]
fn visit_body<T>(
    node: roxmltree::Node<'_, '_>,
    parent_link_name: Option<&str>,
    links: &mut Vec<Link>,
    joints: &mut Vec<Joint<T>>,
    settings: &CompilerSettings,
    defaults: &MjcfDefaults,
    inherited_class: Option<&str>,
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

    // `childclass` on this body overrides the inherited class for all descendants.
    let child_class: Option<&str> = node.attribute("childclass").or(inherited_class);

    let body_transform = parse_mjcf_transform::<T>(node, settings, &body_name)?;
    links.push(Link {
        name: body_name.clone(),
    });

    // Collect direct <joint> children, excluding <freejoint>/<free>
    let all_joint_nodes: Vec<_> = node
        .children()
        .filter(|n| n.is_element() && n.tag_name().name() == "joint")
        .collect();

    // Validate types early
    for jn in &all_joint_nodes {
        let t = jn.attribute("type").unwrap_or("hinge");
        if t == "ball" {
            return Err(MjcfParseError::UnsupportedJointType {
                joint: jn.attribute("name").unwrap_or("unknown").to_string(),
                joint_type: "ball".to_string(),
            });
        }
        if !matches!(t, "hinge" | "slide" | "free") {
            return Err(MjcfParseError::UnknownJointType {
                joint: jn.attribute("name").unwrap_or("unknown").to_string(),
                found: t.to_string(),
            });
        }
    }

    // Keep only hinge/slide (skip free)
    let actuated: Vec<_> = all_joint_nodes
        .iter()
        .filter(|jn| jn.attribute("type").unwrap_or("hinge") != "free")
        .copied()
        .collect();

    match (parent_link_name, actuated.as_slice()) {
        (None, []) => {
            // Root body rigidly attached to the world. Apply body_transform via a
            // virtual world-root link only when the transform is non-identity
            // (e.g. Spot: pos="0 0 0.75", Shadow Hand: quat="0 1 0 1").
            let eps: T = nalgebra::convert(1e-10_f64);
            let has_translation = body_transform.translation.vector.norm() > eps;
            let has_rotation = (T::one() - body_transform.rotation.quaternion().w.abs()) > eps;
            if has_translation || has_rotation {
                let world_root = format!("{body_name}__world_root");
                let insert_at = links.len() - 1;
                links.insert(
                    insert_at,
                    Link {
                        name: world_root.clone(),
                    },
                );
                joints.push(mjcf_fixed_joint(
                    &format!("world_to_{body_name}"),
                    &world_root,
                    &body_name,
                    body_transform,
                ));
            }
        }
        (None, _) => {
            // Root body with actuated joints (free joints already filtered).
            // Non-identity body_transform + actuated joints on the root body
            // are not supported in the current model set.
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
            let attrs = effective_joint_attrs(*jn, defaults, child_class);
            let type_str = attrs
                .get("type")
                .map(|s| s.as_str())
                .unwrap_or("hinge")
                .to_string();
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
                joints.push(build_actuated_joint(
                    &joint_name,
                    &type_str,
                    &attrs,
                    body_transform,
                    parent,
                    &body_name,
                    settings,
                )?);
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
                child_class,
            )?;
        }
    }

    // Recurse into child bodies, propagating the current class.
    for child in node.children() {
        if child.is_element() && child.tag_name().name() == "body" {
            visit_body(
                child,
                Some(&body_name),
                links,
                joints,
                settings,
                defaults,
                child_class,
            )?;
        }
    }
    Ok(())
}

// =============================================================================
// PUBLIC ENTRY POINT
// =============================================================================

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

    for child in worldbody.children() {
        if child.is_element() && child.tag_name().name() == "include" {
            return Err(MjcfParseError::IncludeNotSupported.into());
        }
    }

    let mut links: Vec<Link> = Vec::new();
    let mut joints: Vec<Joint<T>> = Vec::new();

    for child in worldbody.children() {
        if child.is_element() && child.tag_name().name() == "body" {
            visit_body(
                child,
                None,
                &mut links,
                &mut joints,
                &settings,
                &defaults,
                None,
            )?;
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
