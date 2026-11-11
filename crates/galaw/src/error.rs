//! Error types returned by this crate's public API.
//!
//! [`crate::error::GalawError`] is the crate-wide error type every public function
//! returns; it wraps the more specific [`crate::error::UrdfParseError`],
//! [`crate::error::ModelTopologyError`], and [`crate::error::KinematicsError`] enums.

/// The crate-wide error type returned by every public `galaw` function.
#[derive(Debug, thiserror::Error)]
pub enum GalawError<T: std::fmt::Debug + std::fmt::Display + Copy> {
    /// A URDF file couldn't be read or parsed.
    #[error(transparent)]
    Parse(#[from] UrdfParseError),
    /// An MJCF file couldn't be read or parsed.
    #[error(transparent)]
    ParseMjcf(#[from] MjcfParseError),
    /// A parsed robot file describes an invalid robot structure.
    #[error(transparent)]
    ModelTopology(#[from] ModelTopologyError),
    /// A kinematics computation failed.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError<T>),
}

/// Errors from reading and parsing a URDF file.
#[derive(Debug, thiserror::Error)]
pub enum UrdfParseError {
    /// The URDF file couldn't be read from disk.
    #[error("failed to read URDF file '{path}'")]
    Io {
        /// Path of the file that failed to read.
        path: String,
        /// The underlying I/O error.
        #[source]
        source: std::io::Error,
    },
    /// The URDF file's contents aren't valid XML.
    #[error("failed to parse XML in URDF file '{path}': {source}")]
    XmlParse {
        /// Path of the file that failed to parse.
        path: String,
        /// The underlying XML parse error.
        #[source]
        source: roxmltree::Error,
    },
    /// An `xyz`/`rpy`/`axis` attribute didn't contain exactly 3 values.
    #[error("expected 3 values, received {1} in '{0}'")]
    InvalidVector3Len(String, usize),
    /// The `<robot>` tag is missing its `name` attribute.
    #[error("missing 'name' attribute on <robot> tag")]
    MissingAttributeRobotName,
    /// A `<link>` tag is missing its `name` attribute.
    #[error("missing 'name' attribute on <link> tag")]
    MissingAttributeLinkName,

    // Errors for <joint/>
    /// A `<joint>` tag is missing its `name` attribute.
    #[error("missing 'name' attribute on <joint> tag")]
    MissingAttributeJointName,
    /// A joint is missing its `type` attribute.
    #[error("missing 'type' attribute for joint '{0}'")]
    MissingAttributeJointType(String),
    /// A joint's `type` isn't `revolute`, `prismatic`, `fixed`, or `continuous`.
    #[error("unknown joint type '{found}' for joint '{name}'")]
    UnknownJointType {
        /// The joint's name.
        name: String,
        /// The invalid type value that was found.
        found: String,
    },

    // <parent/>
    /// A joint is missing its `<parent>` tag.
    #[error("missing '<parent>' tag for joint '{0}'")]
    MissingTagJointParent(String),
    /// A joint's `<parent>` tag is missing its `link` attribute.
    #[error("missing 'link' attribute on <parent> tag for joint '{0}'")]
    MissingAttributeJointParentLink(String),

    // <child/>
    /// A joint is missing its `<child>` tag.
    #[error("missing '<child>' tag for joint '{0}'")]
    MissingTagJointChild(String),
    /// A joint's `<child>` tag is missing its `link` attribute.
    #[error("missing 'link' attribute on <child> tag for joint '{0}'")]
    MissingAttributeJointChildLink(String),

    // <origin/>
    /// A joint is missing its `<origin>` tag.
    #[error("missing '<origin>' tag for joint '{0}'")]
    MissingTagJointOrigin(String),
    /// A joint's `<origin>` tag is missing its `xyz` attribute.
    #[error("missing 'xyz' attribute on <origin> tag for joint '{0}'")]
    MissingAttributeJointOriginXyz(String),
    /// A joint's `<origin>` tag is missing its `rpy` attribute.
    #[error("missing 'rpy' attribute on <origin> tag for joint '{0}'")]
    MissingAttributeJointOriginRpy(String),

    // <axis/>
    /// A joint's `<axis>` tag is missing its `xyz` attribute.
    #[error("missing 'xyz' attribute on <axis> tag for joint '{0}'")]
    MissingAttributeJointAxisXyz(String),

    // <limit/>
    /// A revolute or prismatic joint is missing its `<limit>` tag.
    #[error("missing '<limit>' tag for joint '{0}'")]
    MissingTagJointLimit(String),
    /// A joint's `<limit>` tag is missing its `lower` attribute.
    #[error("missing 'lower' attribute on <limit> tag for joint '{0}'")]
    MissingAttributeJointLimitLower(String),
    /// A joint's `<limit>` tag is missing its `upper` attribute.
    #[error("missing 'upper' attribute on <limit> tag for joint '{0}'")]
    MissingAttributeJointLimitUpper(String),
    /// An attribute expected to hold a number couldn't be parsed as one.
    #[error("invalid number '{value}'")]
    InvalidNumberFormat {
        /// The invalid value that was found.
        value: String,
        /// The underlying parse error.
        #[source]
        source: std::num::ParseFloatError,
    },
}

/// Errors from reading and parsing an MJCF file.
#[derive(Debug, thiserror::Error)]
pub enum MjcfParseError {
    /// The MJCF file couldn't be read from disk.
    #[error("failed to read MJCF file '{path}'")]
    Io {
        /// Path of the file that failed to read.
        path: String,
        /// The underlying I/O error.
        #[source]
        source: std::io::Error,
    },
    /// The MJCF file's contents aren't valid XML.
    #[error("failed to parse XML in MJCF file '{path}': {source}")]
    XmlParse {
        /// Path of the file that failed to parse.
        path: String,
        /// The underlying XML parse error.
        #[source]
        source: roxmltree::Error,
    },
    /// The `<mujoco>` root element is missing its `model` attribute.
    #[error("missing 'model' attribute on <mujoco> tag")]
    MissingAttributeModelName,
    /// A `<body>` element is missing its `name` attribute.
    #[error("missing 'name' attribute on <body> at path '{body_path}'")]
    MissingAttributeBodyName {
        /// XPath-like description of where the unnamed body appears.
        body_path: String,
    },
    /// An attribute expected to hold a Vec3 didn't contain exactly 3 values.
    #[error("expected 3 values, received {1} in '{0}'")]
    InvalidVector3Len(String, usize),
    /// An attribute expected to hold a Vec4 didn't contain exactly 4 values.
    #[error("expected 4 values, received {1} in '{0}'")]
    InvalidVector4Len(String, usize),
    /// A body specifies more than one rotation format simultaneously.
    #[error(
        "body '{body}' specifies more than one rotation attribute (quat/euler/axisangle/xyaxes/zaxis)"
    )]
    MultipleRotationFormats {
        /// Name of the body with conflicting rotation attributes.
        body: String,
    },
    /// A joint's `type` isn't a recognised MJCF type.
    #[error("unknown joint type '{found}' for joint '{joint}'")]
    UnknownJointType {
        /// The joint's name.
        joint: String,
        /// The unrecognised type value that was found.
        found: String,
    },
    /// A joint type is recognised but not yet supported by galaw (e.g. `ball`).
    #[error("joint '{joint}' has unsupported type '{joint_type}'")]
    UnsupportedJointType {
        /// The joint's name.
        joint: String,
        /// The unsupported type.
        joint_type: String,
    },
    /// An attribute expected to hold a number couldn't be parsed as one.
    #[error("invalid number '{value}'")]
    InvalidNumberFormat {
        /// The invalid value that was found.
        value: String,
        /// The underlying parse error.
        #[source]
        source: std::num::ParseFloatError,
    },
    /// An `<include>` element was encountered; this is not yet implemented.
    #[error("<include> is not yet supported by the galaw MJCF parser")]
    IncludeNotSupported,
}

/// Errors from validating a parsed URDF's link/joint structure.
#[derive(Debug, thiserror::Error)]
pub enum ModelTopologyError {
    /// No link is without a parent, so no root link could be found.
    #[error("no root link found, every link has a parent (URDF may contain a cycle)")]
    MissingRootLink,
    /// More than one link has no parent — the URDF is a disconnected forest, not one tree.
    #[error("multiple root-like links found, URDF may be disconnected: {0:?}")]
    MultipleRootLinks(Vec<String>),
    /// Some joints aren't reachable from the root link.
    #[error("joint unreachable from root, URDF may be disconnected: {0:?}")]
    DisconnectedJoints(Vec<String>),
    /// A link was revisited while walking the tree from root, indicating a cycle.
    #[error("link '{0}' has a cyclic connection")]
    CyclicLink(String),
}

/// Errors from computing forward kinematics.
#[derive(Debug, thiserror::Error)]
pub enum KinematicsError<T: std::fmt::Debug + std::fmt::Display + Copy> {
    /// The `joint_cmds` slice's length doesn't match the model's actuated joint count.
    #[error("expected {num_actuated} joint cmds, received {num_input}")]
    JointCmdLengthMismatch {
        /// Expected number of joint commands.
        num_actuated: usize,
        /// Number of joint commands actually given.
        num_input: usize,
    },
    /// `target_link_idx` is out of range for this model's links.
    #[error("model has {num_links} links, requested index {requested}")]
    LinkIdxOutOfBounds {
        /// Number of links in the model.
        num_links: usize,
        /// The out-of-range index that was requested.
        requested: usize,
    },
    /// IK failed to converge
    #[error("ik did not converge within {iterations} iterations. final error: {final_error}")]
    IkDidNotConverge {
        /// Number of iterations
        iterations: usize,
        /// Final error of iterations
        final_error: T,
    },
}
