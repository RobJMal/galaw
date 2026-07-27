#[derive(Debug, thiserror::Error)]
pub enum GalawError {
    #[error(transparent)]
    Parse(#[from] UrdfParseError),
    #[error(transparent)]
    ModelTopology(#[from] ModelTopologyError),
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
}

#[derive(Debug, thiserror::Error)]
pub enum UrdfParseError {
    #[error("failed to read URDF file '{path}'")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to parse XML content: '{xml_content}'")]
    XmlParse {
        xml_content: String,
        #[source]
        source: roxmltree::Error,
    },
    #[error("expected 3 values, received {1} in '{0}'")]
    InvalidVector3Len(String, usize),
    #[error("missing 'name' attribute on <robot> tag")]
    MissingAttributeRobotName,
    #[error("missing 'name' attribute on <link> tag")]
    MissingAttributeLinkName,

    // Errors for <joint/>
    #[error("missing 'name' attribute on <joint> tag")]
    MissingAttributeJointName,
    #[error("missing 'type' attribute for joint '{0}'")]
    MissingAttributeJointType(String),
    #[error("unknown joint type '{found}' for joint '{name}'")]
    UnknownJointType { name: String, found: String },

    // <parent/>
    #[error("missing '<parent>' tag for joint '{0}'")]
    MissingTagJointParent(String),
    #[error("missing 'link' attribute on <parent> tag for joint '{0}'")]
    MissingAttributeJointParentLink(String),

    // <child/>
    #[error("missing '<child>' tag for joint '{0}'")]
    MissingTagJointChild(String),
    #[error("missing 'link' attribute on <child> tag for joint '{0}'")]
    MissingAttributeJointChildLink(String),

    // <origin/>
    #[error("missing '<origin>' tag for joint '{0}'")]
    MissingTagJointOrigin(String),
    #[error("missing 'xyz' attribute on <origin> tag for joint '{0}'")]
    MissingAttributeJointOriginXyz(String),
    #[error("missing 'rpy' attribute on <origin> tag for joint '{0}'")]
    MissingAttributeJointOriginRpy(String),

    // <axis/>
    #[error("missing 'xyz' attribute on <axis> tag for joint '{0}'")]
    MissingAttributeJointAxisXyz(String),

    // <limit/>
    #[error("missing '<limit>' tag for joint '{0}'")]
    MissingTagJointLimit(String),
    #[error("missing 'lower' attribute on <limit> tag for joint '{0}'")]
    MissingAttributeJointLimitLower(String),
    #[error("missing 'upper' attribute on <limit> tag for joint '{0}'")]
    MissingAttributeJointLimitUpper(String),
    #[error("invalid number '{value}'")]
    InvalidNumberFormat {
        value: String,
        #[source]
        source: std::num::ParseFloatError,
    },
}

#[derive(Debug, thiserror::Error)]
pub enum ModelTopologyError {
    #[error("no root link found, every link has a parent (URDF may contain a cycle)")]
    MissingRootLink,
    #[error("multiple root-like links found, URDF may be disconnected: {0:?}")]
    MultipleRootLinks(Vec<String>),
    #[error("joint unreachable from root, URDF may be disconnected: {0:?}")]
    DisconnectedJoints(Vec<String>),
    #[error("link '{0}' has a cyclic connection")]
    CyclicLink(String),
}

#[derive(Debug, thiserror::Error)]
pub enum KinematicsError {
    #[error("expected {num_actuated} joint cmds, received {num_input}")]
    JointCmdLengthMismatch {
        num_actuated: usize,
        num_input: usize,
    },
}
