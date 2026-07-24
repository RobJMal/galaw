#[derive(Debug, thiserror::Error)]
pub enum GalawError {
    #[error(transparent)]
    Parse(#[from] UrdfParseError)
}

#[derive(Debug, thiserror::Error)]
pub enum UrdfParseError {
    #[error("failed to read URDF file {path}")]
    Io {
        path: String,
        #[source]
        source: std::io::Error,
    },
    #[error("invalid XML content: {xml_content}")]
    XmlParse {
        xml_content: String,
        #[source]
        source: roxmltree::Error,
    },
    #[error("robot tag missing name attribute")]
    MissingAttributeRobotName,
    #[error("link tag missing name attribute")]
    MissingAttributeLinkName,

    // Errors for <joint/>
    #[error("joint tag missing name attribute")]
    MissingAttributeJointName,
    #[error("joint {0} missing type attribute")]
    MissingAttributeJointType(String),
    #[error("unknown joint type '{found}' for joint {name}")]
    UnknownJointType {
        name: String,
        found: String,
    },

    // <parent/>
    #[error("missing parent tag for joint {0}")]
    MissingTagJointParent(String),
    #[error("missing parent link for joint {0}")]
    MissingAttributeJointParentLink(String),

    // <child/>
    #[error("missing child tag for joint {0}")]
    MissingTagJointChild(String),
    #[error("missing child link for joint {0}")]
    MissingAttributeJointChildLink(String),

    // <origin/>
    #[error("joint {0} missing origin")]
    MissingTagJointOrigin(String),
    #[error("missing xyz data for joint {0}")]
    MissingAttributeJointOriginXyz(String),
    #[error("missing rpy data for joint {0}")]
    MissingAttributeJointOriginRpy(String),

    // <axis/>
    #[error("missing axis xyz data for joint {0}")]
    MissingAttributeJointAxisXyz(String),

    // <limit/>
    #[error("missing joint limit tag for joint {0}")]
    MissingTagJointLimit(String),
    #[error("missing joint limit lower attribute for joint {0}")]
    MissingAttributeJointLimitLower(String),
    #[error("missing joint limit upper attribute for joint {0}")]
    MissingAttributeJointLimitUpper(String),
    #[error("value is invalid number: {value}")]
    InvalidNumberFormat {
        value: String,
        source: std::num::ParseFloatError,
    },
}