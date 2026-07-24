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
    MissingXmlAttributeRobotName,
    #[error("link tag missing name attribute")]
    MissingXmlAttributeLinkName,

    // Errors for <joint/>
    #[error("joint tag missing name attribute")]
    MissingXmlAttributeJointName,
    #[error("joint {0} missing type attribute")]
    MissingXmlAttributeJointType(String),

    // <parent/>
    #[error("missing parent tag for joint {0}")]
    MissingXmlTagJointParent(String),
    #[error("missing parent link for joint {0}")]
    MissingXmlAttributeJointParentLink(String),

    // <child/>
    #[error("missing child tag for joint {0}")]
    MissingXmlTagChildLink(String),
    #[error("missing child link for joint {0}")]
    MissingXmlAttributeJointChildLink(String),

    // <origin/>
    #[error("joint {0} missing origin")]
    MissingXmlTagJointOrigin(String),
    #[error("missing xyz data for joint {0}")]
    MissingXmlAttributeJointOriginXyz(String),
    #[error("missing rpy data for joint {0}")]
    MissingXmlAttributeJointOriginRpy(String),

    // <axis/>
    #[error("missing axis xyz data for joint {0}")]
    MissingXmlAttributeJointAxisXyz(String),

    // <limit/>
    #[error("missing joint limit tag for joint {0}")]
    MissingXmlTagJointLimit(String),
    #[error("missing joint limit lower attribute for joint {0}")]
    MissingXmlAttributeJointLimitLower(String),
    #[error("missing joint limit upper attribute for joint {0}")]
    MissingXmlAttributeJointLimitUpper(String),
}