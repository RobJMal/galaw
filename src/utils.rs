// Custom
use crate::error::UrdfParseError;

/// Parses and extracts values from a string
///
/// Assumes that it contains 3 numeric values, raises errors otheriwse.
pub fn parse_vec3_str(input_str: &str) -> Result<(f64, f64, f64), UrdfParseError> {
    let vals: Vec<f64> = input_str
        .split_whitespace()
        .map(|n| {
            n.parse::<f64>()
                .map_err(|source| UrdfParseError::InvalidNumberFormat {
                    value: n.to_string(),
                    source,
                })
        })
        .collect::<Result<Vec<f64>, _>>()?;

    if vals.len() != 3 {
        return Err(UrdfParseError::InvalidVector3Len(
            input_str.to_string(),
            vals.len(),
        ));
    }

    Ok((vals[0], vals[1], vals[2]))
}
