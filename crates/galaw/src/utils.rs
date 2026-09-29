// Custom
use crate::error::UrdfParseError;

/// Parses and extracts values from a string
///
/// Assumes that it contains 3 numeric values, raises errors otherwise.
pub(crate) fn parse_vec3_str<T>(input_str: &str) -> Result<(T, T, T), UrdfParseError>
where
    T: std::str::FromStr<Err = std::num::ParseFloatError> + Copy,
{
    let vals: Vec<T> = input_str
        .split_whitespace()
        .map(|n| {
            n.parse::<T>()
                .map_err(|source| UrdfParseError::InvalidNumberFormat {
                    value: n.to_string(),
                    source,
                })
        })
        .collect::<Result<Vec<T>, _>>()?;

    if vals.len() != 3 {
        return Err(UrdfParseError::InvalidVector3Len(
            input_str.to_string(),
            vals.len(),
        ));
    }

    Ok((vals[0], vals[1], vals[2]))
}
