pub fn parse_vec3_str(input_str: &str) -> Result<(f64, f64, f64), Box<dyn std::error::Error>> {
    // Parses and extracts values from string. Assumes will contain 3 values.
    let vals: Vec<f64> = input_str
        .split_whitespace()
        .map(|n| n.parse::<f64>())
        .collect::<Result<Vec<f64>, _>>()?;

    if vals.len() != 3 {
        return Err("expected exactly 3 values".into());
    }

    Ok((vals[0], vals[1], vals[2]))
}