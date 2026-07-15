//! Renders FK benchmark charts from Criterion's JSON output using
//! `charming` (Apache ECharts bindings). Reads each benchmark's `estimates.json`
//! so the charts stay in sync with the latest `cargo bench` run.
//!
//! Each chart is a line-over-DOF plot per implementation, showing:
//!   * the mean per-call time/throughput (with the value printed as a label), and
//!   * a shaded 95% confidence-interval band (ECharts has no native error bars;
//!     the band is drawn as a stacked area between the CI's lower and upper bounds).
//!
//! Usage (after `cargo bench`):
//!     cargo run --release --example plot_bench
//!
//! Output PNGs land in docs/bench/. Requires dev-deps `charming` (feature
//! "ssr-raster") and `serde_json`. The first build is slow: charming's `ssr`
//! feature bundles a JS engine (deno_core) to render ECharts server-side.

use std::error::Error;
use std::fs;
use std::path::PathBuf;

use charming::component::{Axis, Legend, Title};
use charming::element::{
    AreaStyle, AxisType, ItemStyle, Label, LabelPosition, LineStyle, NameLocation,
};
use charming::series::Line;
use charming::{Chart, ImageFormat, ImageRenderer};

/// Calls per timed iteration in benches/fk_speed.rs. Criterion's estimates are
/// per iteration, so dividing by this converts to per single FK call.
const N_POSES: f64 = 100.0;

/// (criterion group dir, display label, DOF). Add a row when you add a robot to
/// the benchmark's URDFS list — keep this in sync with the robot `name=` attrs.
const ROBOTS: &[(&str, &str, u32)] = &[
    ("fk_simple_arm", "simple_arm", 2),
    ("fk_simple_arm_6dof", "simple_arm_6dof", 6),
    ("fk_simple_arm_10dof", "simple_arm_10dof", 10),
];

const IMPLS: [&str; 2] = ["galaw", "k"];

/// Wong (2011) colorblind-safe pair, in series order: galaw=blue, k=orange.
const COLORS: [&str; 2] = ["#0072B2", "#E69F00"];

/// Mean and 95% CI bounds for a single benchmark, in ns per FK call.
struct Stat {
    mean: f64,
    lo: f64,
    hi: f64,
}

fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Reads mean + confidence interval (ns per call) from Criterion's estimates.json.
fn stat(group: &str, impl_: &str, dof: u32) -> Result<Stat, Box<dyn Error>> {
    let path = manifest_dir()
        .join("target/criterion")
        .join(group)
        .join(impl_)
        .join(dof.to_string())
        .join("new/estimates.json");

    let text = fs::read_to_string(&path).map_err(|e| {
        format!("could not read {} (run `cargo bench` first): {e}", path.display())
    })?;
    let v: serde_json::Value = serde_json::from_str(&text)?;
    let mean = &v["mean"];
    let field = |ptr: &serde_json::Value, key: &str| -> Result<f64, Box<dyn Error>> {
        Ok(ptr[key].as_f64().ok_or_else(|| format!("estimates.json: missing {key}"))? / N_POSES)
    };

    // estimates.json: { "mean": { "point_estimate": <ns/iter>,
    //                             "confidence_interval": { "lower_bound", "upper_bound" } } }
    Ok(Stat {
        mean: field(mean, "point_estimate")?,
        lo: field(&mean["confidence_interval"], "lower_bound")?,
        hi: field(&mean["confidence_interval"], "upper_bound")?,
    })
}

/// Builds a line chart over DOF with a mean line + labels and a CI band per impl.
///
/// `to_vals` maps a Stat (ns/call) into (mean, lo, hi) in the chart's own units
/// — throughput inverts, so it also swaps lo/hi. `round` tidies the label text.
fn build_chart(
    title: &str,
    y_name: &str,
    to_vals: impl Fn(&Stat) -> (f64, f64, f64),
    round: impl Fn(f64) -> f64,
) -> Result<Chart, Box<dyn Error>> {
    let dof_labels: Vec<String> = ROBOTS.iter().map(|(_, _, d)| d.to_string()).collect();

    let mut chart = Chart::new()
        .background_color("#ffffff")
        .title(Title::new().text(title).left("center"))
        // Only the mean lines get a legend entry; the band series are unnamed.
        .legend(Legend::new().top("bottom").data(IMPLS.to_vec()))
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Degrees of freedom (joints)")
                .name_location(NameLocation::Middle) // centered under the axis, not clipped at the end
                .name_gap(32.0)
                .data(dof_labels),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Value)
                .name(y_name)
                .name_location(NameLocation::Middle) // centered & auto-rotated along the axis
                .name_gap(50.0) // clears the tick numbers
                .min(0.0), // zero baseline; ECharts auto-picks a clean max
        );

    for (i, (&impl_, &color)) in IMPLS.iter().zip(COLORS.iter()).enumerate() {
        // galaw's label sits above its line, k's below — so the two near-identical
        // series never stack their boxes on top of each other.
        let label_pos = if i == 0 { LabelPosition::Top } else { LabelPosition::Bottom };
        let (mut means, mut los, mut heights) = (Vec::new(), Vec::new(), Vec::new());
        for &(group, _, dof) in ROBOTS {
            let (m, lo, hi) = to_vals(&stat(group, impl_, dof)?);
            means.push(round(m));
            los.push(lo);
            heights.push(hi - lo); // band height, stacked on top of `lo`
        }

        let stack_id = format!("band_{impl_}");
        // Invisible base line lifts the band's baseline to the CI lower bound.
        chart = chart.series(
            Line::new()
                .stack(stack_id.clone())
                .show_symbol(false)
                .line_style(LineStyle::new().opacity(0.0))
                .data(los),
        );
        // Filled band spanning (upper - lower), i.e. the CI, at 18% opacity.
        chart = chart.series(
            Line::new()
                .stack(stack_id)
                .show_symbol(false)
                .line_style(LineStyle::new().opacity(0.0))
                .area_style(AreaStyle::new().color(color).opacity(0.18))
                .data(heights),
        );
        // Mean line on top, with the value printed at each point.
        chart = chart.series(
            Line::new()
                .name(impl_)
                .line_style(LineStyle::new().color(color).width(2.0))
                .item_style(ItemStyle::new().color(color))
                .label(
                    Label::new()
                        .show(true)
                        .position(label_pos) // galaw above its line, k below
                        .distance(6.0) // gap between the point and the box
                        .color(color) // text in the line's color
                        .background_color("#ffffff") // white box fill
                        .border_color(color) // box outline in the line's color
                        .border_width(1.0)
                        .padding((3.0, 6.0, 3.0, 6.0)), // spacing inside the box
                )
                .data(means),
        );
    }
    Ok(chart)
}

fn main() -> Result<(), Box<dyn Error>> {
    let out = manifest_dir().join("docs/bench");
    fs::create_dir_all(&out)?;
    let mut renderer = ImageRenderer::new(900, 560);

    // Latency: ns/call, CI bounds used directly.
    let latency = build_chart(
        "FK latency scaling: galaw vs. k (95% CI)",
        "ns per call (lower is better)",
        |s| (s.mean, s.lo, s.hi),
        |x| x.round(),
    )?;
    let p1 = out.join("scaling_ns_per_call.png");
    renderer.save_format(ImageFormat::Png, &latency, p1.to_str().ok_or("non-utf8 path")?)?;
    println!("wrote {}", p1.display());

    // Throughput: M calls/sec = 1e9 / ns / 1e6. Decreasing in ns, so lo/hi swap.
    let mcps = |ns: f64| 1e9 / ns / 1e6;
    let throughput = build_chart(
        "FK throughput: galaw vs. k (95% CI)",
        "M calls/sec (higher is better)",
        move |s| (mcps(s.mean), mcps(s.hi), mcps(s.lo)),
        |x| (x * 100.0).round() / 100.0,
    )?;
    let p2 = out.join("throughput_mcalls.png");
    renderer.save_format(ImageFormat::Png, &throughput, p2.to_str().ok_or("non-utf8 path")?)?;
    println!("wrote {}", p2.display());

    Ok(())
}
