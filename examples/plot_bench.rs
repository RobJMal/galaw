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

// Third-party
use charming::component::{Axis, Grid, Legend, Title};
use charming::element::{
    AreaStyle, AxisLabel, AxisType, ItemStyle, Label, LabelPosition, LineStyle, NameLocation,
    TextStyle,
};
use charming::series::Line;
use charming::{Chart, ImageFormat, ImageRenderer};

// Custom
use galaw::{fixtures::BENCH_URDFS, load_urdf};

/// Calls per timed iteration in benches/fk_speed.rs. Criterion's estimates are
/// per iteration, so dividing by this converts to per single FK call.
const N_POSES: f64 = 100.0;

const IMPLS: [&str; 3] = ["galaw", "galaw-generated", "k"];

/// Wong (2011) colorblind-safe triple, in series order: galaw=blue,
/// galaw-generated=bluish green, k=orange.
const COLORS: [&str; 3] = ["#0072B2", "#009E73", "#E69F00"];

const LEGEND_FONT_SIZE: f64 = 18.0;

/// Approx. rendered height (px) of one data-point label box at LABEL_FONT_SIZE
/// (text line-height + the label's own padding/border) — used to stagger each
/// series' label distance from its point by index, so two series' labels can
/// never collide even if their points land at the same y-pixel. Scales to
/// however many entries IMPLS has; no per-series manual tuning needed.
const LABEL_FONT_SIZE: f64 = 17.0;
const LABEL_STAGGER_PX: f64 = 30.0;

/// Mean and 95% CI bounds for a single benchmark, in ns per FK call.
struct Stat {
    mean: f64,
    lo: f64,
    hi: f64,
}

struct RobotInfo {
    name: String, // matches galaw_model.name, for the x-axis label
    group: String,
    bench_id: u32, // matches galaw_model.joints.len()
    dof: u32,      // matches galaw_model.num_actuated_joints
}

// ----- HELPER METHODS -----
fn manifest_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn robot_info(urdf_path: &str) -> Result<RobotInfo, Box<dyn Error>> {
    let model = load_urdf(urdf_path)?;
    Ok(RobotInfo {
        name: model.name.clone(),
        group: format!("fk_{}", model.name),
        bench_id: model.joints.len() as u32,
        dof: model.num_actuated_joints as u32,
    })
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
        format!(
            "could not read {} (run `cargo bench` first): {e}",
            path.display()
        )
    })?;
    let v: serde_json::Value = serde_json::from_str(&text)?;
    let mean = &v["mean"];
    let field = |ptr: &serde_json::Value, key: &str| -> Result<f64, Box<dyn Error>> {
        Ok(ptr[key]
            .as_f64()
            .ok_or_else(|| format!("estimates.json: missing {key}"))?
            / N_POSES)
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
    robots: &[RobotInfo],
    title: &str,
    y_name: &str,
    to_vals: impl Fn(&Stat) -> (f64, f64, f64),
    round: impl Fn(f64) -> f64,
) -> Result<Chart, Box<dyn Error>> {
    // "robot_name\n[total_joints/actuated_joints]" — e.g. "Enlight-L\n[9/7]".
    let dof_labels: Vec<String> = robots
        .iter()
        .map(|r| format!("{}\n[{}/{}]", r.name, r.bench_id, r.dof))
        .collect();

    let mut chart = Chart::new()
        .background_color("#ffffff")
        .title(
            Title::new()
                .text(title)
                .left("center")
                .text_style(TextStyle::new().font_size(34.0)),
        )
        // Only the mean lines get a legend entry; the band series are unnamed.
        .legend(
            Legend::new()
                .top("bottom")
                .text_style(TextStyle::new().font_size(LEGEND_FONT_SIZE))
                // item_gap is derived from the font size (not a flat magic
                // number) so it stays proportionate if the font size ever
                // changes. width caps the legend's own box so ECharts wraps
                // it onto additional rows automatically once entries no
                // longer fit on one line - covers however many entries
                // IMPLS ends up with, not just today's 3.
                .item_gap(LEGEND_FONT_SIZE * 2.0)
                .width("90%")
                .data(IMPLS.to_vec()),
        )
        // Tight left/right/top margins so the plot fills the canvas instead
        // of leaving unused whitespace at the much larger 1600x900 size;
        // bottom stays generous for the two-line tick labels + axis name + legend.
        .grid(
            Grid::new()
                .left("4%")
                .right("4%")
                .top("12%")
                .bottom(170)
                .contain_label(true),
        )
        .x_axis(
            Axis::new()
                .type_(AxisType::Category)
                .name("Robot [total joints / actuated joints]")
                .name_location(NameLocation::Middle) // centered under the axis, not clipped at the end
                .name_gap(85.0) // clears the two-line tick labels above it
                .name_text_style(TextStyle::new().font_size(20.0))
                // interval(0.0) forces every category to render — ECharts'
                // default "auto" interval was silently hiding most of these
                // (only 4 of 8 robots were showing up) because it judged
                // them too crowded to fit; font_size makes them readable.
                .axis_label(AxisLabel::new().font_size(17.0).interval(0.0))
                .data(dof_labels),
        )
        .y_axis(
            Axis::new()
                // Log, not linear: values span ~100x (e.g. 67ns to 7946ns), so
                // on a linear axis the close-together galaw/galaw-generated
                // points for small/fast robots were only a few pixels apart -
                // not enough room for their value labels to avoid colliding.
                // Log scale gives every *ratio* equal visual space regardless
                // of magnitude, which is what was actually missing (a log
                // axis can't include zero, so there's no .min(0.0) here -
                // ECharts auto-fits the range to the data instead).
                .type_(AxisType::Log)
                .log_base(10.0)
                .name(y_name)
                .name_location(NameLocation::Middle) // centered & auto-rotated along the axis
                .name_gap(70.0) // clears the tick numbers
                .name_text_style(TextStyle::new().font_size(20.0))
                .axis_label(AxisLabel::new().font_size(17.0)),
        );

    // Gather every series' full data first, in one pass, instead of building
    // and rendering each series immediately - the label stagger below needs
    // to compare series against each other, so all of them have to be known
    // before any one series' label distance can be decided.
    struct SeriesData {
        impl_: &'static str,
        color: &'static str,
        means: Vec<f64>,
        los: Vec<f64>,
        heights: Vec<f64>,
        typical: f64, // median mean, used only to rank stagger order below
    }
    let mut all_series: Vec<SeriesData> = Vec::new();
    for (&impl_, &color) in IMPLS.iter().zip(COLORS.iter()) {
        let (mut means, mut los, mut heights) = (Vec::new(), Vec::new(), Vec::new());
        for robot in robots {
            let (m, lo, hi) = to_vals(&stat(&robot.group, impl_, robot.bench_id)?);
            means.push(round(m));
            los.push(lo);
            heights.push(hi - lo); // band height, stacked on top of `lo`
        }
        let mut sorted_means = means.clone();
        sorted_means.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let typical = sorted_means[sorted_means.len() / 2];
        all_series.push(SeriesData {
            impl_,
            color,
            means,
            los,
            heights,
            typical,
        });
    }

    // Rank series by typical value, ascending, and stagger each one's label
    // distance to match that rank — NOT by IMPLS order. Every label sits
    // above its own point (never below, so none can ever collide with the
    // x-axis tick labels), and a fixed per-series offset that ignores actual
    // value order can backfire badly: earlier, "galaw-generated" (typically
    // the smallest value) got a *larger* offset than "galaw" purely by
    // IMPLS position, which pushed its label *up*, into "galaw"'s point and
    // label instead of away from it, whenever the two were close. Ranking by
    // the data itself instead of array position means the smallest-typical
    // series always gets the smallest offset (staying close to its own,
    // lower point) and each larger one gets pushed further up in turn — away
    // from its neighbors below, not into them — regardless of how many
    // series there are or what order IMPLS happens to list them in.
    let mut rank_order: Vec<usize> = (0..all_series.len()).collect();
    rank_order.sort_by(|&a, &b| {
        all_series[a]
            .typical
            .partial_cmp(&all_series[b].typical)
            .unwrap()
    });
    let mut stagger_rank = vec![0usize; all_series.len()];
    for (rank, &series_idx) in rank_order.iter().enumerate() {
        stagger_rank[series_idx] = rank;
    }

    for (i, series) in all_series.into_iter().enumerate() {
        let SeriesData {
            impl_,
            color,
            means,
            los,
            heights,
            ..
        } = series;
        let label_pos = LabelPosition::Top;
        let label_distance = 8.0 + stagger_rank[i] as f64 * LABEL_STAGGER_PX;

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
                        .position(label_pos) // every series stacks above its line, staggered by value rank
                        .distance(label_distance) // gap between the point and the box
                        .font_size(LABEL_FONT_SIZE)
                        .color(color) // text in the line's color
                        .background_color("#ffffff") // white box fill
                        .border_color(color) // box outline in the line's color
                        .border_width(1.0)
                        .padding((4.0, 8.0, 4.0, 8.0)), // spacing inside the box
                )
                .data(means),
        );
    }
    Ok(chart)
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut robots: Vec<RobotInfo> = BENCH_URDFS
        .iter()
        .map(|&p| robot_info(p))
        .collect::<Result<_, _>>()?;
    // BENCH_URDFS isn't declared in DOF order, so the x-axis needs an
    // explicit sort — otherwise the category axis just follows fixture
    // declaration order, which isn't monotonic.
    robots.sort_by_key(|r| r.dof);

    let out = manifest_dir().join("docs/bench");
    fs::create_dir_all(&out)?;
    // Wider (room for 8 two-line category labels at readable size, without
    // ECharts skipping any) and taller (room between close-together points
    // and bigger text overall) than the original 900x560.
    let mut renderer = ImageRenderer::new(1600, 900);

    // Latency: ns/call, CI bounds used directly.
    let latency = build_chart(
        &robots,
        "FK latency scaling (95% CI)",
        "ns per call (lower is better)",
        |s| (s.mean, s.lo, s.hi),
        |x| x.round(),
    )?;
    let p1 = out.join("scaling_ns_per_call.png");
    renderer.save_format(
        ImageFormat::Png,
        &latency,
        p1.to_str().ok_or("non-utf8 path")?,
    )?;
    println!("wrote {}", p1.display());

    // Throughput: million calls/sec = 1e9 / ns / 1e6. Decreasing in ns, so lo/hi swap.
    let mcps = |ns: f64| 1e9 / ns / 1e6;
    let throughput = build_chart(
        &robots,
        "FK throughput (95% CI)",
        "million calls/sec (higher is better)",
        move |s| (mcps(s.mean), mcps(s.hi), mcps(s.lo)),
        |x| (x * 100.0).round() / 100.0,
    )?;
    let p2 = out.join("throughput_mcalls.png");
    renderer.save_format(
        ImageFormat::Png,
        &throughput,
        p2.to_str().ok_or("non-utf8 path")?,
    )?;
    println!("wrote {}", p2.display());

    Ok(())
}
