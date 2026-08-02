//! Renders Jacobian benchmark charts from Criterion's JSON output using
//! `charming` (Apache ECharts bindings). Reads each benchmark's `estimates.json`
//! so the charts stay in sync with the latest `cargo bench --bench jacobian_speed` run.
//!
//! Each chart is a line-over-DOF plot per implementation, showing:
//!   * the mean per-call time/throughput (with the value printed as a label), and
//!   * a shaded 95% confidence-interval band (ECharts has no native error bars;
//!     the band is drawn as a stacked area between the CI's lower and upper bounds).
//!
//! Usage (after `cargo bench --bench jacobian_speed`):
//!     cargo run --release --example plot_jacobian_bench
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

/// Calls per timed iteration in benches/jacobian_speed.rs. Criterion's estimates
/// are per iteration, so dividing by this converts to per single `compute_jacobian` call.
const N_POSES: f64 = 100.0;

/// No "galaw-generated" entry — no codegen'd Jacobian yet.
const IMPLS: [&str; 2] = ["galaw-runtime", "k"];

/// Wong (2011) colorblind-safe pair, in series order: galaw-runtime=blue, k=orange.
const COLORS: [&str; 2] = ["#0072B2", "#E69F00"];

// ----- FONT SIZES (tweak here — every chart text element is driven off these) -----
const TITLE_FONT_SIZE: f64 = 38.0;
const LEGEND_FONT_SIZE: f64 = 20.0;
/// Shared by both axes' `name` (the "Robot [...]" / "ns per call" labels).
const AXIS_NAME_FONT_SIZE: f64 = 22.0;
/// Shared by both axes' tick labels (the numbers/categories along each axis).
const AXIS_TICK_FONT_SIZE: f64 = 19.0;

/// Approx. rendered height (px) of one data-point label box at LABEL_FONT_SIZE
/// (text line-height + the label's own padding/border) — used to stagger each
/// series' label distance from its point by index, so two series' labels can
/// never collide even if their points land at the same y-pixel. Scales to
/// however many entries IMPLS has; no per-series manual tuning needed.
/// Keep this in sync with LABEL_FONT_SIZE — it's sized for the box height
/// *at that font size*, not computed from it.
const LABEL_FONT_SIZE: f64 = 19.0;
const LABEL_STAGGER_PX: f64 = 16.0;

/// Mean and 95% CI bounds for a single benchmark, in ns per `compute_jacobian` call.
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
        group: format!("jacobian_{}", model.name),
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
            "could not read {} (run `cargo bench --bench jacobian_speed` first): {e}",
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

    Ok(Stat {
        mean: field(mean, "point_estimate")?,
        lo: field(&mean["confidence_interval"], "lower_bound")?,
        hi: field(&mean["confidence_interval"], "upper_bound")?,
    })
}

/// Builds a line chart over DOF with a mean line + labels and a CI band per impl.
fn build_chart(
    robots: &[RobotInfo],
    title: &str,
    y_name: &str,
    to_vals: impl Fn(&Stat) -> (f64, f64, f64),
    round: impl Fn(f64) -> f64,
) -> Result<Chart, Box<dyn Error>> {
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
                .text_style(TextStyle::new().font_size(TITLE_FONT_SIZE)),
        )
        .legend(
            Legend::new()
                .top("bottom")
                .text_style(TextStyle::new().font_size(LEGEND_FONT_SIZE))
                .item_gap(LEGEND_FONT_SIZE * 2.0)
                .width("90%")
                .data(IMPLS.to_vec()),
        )
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
                .name_location(NameLocation::Middle)
                .name_gap(85.0)
                .name_text_style(TextStyle::new().font_size(AXIS_NAME_FONT_SIZE))
                .axis_label(
                    AxisLabel::new()
                        .font_size(AXIS_TICK_FONT_SIZE)
                        .interval(0.0),
                )
                .data(dof_labels),
        )
        .y_axis(
            Axis::new()
                .type_(AxisType::Log)
                .log_base(10.0)
                .name(y_name)
                .name_location(NameLocation::Middle)
                .name_gap(70.0)
                .name_text_style(TextStyle::new().font_size(AXIS_NAME_FONT_SIZE))
                .axis_label(AxisLabel::new().font_size(AXIS_TICK_FONT_SIZE)),
        );

    struct SeriesData {
        impl_: &'static str,
        color: &'static str,
        means: Vec<f64>,
        los: Vec<f64>,
        heights: Vec<f64>,
        typical: f64,
    }
    let mut all_series: Vec<SeriesData> = Vec::new();
    for (&impl_, &color) in IMPLS.iter().zip(COLORS.iter()) {
        let (mut means, mut los, mut heights) = (Vec::new(), Vec::new(), Vec::new());
        for robot in robots {
            let (m, lo, hi) = to_vals(&stat(&robot.group, impl_, robot.bench_id)?);
            means.push(round(m));
            los.push(lo);
            heights.push(hi - lo);
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
        let label_distance = 4.0 + stagger_rank[i] as f64 * LABEL_STAGGER_PX;

        let stack_id = format!("band_{impl_}");
        chart = chart.series(
            Line::new()
                .stack(stack_id.clone())
                .show_symbol(false)
                .line_style(LineStyle::new().opacity(0.0))
                .data(los),
        );
        chart = chart.series(
            Line::new()
                .stack(stack_id)
                .show_symbol(false)
                .line_style(LineStyle::new().opacity(0.0))
                .area_style(AreaStyle::new().color(color).opacity(0.18))
                .data(heights),
        );
        chart = chart.series(
            Line::new()
                .name(impl_)
                .line_style(LineStyle::new().color(color).width(2.0))
                .item_style(ItemStyle::new().color(color))
                .label(
                    Label::new()
                        .show(true)
                        .position(label_pos)
                        .distance(label_distance)
                        .font_size(LABEL_FONT_SIZE)
                        .color(color)
                        .background_color("#ffffff")
                        .border_color(color)
                        .border_width(1.0)
                        .padding((4.0, 8.0, 4.0, 8.0)),
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
    robots.sort_by_key(|r| r.dof);

    let out = manifest_dir().join("docs/bench");
    fs::create_dir_all(&out)?;
    let mut renderer = ImageRenderer::new(1600, 900);

    let latency = build_chart(
        &robots,
        "Jacobian latency scaling (95% CI)",
        "ns per call (lower is better)",
        |s| (s.mean, s.lo, s.hi),
        |x| x.round(),
    )?;
    let p1 = out.join("jacobian_scaling_ns_per_call.png");
    renderer.save_format(
        ImageFormat::Png,
        &latency,
        p1.to_str().ok_or("non-utf8 path")?,
    )?;
    println!("wrote {}", p1.display());

    let mcps = |ns: f64| 1e9 / ns / 1e6;
    let throughput = build_chart(
        &robots,
        "Jacobian throughput (95% CI)",
        "million calls/sec (higher is better)",
        move |s| (mcps(s.mean), mcps(s.hi), mcps(s.lo)),
        |x| (x * 100.0).round() / 100.0,
    )?;
    let p2 = out.join("jacobian_throughput_mcalls.png");
    renderer.save_format(
        ImageFormat::Png,
        &throughput,
        p2.to_str().ok_or("non-utf8 path")?,
    )?;
    println!("wrote {}", p2.display());

    Ok(())
}
