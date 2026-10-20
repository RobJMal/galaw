#pragma once

// fwd.hpp must come first so Pinocchio can install its Eigen plugins.
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace bench {

// ---- Constants (mirror the Rust bench) ----
static constexpr int     N_POSES          = 100;
static constexpr int     WARMUP_ITERS     = 1000;
static constexpr int     N_SAMPLES        = 100;
static constexpr double  TARGET_SAMPLE_NS = 50'000'000.0; // ~50 ms per sample

// ---- Robot descriptors ----
struct Robot {
    std::string name;
    std::string urdf_path;
    int         bench_id; // galaw_model.joints.len() — must match the Rust bench's BenchmarkId
};

// The four benchmark robots. bench_ids are hardcoded from Rust bench output:
//   galaw_model.joints.len() for each robot.
inline std::vector<Robot> bench_robots(const std::string& repo_root) {
    return {
        {"Enlight-L",
         repo_root + "/crates/galaw/assets/urdf/third_party/Flexiv_Enlight-L/Enlight-L.urdf",
         9},
        {"anymal-D",
         repo_root + "/crates/galaw/assets/urdf/third_party/ANYbotics_ANYmal-D/ANYmal-D.urdf",
         95},
        {"stretch4",
         repo_root + "/crates/galaw/assets/urdf/third_party/Hello-Robot_Stretch4/Stretch4.urdf",
         53},
        {"wujihand-right",
         repo_root + "/crates/galaw/assets/urdf/third_party/Wuji-Technology_Wuji-Hand/Wuji-Hand-v1_right.urdf",
         25},
    };
}

// ---- Config generation ----
// Generates N_POSES random configurations for a Pinocchio model.
// Uses std::mt19937 seeded with 42 (matches the Rust bench's RNG_SEED).
// Exact values differ (ChaCha8 vs MT19937) but the distribution is identical:
// uniform over [lower, upper] for bounded joints, 0.0 for unbounded ones.
inline std::vector<Eigen::VectorXd>
generate_configs(const pinocchio::Model& model) {
    std::mt19937 rng(42);

    std::vector<Eigen::VectorXd> configs;
    configs.reserve(N_POSES);

    for (int p = 0; p < N_POSES; ++p) {
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
        for (int k = 0; k < model.nq; ++k) {
            double lb = model.lowerPositionLimit[k];
            double ub = model.upperPositionLimit[k];
            // Pinocchio uses ±1e10 as the infinity sentinel for unbounded joints.
            bool limited = std::isfinite(lb) && std::isfinite(ub)
                           && std::abs(lb) < 1e9 && std::abs(ub) < 1e9;
            if (limited) {
                std::uniform_real_distribution<double> dist(lb, ub);
                q[k] = dist(rng);
            }
            // else leave q[k] = 0.0
        }
        configs.push_back(std::move(q));
    }
    return configs;
}

// ---- Frame selection ----
// Returns indices of BODY frames — URDF links, matching galaw_model.links.
// Excludes JOINT, FIXED_JOINT, SENSOR, and OP_FRAME types.
inline std::vector<pinocchio::FrameIndex>
body_frame_ids(const pinocchio::Model& model) {
    std::vector<pinocchio::FrameIndex> ids;
    for (pinocchio::FrameIndex i = 0; i < static_cast<pinocchio::FrameIndex>(model.nframes); ++i) {
        if (model.frames[i].type == pinocchio::BODY) {
            ids.push_back(i);
        }
    }
    return ids;
}

// ---- Dead-code elimination prevention ----
// Equivalent to Rust's std::hint::black_box / criterion's black_box.
// Forces the compiler to treat &val as live memory, preventing the optimizer
// from eliding the computation that produced it. GCC/Clang only.
template <class T>
inline void do_not_optimize(const T& val) {
    asm volatile("" : : "m"(val) : "memory");
}

// ---- Timing harness ----
using Clock  = std::chrono::steady_clock;
using Nanos  = double;

struct BenchResult {
    Nanos mean;
    Nanos lo;      // 95% CI lower bound
    Nanos hi;      // 95% CI upper bound
    Nanos std_dev;
    Nanos median;
    Nanos mad;     // median absolute deviation
};

// Runs fn() repeatedly and returns per-call statistics.
// Protocol:
//   1. Warm up (WARMUP_ITERS calls, discarded).
//   2. Calibrate: time one call to choose iters_per_sample so each sample
//      chunk takes ~TARGET_SAMPLE_NS.
//   3. Collect N_SAMPLES chunk means. Each chunk mean = total_ns / iters.
//   4. Report mean ± 1.96σ/√N (95% CI) and median/MAD.
template <typename Fn>
BenchResult run_bench(Fn&& fn) {
    // --- Warm up ---
    for (int i = 0; i < WARMUP_ITERS; ++i) fn();

    // --- Calibrate ---
    auto t0 = Clock::now();
    fn();
    auto t1 = Clock::now();
    Nanos one_ns = static_cast<Nanos>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count());
    int iters = static_cast<int>(
        std::clamp(TARGET_SAMPLE_NS / std::max(one_ns, 1.0), 1.0, 1e5));

    // --- Collect samples ---
    std::vector<Nanos> sample_means(N_SAMPLES);
    for (int s = 0; s < N_SAMPLES; ++s) {
        auto s0 = Clock::now();
        for (int k = 0; k < iters; ++k) fn();
        auto s1 = Clock::now();
        Nanos total = static_cast<Nanos>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(s1 - s0).count());
        sample_means[s] = total / iters;
    }

    // --- Statistics ---
    const double n = N_SAMPLES;
    double mean = std::accumulate(sample_means.begin(), sample_means.end(), 0.0) / n;
    double variance = 0.0;
    for (auto v : sample_means) variance += (v - mean) * (v - mean);
    variance /= (n - 1.0);
    double std_dev = std::sqrt(variance);
    double se      = std_dev / std::sqrt(n);

    auto sorted = sample_means;
    std::sort(sorted.begin(), sorted.end());
    double median = sorted[N_SAMPLES / 2];

    std::vector<double> abs_devs(N_SAMPLES);
    for (int i = 0; i < N_SAMPLES; ++i) abs_devs[i] = std::abs(sample_means[i] - median);
    std::sort(abs_devs.begin(), abs_devs.end());
    double mad = abs_devs[N_SAMPLES / 2];

    return BenchResult{mean, mean - 1.96 * se, mean + 1.96 * se, std_dev, median, mad};
}

// ---- Criterion-compatible JSON output ----
// Writes estimates.json matching Criterion's schema so plot_bench.rs picks it up.
// plot_bench.rs reads: mean.point_estimate and mean.confidence_interval.{lower,upper}_bound.
inline void write_estimates_json(const std::string& path, const BenchResult& r) {
    namespace fs = std::filesystem;
    fs::create_directories(fs::path(path).parent_path());

    auto stat_block = [](std::ostringstream& js,
                         const std::string& name,
                         double pe, double lo, double hi, double se) {
        js << "\"" << name << "\":{"
           << "\"confidence_interval\":{"
           << "\"confidence_level\":0.95,"
           << "\"lower_bound\":"  << lo  << ","
           << "\"upper_bound\":"  << hi  << "},"
           << "\"point_estimate\":" << pe << ","
           << "\"standard_error\":" << se << "}";
    };

    std::ostringstream js;
    js << std::fixed;
    js.precision(10);
    js << "{";
    double se = (r.hi - r.lo) / (2.0 * 1.96);
    stat_block(js, "mean",           r.mean,    r.lo,             r.hi,             se);
    js << ",";
    stat_block(js, "median",         r.median,  r.median - r.mad, r.median + r.mad, r.mad * 0.5);
    js << ",";
    stat_block(js, "median_abs_dev", r.mad,     r.mad * 0.9,      r.mad * 1.1,      r.mad * 0.05);
    js << ",\"slope\":null,";
    stat_block(js, "std_dev",        r.std_dev, r.std_dev * 0.9,  r.std_dev * 1.1,  r.std_dev * 0.05);
    js << "}";

    std::ofstream f(path);
    if (!f) throw std::runtime_error("cannot open for writing: " + path);
    f << js.str();
}

// Writes the result for one robot to
//   {repo_root}/target/criterion/{prefix}_{robot.name}/{impl_name}/{bench_id}/new/estimates.json
inline void write_result(const std::string& repo_root,
                         const std::string& prefix,
                         const Robot&       robot,
                         const std::string& impl_name,
                         const BenchResult& r) {
    std::string path = repo_root + "/target/criterion/"
        + prefix + "_" + robot.name + "/"
        + impl_name + "/"
        + std::to_string(robot.bench_id) + "/new/estimates.json";
    write_estimates_json(path, r);
    std::cout << "  wrote " << path << "\n";
}

} // namespace bench
