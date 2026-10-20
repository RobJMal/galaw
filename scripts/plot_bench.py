#!/usr/bin/env python3
"""
Plot FK, IK, and Jacobian benchmark charts from Criterion's estimates.json.

Each plot type puts all kinematic computations (FK / Jacobian / IK) side by
side in ONE figure, so a reader sees the whole story at a glance.

Usage:
  python3 scripts/plot_bench.py [fk|ik|jacobian] [--plots LIST]

  --plots     comma-separated subset of:
                speedup_vs_pinocchio_cpp  speedup vs. native C++ Pinocchio (FK/Jacobian)
                speedup_vs_k_rust        speedup vs. k (Rust) (FK/Jacobian/IK)
                grouped   absolute latency, grouped bars, log y
                multiples one panel per robot, horizontal bars with exact values
                table     markdown tables for the README (benchmark_results.md)
              default: speedup_vs_pinocchio-cpp,speedup_vs_k,grouped,multiples,table

Output: docs/bench/*.png and docs/bench/benchmark_results.md
Deps:   matplotlib, numpy
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs/bench"

# Criterion impl directory names (must match BenchmarkId::new(...) in bench code).
IMPLS  = ["galaw-runtime", "galaw-generated", "k", "pinocchio", "pinocchio-cpp"]
LABELS = ["galaw-runtime", "galaw-generated", "k", "rs-pinocchio", "pinocchio-cpp"]
# Wong (2011) colorblind-safe palette.
COLORS = ["#0072B2", "#009E73", "#E69F00", "#56B4E9", "#CC79A7"]
LABEL = dict(zip(IMPLS, LABELS))
COLOR = dict(zip(IMPLS, COLORS))

# Implementations shown as bars in the speedup figure (the baseline is the 1x line).
SPEEDUP_IMPLS = ["galaw-runtime", "galaw-generated"]

# (name, bench_id=total_joints, dof=actuated_joints), sorted by dof ascending.
ROBOTS = [
    ("Enlight-L",       9,  7),
    ("stretch4",       53, 13),
    ("anymal-D",       95, 14),
    ("wujihand-right", 25, 20),
]

ALL_KINDS = ["fk", "jacobian", "ik"]
KIND_TITLE = {"fk": "FK", "ik": "IK", "jacobian": "Jacobian"}
ALL_PLOTS = ["speedup_vs_pinocchio_cpp", "speedup_vs_k_rust", "grouped", "multiples", "table"]
DEFAULT_PLOTS = ["speedup_vs_pinocchio_cpp", "speedup_vs_k_rust", "grouped", "multiples", "table"]

NOTE = ("Mean time per call, single-threaded, 95% CI. "
        "rs-pinocchio is a Rust wrapper over Pinocchio; pinocchio-cpp is native C++.")

plt.rcParams.update({
    "axes.spines.top":   False,
    "axes.spines.right": False,
    "axes.grid":         True,
    "grid.color":        "#cccccc",
    "grid.linestyle":    "--",
    "grid.linewidth":    0.6,
    "grid.alpha":        0.6,
})


# ---------------------------------------------------------------- loading ---

def load_stat(kind, robot, impl_, bench_id):
    """Return (mean_ns, lo_ns, hi_ns) or None."""
    path = (
        REPO_ROOT / "target/criterion"
        / f"{kind}_{robot}" / impl_ / str(bench_id)
        / "new/estimates.json"
    )
    try:
        v = json.loads(path.read_text())["mean"]
        ci = v["confidence_interval"]
        return v["point_estimate"], ci["lower_bound"], ci["upper_bound"]
    except (FileNotFoundError, KeyError):
        return None


def collect(kinds):
    """data[kind][impl] -> array of shape (n_robots, 3): mean, lo, hi (ns).

    An implementation is included only if it has data for every robot."""
    data = {}
    for kind in kinds:
        per_impl = {}
        for impl_ in IMPLS:
            rows = [load_stat(kind, r, impl_, b) for r, b, _ in ROBOTS]
            if all(rows):
                per_impl[impl_] = np.array(rows, dtype=float)
        if per_impl:
            data[kind] = per_impl
    return data


def pick_baseline(per_impl, preferred):
    for cand in (preferred, "k"):
        if cand in per_impl:
            return cand
    return None


# ---------------------------------------------------------------- helpers ---

def fmt_tick(x, _):
    if x >= 1e6:
        return f"{x/1e6:.0f}M"
    if x >= 1e3:
        return f"{x/1e3:.0f}k"
    if x >= 1:
        return f"{x:.0f}"
    return f"{x:g}"


def style_log_y(ax):
    ax.set_yscale("log")
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(fmt_tick))
    ax.yaxis.set_minor_formatter(ticker.NullFormatter())


def robot_tick_labels():
    return [f"{n}\n[{b}/{d}]" for n, b, d in ROBOTS]


def add_legend(fig, axes, ncol, y=0.97):
    seen = {}
    for ax in axes:
        for h, l in zip(*ax.get_legend_handles_labels()):
            seen.setdefault(l, h)
    fig.legend(seen.values(), seen.keys(), loc="upper center", ncol=ncol,
               frameon=False, fontsize=11, bbox_to_anchor=(0.5, y))


def finish(fig, title, stem, suffix, note=NOTE):
    fig.suptitle(title, fontsize=16, fontweight="bold", y=1.03)
    fig.text(0.5, -0.02, note, ha="center", fontsize=9, color="dimgray")
    out = OUT_DIR / f"{stem}{suffix}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  wrote {out}")


# ------------------------------------------------------------------ plots ---

def plot_speedup_vs(data, baseline_impl, stem, suffix):
    """Headline figure: speedup relative to a fixed baseline, one panel per kind
    that has data for that baseline.

    Ratio = baseline_mean / impl_mean. Error bar: baseline_lo/impl_hi .. baseline_hi/impl_lo."""
    kinds = [k for k in data if baseline_impl in data[k]]
    if not kinds:
        return
    fig, axes = plt.subplots(1, len(kinds), figsize=(5.6 * len(kinds) + 0.6, 5.4),
                             squeeze=False)
    for ax, kind in zip(axes[0], kinds):
        d = data[kind]
        series = [i for i in SPEEDUP_IMPLS if i in d and i != baseline_impl]
        w = 0.8 / max(len(series), 1)
        x = np.arange(len(ROBOTS))
        top = 1.0
        for j, impl in enumerate(series):
            m = d[baseline_impl][:, 0] / d[impl][:, 0]
            lo = d[baseline_impl][:, 1] / d[impl][:, 2]
            hi = d[baseline_impl][:, 2] / d[impl][:, 1]
            pos = x + (j - (len(series) - 1) / 2) * w
            ax.bar(pos, m, w, color=COLOR[impl], label=LABEL[impl],
                   yerr=[m - lo, hi - m], capsize=2,
                   error_kw=dict(lw=1, alpha=0.7))
            for p, v, h in zip(pos, m, hi):
                ax.text(p, h * 1.02, f"{v:.1f}x", ha="center", va="bottom", fontsize=9)
            top = max(top, hi.max())
        ax.axhline(1, color="black", lw=1.2, ls="--", zorder=0)
        ax.set_ylim(0, top * 1.15)
        ax.set_xticks(x)
        ax.set_xticklabels(robot_tick_labels(), fontsize=9)
        ax.set_title(KIND_TITLE[kind], fontsize=13, fontweight="bold")
        ax.set_xlabel("Robot [total joints / actuated joints]", fontsize=9)
    axes[0][0].set_ylabel("Speedup (higher is better; dashed line = baseline)", fontsize=10)
    add_legend(fig, axes[0], ncol=2)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    finish(fig, f"galaw speedup vs {LABEL[baseline_impl]}", stem, suffix,
           note=NOTE + f" Baseline: {LABEL[baseline_impl]} (dashed line = 1×).")


def plot_grouped(data, suffix):
    kinds = list(data)
    fig, axes = plt.subplots(1, len(kinds), figsize=(6.2 * len(kinds) + 0.6, 5.6),
                             squeeze=False)
    for ax, kind in zip(axes[0], kinds):
        d = data[kind]
        impls = [i for i in IMPLS if i in d]
        w = 0.8 / len(impls)
        x = np.arange(len(ROBOTS))
        for j, impl in enumerate(impls):
            m, lo, hi = d[impl].T
            ax.bar(x + (j - (len(impls) - 1) / 2) * w, m, w, color=COLOR[impl],
                   label=LABEL[impl], yerr=[m - lo, hi - m],
                   error_kw=dict(lw=0.8, alpha=0.6))
        style_log_y(ax)
        ax.set_xticks(x)
        ax.set_xticklabels(robot_tick_labels(), fontsize=9)
        ax.set_xlabel("Robot [total joints / actuated joints]", fontsize=9)
        ax.set_title(KIND_TITLE[kind], fontsize=13, fontweight="bold")
    axes[0][0].set_ylabel("ns per call (log scale, lower is better)", fontsize=10)
    add_legend(fig, axes[0], ncol=len(IMPLS))
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    finish(fig, "Latency by robot", "latency_comparison_all_robots", suffix)


def plot_multiples(data, suffix):
    kinds = list(data)
    nr, nc = len(kinds), len(ROBOTS)
    fig, axes = plt.subplots(nr, nc, figsize=(4.1 * nc, 2.7 * nr + 0.8),
                             sharex="row", squeeze=False)
    for r, kind in enumerate(kinds):
        d = data[kind]
        impls = [i for i in IMPLS if i in d]
        ypos = np.arange(len(impls))[::-1]  # first impl at the top
        row_min = min(d[i][:, 0].min() for i in impls)
        row_max = max(d[i][:, 0].max() for i in impls)
        for c, (robot, total, dof) in enumerate(ROBOTS):
            ax = axes[r][c]
            vals = [d[i][c, 0] for i in impls]
            ax.barh(ypos, vals, color=[COLOR[i] for i in impls], height=0.7)
            ax.set_xscale("log")
            ax.set_xlim(row_min * 0.5, row_max * 9)
            for y, v in zip(ypos, vals):
                ax.text(v * 1.15, y, f"{v:,.0f}", va="center", fontsize=8.5)
            ax.set_yticks(ypos)
            ax.set_yticklabels([LABEL[i] for i in impls] if c == 0 else [], fontsize=9)
            ax.grid(axis="y", visible=False)
            if r == 0:
                ax.set_title(f"{robot} [{total}/{dof}]", fontsize=11, fontweight="bold")
            if c == 0:
                ax.set_ylabel(KIND_TITLE[kind], fontsize=12, fontweight="bold")
            if r == nr - 1:
                ax.set_xlabel("ns per call (log)", fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    finish(fig, "Latency per robot (lower is better)", "latency_per_robot_with_values", suffix)



def write_table(data, preferred, suffix):
    out = []
    for kind, d in data.items():
        impls = [i for i in IMPLS if i in d]
        out.append(f"### {KIND_TITLE[kind]}: mean ns per call\n")
        out.append("| Robot [total/actuated] | " + " | ".join(LABEL[i] for i in impls) + " |")
        out.append("|---|" + "---:|" * len(impls))
        for r, (name, total, dof) in enumerate(ROBOTS):
            cells = " | ".join(f"{d[i][r, 0]:,.0f}" for i in impls)
            out.append(f"| {name} [{total}/{dof}] | {cells} |")
        base = pick_baseline(d, preferred)
        series = [i for i in SPEEDUP_IMPLS if i in d and i != base]
        if base and series:
            out.append(f"\n{KIND_TITLE[kind]} speedup vs. {LABEL[base]}:\n")
            out.append("| Robot | " + " | ".join(LABEL[i] for i in series) + " |")
            out.append("|---|" + "---:|" * len(series))
            for r, (name, _, _) in enumerate(ROBOTS):
                cells = " | ".join(f"{d[base][r, 0] / d[i][r, 0]:.1f}x" for i in series)
                out.append(f"| {name} | {cells} |")
        out.append("")
    out.append(f"_{NOTE}_\n")
    path = OUT_DIR / f"benchmark_results{suffix}.md"
    path.write_text("\n".join(out))
    print(f"  wrote {path}")


# ------------------------------------------------------------------- main ---

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", nargs="?", choices=ALL_KINDS,
                    help="only this computation (default: all, side by side)")
    ap.add_argument("--plots", default=",".join(DEFAULT_PLOTS),
                    help="comma-separated: " + ",".join(ALL_PLOTS))
    args = ap.parse_args()

    plots = [p.strip() for p in args.plots.split(",") if p.strip()]
    bad = [p for p in plots if p not in ALL_PLOTS]
    if bad:
        ap.error(f"unknown plot(s) {bad}; valid: {ALL_PLOTS}")

    kinds = [args.kind] if args.kind else ALL_KINDS
    suffix = f"_{args.kind}" if args.kind else ""
    data = collect(kinds)
    if not data:
        raise SystemExit("no Criterion data found under target/criterion")
    missing = [k for k in kinds if k not in data]
    if missing:
        print(f"note: no data for {missing}, skipping")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    print("rendering...")
    if "speedup_vs_pinocchio_cpp" in plots:
        plot_speedup_vs(data, "pinocchio-cpp", "speedup_vs_pinocchio_cpp", suffix)
    if "speedup_vs_k_rust" in plots:
        plot_speedup_vs(data, "k", "speedup_vs_k_rust", suffix)
    if "grouped" in plots:
        plot_grouped(data, suffix)
    if "multiples" in plots:
        plot_multiples(data, suffix)
    if "table" in plots:
        write_table(data, "pinocchio-cpp", suffix)


if __name__ == "__main__":
    main()