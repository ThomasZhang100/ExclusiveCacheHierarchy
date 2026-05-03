#!/usr/bin/env python3

import argparse
import csv
import math
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "build" / "experiments" / "csv" / "cache_experiments.csv"
DEFAULT_PLOT_DIR = ROOT / "build" / "experiments" / "plots"


def read_rows(csv_path):
    with csv_path.open() as csv_file:
        return list(csv.DictReader(csv_file))


def load_matplotlib():
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required to render plots. "
            "Run the experiment CSV generation first, then install matplotlib if needed."
        ) from exc
    return plt


def unique_in_order(values):
    seen = set()
    ordered = []
    for value in values:
        if value not in seen:
            seen.add(value)
            ordered.append(value)
    return ordered


def rows_by_key(rows, *keys):
    table = {}
    for row in rows:
        table[tuple(row[key] for key in keys)] = row
    return table


def plot_default_baseline(plt, rows, output_dir):
    default_rows = [row for row in rows if row["experiment"] == "default"]
    bench_names = unique_in_order(row["benchmark"] for row in default_rows)
    lookup = rows_by_key(default_rows, "variant", "benchmark")

    exclusive = [int(lookup[("exclusive", bench)]["num_cycles"]) for bench in bench_names]
    inclusive = [int(lookup[("inclusive", bench)]["num_cycles"]) for bench in bench_names]

    positions = list(range(len(bench_names)))
    width = 0.36

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar([p - width / 2 for p in positions], exclusive, width=width, color="#0f766e", label="Exclusive")
    ax.bar([p + width / 2 for p in positions], inclusive, width=width, color="#f59e0b", label="Inclusive")

    ax.set_title("Baseline Cache Comparison")
    ax.set_ylabel("Cycles")
    ax.set_xticks(positions)
    ax.set_xticklabels(bench_names)
    ax.legend(frameon=False)

    for idx, value in enumerate(exclusive):
        ax.text(idx - width / 2, value, str(value), ha="center", va="bottom", fontsize=8)
    for idx, value in enumerate(inclusive):
        ax.text(idx + width / 2, value, str(value), ha="center", va="bottom", fontsize=8)

    fig.tight_layout()
    fig.savefig(output_dir / "baseline_cycles.png", dpi=220)
    plt.close(fig)


def plot_sweep_by_benchmark(plt, rows, output_dir):
    experiments = unique_in_order(row["experiment"] for row in rows)
    bench_names = unique_in_order(row["benchmark"] for row in rows)
    lookup = rows_by_key(rows, "experiment", "variant", "benchmark")

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    axes = axes.flatten()

    for ax, bench in zip(axes, bench_names):
        excl = [int(lookup[(exp, "exclusive", bench)]["num_cycles"]) for exp in experiments]
        incl = [int(lookup[(exp, "inclusive", bench)]["num_cycles"]) for exp in experiments]
        ax.plot(experiments, excl, marker="o", linewidth=2, color="#0f766e", label="Exclusive")
        ax.plot(experiments, incl, marker="o", linewidth=2, color="#f59e0b", label="Inclusive")
        ax.set_title(bench)
        ax.set_ylabel("Cycles")
        ax.tick_params(axis="x", rotation=25)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False)
    fig.suptitle("Cache Sweep Results by Benchmark", y=0.98, fontsize=16)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(output_dir / "sweep_cycles_by_benchmark.png", dpi=220)
    plt.close(fig)


def plot_speedup_heatmap(plt, rows, output_dir):
    experiments = unique_in_order(row["experiment"] for row in rows)
    bench_names = unique_in_order(row["benchmark"] for row in rows)
    lookup = rows_by_key(rows, "experiment", "variant", "benchmark")

    matrix = []
    for bench in bench_names:
        row_values = []
        for experiment in experiments:
            excl = int(lookup[(experiment, "exclusive", bench)]["num_cycles"])
            incl = int(lookup[(experiment, "inclusive", bench)]["num_cycles"])
            speedup_pct = (incl - excl) / incl * 100.0
            row_values.append(speedup_pct)
        matrix.append(row_values)

    plt.style.use("seaborn-v0_8-white")
    fig, ax = plt.subplots(figsize=(11, 4.8))
    image = ax.imshow(matrix, cmap="YlGn", aspect="auto")
    ax.set_title("Exclusive Speedup Over Inclusive (%)")
    ax.set_xticks(range(len(experiments)))
    ax.set_xticklabels(experiments, rotation=25, ha="right")
    ax.set_yticks(range(len(bench_names)))
    ax.set_yticklabels(bench_names)

    for row_idx, bench in enumerate(bench_names):
        for col_idx, experiment in enumerate(experiments):
            value = matrix[row_idx][col_idx]
            ax.text(col_idx, row_idx, f"{value:.1f}", ha="center", va="center", fontsize=9)

    fig.colorbar(image, ax=ax, shrink=0.9, label="Speedup (%)")
    fig.tight_layout()
    fig.savefig(output_dir / "exclusive_speedup_heatmap.png", dpi=220)
    plt.close(fig)


def write_markdown_summary(rows, output_dir):
    experiments = unique_in_order(row["experiment"] for row in rows)
    bench_names = unique_in_order(row["benchmark"] for row in rows)
    lookup = rows_by_key(rows, "experiment", "variant", "benchmark")

    summary_lines = [
        "# Cache Experiment Summary",
        "",
        "| experiment | benchmark | exclusive_cycles | inclusive_cycles | exclusive_speedup_pct |",
        "|---|---|---:|---:|---:|",
    ]

    for experiment in experiments:
        for bench in bench_names:
            excl = int(lookup[(experiment, "exclusive", bench)]["num_cycles"])
            incl = int(lookup[(experiment, "inclusive", bench)]["num_cycles"])
            speedup_pct = (incl - excl) / incl * 100.0
            summary_lines.append(
                f"| {experiment} | {bench} | {excl} | {incl} | {speedup_pct:.2f} |"
            )

    (output_dir / "experiment_summary.md").write_text("\n".join(summary_lines) + "\n")


def parse_args():
    parser = argparse.ArgumentParser(description="Plot cache experiment results.")
    parser.add_argument("--csv", default=str(DEFAULT_CSV), help="Input CSV from run_cache_experiments.py")
    parser.add_argument("--output-dir", default=str(DEFAULT_PLOT_DIR), help="Directory for plot outputs")
    return parser.parse_args()


def main():
    args = parse_args()
    csv_path = pathlib.Path(args.csv)
    output_dir = pathlib.Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = read_rows(csv_path)
    if not rows:
        raise SystemExit(f"No rows found in {csv_path}")

    plt = load_matplotlib()
    plot_default_baseline(plt, rows, output_dir)
    plot_sweep_by_benchmark(plt, rows, output_dir)
    plot_speedup_heatmap(plt, rows, output_dir)
    write_markdown_summary(rows, output_dir)
    print(f"Wrote plots and summary files to {output_dir}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
