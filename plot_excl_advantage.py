#!/usr/bin/env python3
"""
Compare exclusive vs. inclusive under two configs side by side:
  default  : L1=1KB  L2=8KB  L3=64KB
  tiny-l2  : L1=1KB  L2=1KB  L3=64KB  (L2 = L1, exposing inclusive duplication penalty)
"""

import argparse
import csv
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "build" / "experiments" / "csv" / "cache_experiments.csv"
DEFAULT_OUT = ROOT / "build" / "experiments" / "plots" / "excl_advantage.png"

BENCHMARKS  = ["vvadd", "cmplx-mult", "masked-filter", "bin-search", "conflict-miss"]
EXPERIMENTS = ["default", "tiny-l2"]
SUBTITLES   = {
    "default": "default  (L1=1 KB, L2=8 KB, L3=64 KB)",
    "tiny-l2": "tiny-l2  (L1=1 KB, L2=1 KB, L3=64 KB)",
}


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def extract(rows, experiment, variant, benchmark):
    for row in rows:
        if (row["experiment"] == experiment
                and row["variant"]    == variant
                and row["benchmark"]  == benchmark):
            return int(row["num_cycles"])
    return None


def parse_args():
    p = argparse.ArgumentParser(description="Plot exclusive advantage under tiny-l2 config.")
    p.add_argument("--csv",    default=str(DEFAULT_CSV))
    p.add_argument("--output", default=str(DEFAULT_OUT))
    return p.parse_args()


def main():
    args   = parse_args()
    csv_path = pathlib.Path(args.csv)
    out_path = pathlib.Path(args.output)

    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")

    try:
        import matplotlib.pyplot as plt
        import matplotlib.ticker as mticker
    except ImportError:
        sys.exit("matplotlib required: pip install matplotlib")

    rows = read_csv(csv_path)
    present_exps = {r["experiment"] for r in rows}
    for exp in EXPERIMENTS:
        if exp not in present_exps:
            sys.exit(f"Experiment '{exp}' not found in CSV — run run_cache_experiments.py first.")

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    x     = list(range(len(BENCHMARKS)))
    width = 0.36

    for ax, exp in zip(axes, EXPERIMENTS):
        excl_cycles = [extract(rows, exp, "exclusive", b) for b in BENCHMARKS]
        incl_cycles = [extract(rows, exp, "inclusive", b) for b in BENCHMARKS]

        missing = [BENCHMARKS[i] for i, v in enumerate(excl_cycles + incl_cycles) if v is None]
        if missing:
            sys.exit(f"Missing data for experiment '{exp}': {missing}")

        bars_e = ax.bar([p - width / 2 for p in x], excl_cycles, width=width,
                        color="#0f766e", label="Exclusive")
        bars_i = ax.bar([p + width / 2 for p in x], incl_cycles, width=width,
                        color="#f59e0b", label="Inclusive")

        for bar in bars_e:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() * 1.01,
                    f"{int(bar.get_height()):,}", ha="center", va="bottom", fontsize=7.5)
        for bar in bars_i:
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() * 1.01,
                    f"{int(bar.get_height()):,}", ha="center", va="bottom", fontsize=7.5)

        ax.set_title(SUBTITLES[exp], fontsize=11)
        ax.set_ylabel("Cycles")
        ax.set_xticks(x)
        ax.set_xticklabels(BENCHMARKS, rotation=15, ha="right")
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))
        ax.legend(frameon=False)

        # annotate speedup/slowdown ratios above the pair
        for i, (e, inc) in enumerate(zip(excl_cycles, incl_cycles)):
            ratio = (inc - e) / inc * 100.0
            sign  = "+" if ratio >= 0 else ""
            color = "#0f766e" if ratio >= 0 else "#dc2626"
            ax.annotate(f"{sign}{ratio:.1f}%",
                        xy=(i, max(e, inc) * 1.12),
                        ha="center", va="bottom", fontsize=8,
                        color=color, fontweight="bold")

    fig.suptitle(
        "Exclusive vs. Inclusive — effect of shrinking L2 to L1 capacity\n"
        "(% = exclusive speedup over inclusive; positive = exclusive wins)",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.93])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
