#!/usr/bin/env python3

import argparse
import csv
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "build" / "experiments" / "csv" / "cache_experiments.csv"
DEFAULT_OUT = ROOT / "build" / "experiments" / "plots" / "default_comparison.png"

BENCHMARKS = ["bin-search", "cmplx-mult", "masked-filter", "vvadd"]


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def extract(rows, variant, benchmark):
    for row in rows:
        if (row["experiment"] == "default"
                and row["variant"]   == variant
                and row["benchmark"] == benchmark):
            return int(row["num_cycles"])
    return None


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--csv",    default=str(DEFAULT_CSV))
    p.add_argument("--output", default=str(DEFAULT_OUT))
    return p.parse_args()


def main():
    args     = parse_args()
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

    excl = [extract(rows, "exclusive", b) for b in BENCHMARKS]
    incl = [extract(rows, "inclusive", b) for b in BENCHMARKS]

    missing = [BENCHMARKS[i] for i, v in enumerate(excl + incl) if v is None]
    if missing:
        sys.exit(f"Missing data for benchmarks: {missing}")

    x     = list(range(len(BENCHMARKS)))
    width = 0.36

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(figsize=(8, 5))

    bars_e = ax.bar([p - width/2 for p in x], excl, width=width,
                    color="#0f766e", label="Exclusive")
    bars_i = ax.bar([p + width/2 for p in x], incl, width=width,
                    color="#f59e0b", label="Inclusive")

    for bar in list(bars_e) + list(bars_i):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.01,
                f"{int(bar.get_height()):,}", ha="center", va="bottom", fontsize=8)

    for i, (e, inc) in enumerate(zip(excl, incl)):
        pct   = (inc - e) / inc * 100.0
        color = "#0f766e" if pct >= 0 else "#dc2626"
        ax.annotate(f"{pct:+.1f}%",
                    xy=(i, max(e, inc) * 1.12),
                    ha="center", va="bottom", fontsize=9,
                    color=color, fontweight="bold")

    ax.set_title("Exclusive vs. Inclusive — Default Cache Config", fontsize=13)
    ax.set_ylabel("Cycles")
    ax.set_xticks(x)
    ax.set_xticklabels(BENCHMARKS)
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))
    ax.legend(frameon=False)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
