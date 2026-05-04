#!/usr/bin/env python3
"""
Plot reuse-scan exclusive vs. inclusive results across experiments.

Left panel : all benchmarks under the 'default' experiment — shows which
             benchmarks are cache-sensitive and which aren't.
Right panel: reuse-scan across all experiments — shows how the exclusive
             advantage changes as cache geometry is swept.
"""

import argparse
import csv
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "build" / "experiments" / "csv" / "cache_experiments.csv"
DEFAULT_OUT = ROOT / "build" / "experiments" / "plots" / "reuse_scan.png"

ALL_BENCHMARKS = [
    "vvadd", "cmplx-mult", "masked-filter", "bin-search",
    "conflict-miss", "reuse-scan",
]

SWEEP_EXPERIMENTS = [
    "default",
    "l1-half-capacity",
    "l1-double-capacity",
    "l1-2way",
    "l1-victim-buffer",
    "tiny-l2",
]

SWEEP_LABELS = {
    "default":            "default\n(L1=1KB L2=1KB)",
    "l1-half-capacity":   "l1-half\n(512B L1)",
    "l1-double-capacity": "l1-double\n(2KB L1)",
    "l1-2way":            "l1-2way\n(2-way L1)",
    "l1-victim-buffer":   "l1-vbuf",
    "tiny-l2":            "tiny-l2\n(256B L2)",
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
    present_exps    = {r["experiment"] for r in rows}
    present_benches = {r["benchmark"]  for r in rows}

    if "reuse-scan" not in present_benches:
        sys.exit("reuse-scan benchmark not found in CSV — rebuild VMH files and rerun experiments.")
    if "default" not in present_exps:
        sys.exit("'default' experiment not found in CSV.")

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(15, 6))

    # ------------------------------------------------------------------ #
    # Left: all benchmarks under 'default', exclusive vs inclusive        #
    # ------------------------------------------------------------------ #
    benches_present = [b for b in ALL_BENCHMARKS if b in present_benches]
    x = list(range(len(benches_present)))
    w = 0.36

    excl = [extract(rows, "default", "exclusive", b) for b in benches_present]
    incl = [extract(rows, "default", "inclusive", b) for b in benches_present]

    bars_e = ax_left.bar([p - w/2 for p in x], excl, width=w,
                         color="#0f766e", label="Exclusive")
    bars_i = ax_left.bar([p + w/2 for p in x], incl, width=w,
                         color="#f59e0b", label="Inclusive")

    for bar in list(bars_e) + list(bars_i):
        ax_left.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.01,
                     f"{int(bar.get_height()):,}", ha="center", va="bottom", fontsize=7)

    for i, (e, inc) in enumerate(zip(excl, incl)):
        if e is None or inc is None:
            continue
        pct   = (inc - e) / inc * 100.0
        color = "#0f766e" if pct >= 0 else "#dc2626"
        ax_left.annotate(f"{pct:+.1f}%",
                         xy=(i, max(e, inc) * 1.13),
                         ha="center", va="bottom", fontsize=8,
                         color=color, fontweight="bold")

    ax_left.set_title("Default config — all benchmarks\n(L1=1KB, L2=1KB, L3=64KB)", fontsize=10)
    ax_left.set_ylabel("Cycles")
    ax_left.set_xticks(x)
    ax_left.set_xticklabels(benches_present, rotation=20, ha="right")
    ax_left.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))
    ax_left.legend(frameon=False)

    # ------------------------------------------------------------------ #
    # Right: reuse-scan across the L1/L2 sweep experiments               #
    # ------------------------------------------------------------------ #
    exps_present = [e for e in SWEEP_EXPERIMENTS if e in present_exps]
    x2 = list(range(len(exps_present)))

    excl2 = [extract(rows, e, "exclusive", "reuse-scan") for e in exps_present]
    incl2 = [extract(rows, e, "inclusive", "reuse-scan") for e in exps_present]

    missing = [exps_present[i] for i, v in enumerate(excl2 + incl2) if v is None]
    if missing:
        sys.exit(f"Missing reuse-scan data for experiments: {missing}")

    bars_e2 = ax_right.bar([p - w/2 for p in x2], excl2, width=w,
                            color="#0f766e", label="Exclusive")
    bars_i2 = ax_right.bar([p + w/2 for p in x2], incl2, width=w,
                            color="#f59e0b", label="Inclusive")

    for bar in list(bars_e2) + list(bars_i2):
        ax_right.text(bar.get_x() + bar.get_width()/2, bar.get_height() * 1.01,
                      f"{int(bar.get_height()):,}", ha="center", va="bottom", fontsize=7)

    for i, (e, inc) in enumerate(zip(excl2, incl2)):
        pct   = (inc - e) / inc * 100.0
        color = "#0f766e" if pct >= 0 else "#dc2626"
        ax_right.annotate(f"{pct:+.1f}%",
                          xy=(i, max(e, inc) * 1.13),
                          ha="center", va="bottom", fontsize=8,
                          color=color, fontweight="bold")

    xlabels = [SWEEP_LABELS.get(e, e) for e in exps_present]
    ax_right.set_title("reuse-scan across L1/L2 experiments\n(% = exclusive speedup over inclusive)", fontsize=10)
    ax_right.set_ylabel("Cycles")
    ax_right.set_xticks(x2)
    ax_right.set_xticklabels(xlabels, rotation=0, ha="center", fontsize=8)
    ax_right.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))
    ax_right.legend(frameon=False)

    fig.suptitle("Exclusive vs. Inclusive Cache Hierarchy — reuse-scan benchmark", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
