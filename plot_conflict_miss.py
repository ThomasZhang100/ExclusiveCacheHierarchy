#!/usr/bin/env python3

import argparse
import csv
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent
DEFAULT_CSV = ROOT / "build" / "experiments" / "csv" / "cache_experiments.csv"
DEFAULT_OUT = ROOT / "build" / "experiments" / "plots" / "conflict_miss_l1_sweep.png"

EXPERIMENTS = [
    "default",
    "l1-half-capacity",
    "l1-double-capacity",
    "l1-2way",
    "l1-victim-buffer",
]

LABELS = {
    "default":            "default\n(64s, 1w)",
    "l1-half-capacity":   "l1-half\n(32s, 1w)",
    "l1-double-capacity": "l1-double\n(128s, 1w)",
    "l1-2way":            "l1-2way\n(64s, 2w)",
    "l1-victim-buffer":   "l1-victim\n(64s, 1w+vb)",
}


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def extract(rows, experiment, variant):
    for row in rows:
        if row["experiment"] == experiment and row["variant"] == variant and row["benchmark"] == "conflict-miss":
            return int(row["num_cycles"])
    return None


def parse_args():
    p = argparse.ArgumentParser(description="Plot conflict-miss L1 sweep.")
    p.add_argument("--csv", default=str(DEFAULT_CSV))
    p.add_argument("--output", default=str(DEFAULT_OUT))
    return p.parse_args()


def main():
    args = parse_args()
    csv_path = pathlib.Path(args.csv)
    out_path = pathlib.Path(args.output)

    if not csv_path.exists():
        sys.exit(f"CSV not found: {csv_path}")

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is required: pip install matplotlib")

    rows = read_csv(csv_path)

    missing = [e for e in EXPERIMENTS if e not in {r["experiment"] for r in rows}]
    if missing:
        sys.exit(f"Experiments not found in CSV: {missing}")

    excl = []
    incl = []
    for exp in EXPERIMENTS:
        e = extract(rows, exp, "exclusive")
        i = extract(rows, exp, "inclusive")
        if e is None or i is None:
            sys.exit(f"Missing conflict-miss data for experiment '{exp}'")
        excl.append(e)
        incl.append(i)

    x = list(range(len(EXPERIMENTS)))
    width = 0.36
    labels = [LABELS[e] for e in EXPERIMENTS]

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, ax = plt.subplots(figsize=(10, 5.5))

    bars_e = ax.bar([p - width / 2 for p in x], excl, width=width,
                    color="#0f766e", label="Exclusive")
    bars_i = ax.bar([p + width / 2 for p in x], incl, width=width,
                    color="#f59e0b", label="Inclusive")

    for bar in bars_e:
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 30,
                str(int(bar.get_height())), ha="center", va="bottom", fontsize=8)
    for bar in bars_i:
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 30,
                str(int(bar.get_height())), ha="center", va="bottom", fontsize=8)

    ax.set_title("conflict-miss: Cycles by L1 Configuration")
    ax.set_ylabel("Cycles")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend(frameon=False)

    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=220)
    plt.close(fig)
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
