#!/usr/bin/env python3

import argparse
import csv
import pathlib
import re
import shutil
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parent
RISCVLONG_DIR = ROOT / "riscvlong"
UBMARK_VMH_DIR = ROOT / "ubmark" / "build" / "vmh"
BUILD_EXPERIMENTS_DIR = ROOT / "build" / "experiments"
GENERATED_SIM_DIR = BUILD_EXPERIMENTS_DIR / "generated"

def _find_vvp():
    iverilog_path = shutil.which("iverilog")
    if iverilog_path:
        candidate = pathlib.Path(iverilog_path).parent / "vvp"
        if candidate.exists():
            return str(candidate)
    return "vvp"


VVP = _find_vvp()

COMP_FLAGS = [
    "iverilog",
    "-g2005",
    "-Wall",
    "-Wno-sensitivity-entire-vector",
    "-Wno-sensitivity-entire-array",
]

INCLUDE_FLAGS = [
    "-I",
    str(ROOT / "riscvlong"),
    "-I",
    str(ROOT / "vc"),
    "-I",
    str(ROOT / "imuldiv"),
    "-I",
    str(ROOT / "cache"),
    "-I",
    str(ROOT / "inclcache"),
]

DEFAULT_PARAMS = {
    "p_l1i_num_sets": 64,
    "p_l1i_num_ways": 1,
    "p_l1i_line_sz": 16,
    "p_l1i_use_victim_buf": 0,
    "p_l1i_hit_lat": 1,
    "p_l1d_num_sets": 64,
    "p_l1d_num_ways": 1,
    "p_l1d_line_sz": 16,
    "p_l1d_use_victim_buf": 0,
    "p_l1d_hit_lat": 1,
    "p_l2_num_sets": 128,
    "p_l2_num_ways": 4,
    "p_l2_line_sz": 16,
    "p_l2_hit_lat": 4,
    "p_l3_num_sets": 512,
    "p_l3_num_ways": 8,
    "p_l3_line_sz": 16,
    "p_l3_hit_lat": 10,
}

EXPERIMENTS = {
    "default": {},
    "tiny-l2": {
        # L2 shrunk to 1 KB (= L1 capacity). Inclusive: L1⊆L2 means L2
        # holds nothing beyond L1 content, so every L1 eviction goes to L3.
        # Exclusive: L1+L2 = 2 KB of unique capacity; evictions land in L2.
        "p_l2_num_sets": 64,
        "p_l2_num_ways": 1,
    },
    "l1-half-capacity": {
        "p_l1i_num_sets": 32,
        "p_l1d_num_sets": 32,
    },
    "l1-double-capacity": {
        "p_l1i_num_sets": 128,
        "p_l1d_num_sets": 128,
    },
    "l1-2way": {
        "p_l1i_num_ways": 2,
        "p_l1d_num_ways": 2,
    },
    "l1-victim-buffer": {
        "p_l1i_use_victim_buf": 1,
        "p_l1d_use_victim_buf": 1,
    },
    "l2-low-assoc": {
        "p_l2_num_ways": 2,
    },
    "l2-high-assoc": {
        "p_l2_num_ways": 8,
    },
    "l3-half-capacity": {
        "p_l3_num_sets": 256,
    },
    "l3-double-capacity": {
        "p_l3_num_sets": 1024,
    },
}

VARIANTS = {
    "exclusive": {
        "template": RISCVLONG_DIR / "riscvlong-cache-sim.v",
        "generated": "generated-riscvlong-cache-sim-{experiment}.v",
        "exe": "riscvlong-cache-sim-{experiment}",
    },
    "inclusive": {
        "template": RISCVLONG_DIR / "riscvlong-inclcache-sim.v",
        "generated": "generated-riscvlong-inclcache-sim-{experiment}.v",
        "exe": "riscvlong-inclcache-sim-{experiment}",
    },
}

STAT_PATTERNS = {
    "status": re.compile(r"status\s*=\s*(\d+)"),
    "num_cycles": re.compile(r"num_cycles\s*=\s*(\d+)"),
    "num_inst": re.compile(r"num_inst\s*=\s*(\d+)"),
    "ipc": re.compile(r"ipc\s*=\s*([0-9.]+)"),
}


def discover_benchmarks():
    benches = sorted(UBMARK_VMH_DIR.glob("ubmark-*.vmh"))
    if not benches:
        raise SystemExit(f"No benchmark vmh files found in {UBMARK_VMH_DIR}")
    return benches


def merged_params(overrides):
    params = dict(DEFAULT_PARAMS)
    params.update(overrides)
    return params


def render_sim_source(template_path, params):
    text = template_path.read_text()
    text = text.replace('../cache/cache-ExclusiveCacheHier.v', 'cache-ExclusiveCacheHier.v')
    text = text.replace('../inclcache/inclcache-InclusiveCacheHier.v', 'inclcache-InclusiveCacheHier.v')
    for key, value in params.items():
        pattern = re.compile(rf"(\.{re.escape(key)}\s*\()\s*\d+\s*(\))")
        text, count = pattern.subn(rf"\g<1>{value}\2", text, count=1)
        if count != 1:
            raise ValueError(f"Could not replace {key} in {template_path}")
    return text


def compile_variant(variant_name, experiment_name, params, force_rebuild):
    variant = VARIANTS[variant_name]
    generated_name = variant["generated"].format(experiment=experiment_name)
    generated_path = GENERATED_SIM_DIR / generated_name
    exe_path = BUILD_EXPERIMENTS_DIR / "bin" / variant["exe"].format(experiment=experiment_name)
    compile_log = BUILD_EXPERIMENTS_DIR / "logs" / f"{variant_name}-{experiment_name}-compile.log"

    sim_source = render_sim_source(variant["template"], params)
    generated_path.write_text(sim_source)

    if exe_path.exists() and not force_rebuild:
        return generated_path, exe_path

    cmd = COMP_FLAGS + ["-o", str(exe_path)] + INCLUDE_FLAGS + [str(generated_path)]
    result = subprocess.run(
        cmd,
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    compile_log.write_text(result.stdout + result.stderr)
    if result.returncode != 0:
        raise RuntimeError(
            f"Compile failed for {variant_name}/{experiment_name}. See {compile_log}"
        )
    return generated_path, exe_path


def parse_stats(stdout_text):
    stats = {}
    for key, pattern in STAT_PATTERNS.items():
        match = pattern.search(stdout_text)
        if not match:
            raise RuntimeError(f"Could not parse {key} from simulator output")
        stats[key] = match.group(1)
    return stats


def run_benchmark(exe_path, variant_name, experiment_name, benchmark_path, max_cycles):
    raw_out_dir = BUILD_EXPERIMENTS_DIR / "raw" / variant_name / experiment_name
    raw_out_dir.mkdir(parents=True, exist_ok=True)
    raw_out_path = raw_out_dir / f"{benchmark_path.stem}.out"

    cmd = [
        VVP,
        str(exe_path),
        "+verbose=1",
        f"+max-cycles={max_cycles}",
        f"+exe={benchmark_path}",
    ]
    result = subprocess.run(
        cmd,
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    raw_out_path.write_text(result.stdout + result.stderr)

    if result.returncode != 0:
        raise RuntimeError(
            f"Simulation failed for {variant_name}/{experiment_name}/{benchmark_path.stem}. "
            f"See {raw_out_path}"
        )

    stats = parse_stats(result.stdout)
    return {
        "benchmark": benchmark_path.stem.removeprefix("ubmark-"),
        "status": int(stats["status"]),
        "num_cycles": int(stats["num_cycles"]),
        "num_inst": int(stats["num_inst"]),
        "ipc": float(stats["ipc"]),
        "raw_output": str(raw_out_path.relative_to(ROOT)),
    }


def ensure_dirs():
    for subdir in ["bin", "csv", "generated", "logs", "plots", "raw"]:
        (BUILD_EXPERIMENTS_DIR / subdir).mkdir(parents=True, exist_ok=True)


def write_csv(rows, output_path):
    fieldnames = [
        "experiment",
        "variant",
        "benchmark",
        "status",
        "num_cycles",
        "num_inst",
        "ipc",
        "raw_output",
    ] + sorted(DEFAULT_PARAMS)
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def parse_args():
    parser = argparse.ArgumentParser(description="Run cache hierarchy benchmark sweeps.")
    parser.add_argument(
        "--experiments",
        nargs="+",
        default=list(EXPERIMENTS),
        choices=list(EXPERIMENTS),
        help="Named experiments to run.",
    )
    parser.add_argument(
        "--variants",
        nargs="+",
        default=list(VARIANTS),
        choices=list(VARIANTS),
        help="Cache hierarchy variants to run.",
    )
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=1000000,
        help="Maximum simulated cycles per benchmark run.",
    )
    parser.add_argument(
        "--force-rebuild",
        action="store_true",
        help="Always recompile generated simulators.",
    )
    parser.add_argument(
        "--output",
        default=str(BUILD_EXPERIMENTS_DIR / "csv" / "cache_experiments.csv"),
        help="CSV output path.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    ensure_dirs()
    benchmarks = discover_benchmarks()
    rows = []

    print(f"Discovered {len(benchmarks)} benchmarks in {UBMARK_VMH_DIR}")
    print(f"Running experiments: {', '.join(args.experiments)}")
    print(f"Running variants: {', '.join(args.variants)}")

    for experiment_name in args.experiments:
        params = merged_params(EXPERIMENTS[experiment_name])
        print(f"\n[{experiment_name}]")

        for variant_name in args.variants:
            _, exe_path = compile_variant(
                variant_name,
                experiment_name,
                params,
                args.force_rebuild,
            )
            print(f"  {variant_name}: compiled {exe_path.name}")

            for benchmark_path in benchmarks:
                stats = run_benchmark(
                    exe_path,
                    variant_name,
                    experiment_name,
                    benchmark_path,
                    args.max_cycles,
                )
                row = {
                    "experiment": experiment_name,
                    "variant": variant_name,
                    **stats,
                    **params,
                }
                rows.append(row)
                print(
                    f"    {stats['benchmark']}: cycles={stats['num_cycles']} "
                    f"ipc={stats['ipc']:.6f}"
                )
                if stats["status"] != 1:
                    raise RuntimeError(
                        f"Benchmark reported failure for "
                        f"{variant_name}/{experiment_name}/{stats['benchmark']}"
                    )

    output_path = pathlib.Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_csv(rows, output_path)
    print(f"\nWrote {len(rows)} rows to {output_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
