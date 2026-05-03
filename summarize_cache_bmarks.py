#!/usr/bin/env python3

import pathlib
import re


ROOT = pathlib.Path(__file__).resolve().parent
BUILD = ROOT / "build"

PATTERNS = {
    "status": re.compile(r"status\s*=\s*(\d+)"),
    "num_cycles": re.compile(r"num_cycles\s*=\s*(\d+)"),
    "num_inst": re.compile(r"num_inst\s*=\s*(\d+)"),
    "ipc": re.compile(r"ipc\s*=\s*([0-9.]+)"),
}


def parse_stats(path: pathlib.Path):
    text = path.read_text()
    data = {}
    for key, pattern in PATTERNS.items():
        match = pattern.search(text)
        data[key] = match.group(1) if match else "?"
    return data


def load_variant(suffix: str):
    stats = {}
    for path in sorted(BUILD.glob(f"ubmark-*-long-{suffix}.out")):
        bench = path.name.removesuffix(f"-long-{suffix}.out").removeprefix("ubmark-")
        stats[bench] = parse_stats(path)
    return stats


def main():
    cache = load_variant("cache")
    incl = load_variant("inclcache")
    benches = sorted(set(cache) | set(incl))

    print("| benchmark | excl_cycles | incl_cycles | delta | excl_ipc | incl_ipc |")
    print("|---|---:|---:|---:|---:|---:|")

    for bench in benches:
        c = cache.get(bench, {})
        i = incl.get(bench, {})
        c_cycles = int(c.get("num_cycles", "0"))
        i_cycles = int(i.get("num_cycles", "0"))
        delta = i_cycles - c_cycles
        print(
            f"| {bench} | {c.get('num_cycles', '?')} | {i.get('num_cycles', '?')} | "
            f"{delta:+d} | {c.get('ipc', '?')} | {i.get('ipc', '?')} |"
        )


if __name__ == "__main__":
    main()
