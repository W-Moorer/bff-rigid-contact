#!/usr/bin/env python
from __future__ import annotations

import argparse
from calg.experiments.gpu_benchmark import run_gpu_benchmark


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", default="results/v0_4_gpu_benchmark")
    p.add_argument("--sizes", default="512,1024,2048,4096")
    p.add_argument("--repeats", type=int, default=3)
    args = p.parse_args()
    sizes = [int(x) for x in args.sizes.split(",") if x.strip()]
    run_gpu_benchmark(args.out_dir, sizes=sizes, repeats=args.repeats)


if __name__ == "__main__":
    main()
