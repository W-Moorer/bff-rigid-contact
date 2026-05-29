#!/usr/bin/env python
from __future__ import annotations
import argparse
from calg.experiments.baseline_comparison import run_baseline_comparison


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', default='results/v0_4_baselines')
    args = p.parse_args()
    run_baseline_comparison(args.out_dir)

if __name__ == '__main__':
    main()
