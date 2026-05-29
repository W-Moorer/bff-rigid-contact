#!/usr/bin/env python
from __future__ import annotations
import argparse
from calg.experiments.robustness import run_robustness_study


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', default='results/v0_4_robustness')
    p.add_argument('--num-cases', type=int, default=21)
    p.add_argument('--seed', type=int, default=7)
    args = p.parse_args()
    run_robustness_study(args.out_dir, num_cases=args.num_cases, seed=args.seed)

if __name__ == '__main__':
    main()
