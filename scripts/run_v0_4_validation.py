#!/usr/bin/env python
from __future__ import annotations
import argparse
from pathlib import Path
from calg.experiments.gpu_benchmark import run_gpu_benchmark
from calg.experiments.convergence import run_convergence_study
from calg.experiments.baseline_comparison import run_baseline_comparison, comparison_cases
from calg.experiments.robustness import run_robustness_study


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', default='results/v0_4')
    p.add_argument('--quick', action='store_true')
    args = p.parse_args()
    out = Path(args.out_dir)
    if args.quick:
        run_gpu_benchmark(out / 'gpu_benchmark', sizes=[256, 512], repeats=1)
        run_convergence_study(out / 'convergence', resolutions=[4, 6, 8], families=['sphere_plane','paraboloid_plane'])
        run_baseline_comparison(out / 'baselines', cases=comparison_cases()[:3])
        run_robustness_study(out / 'robustness', num_cases=6, seed=7)
    else:
        run_gpu_benchmark(out / 'gpu_benchmark')
        run_convergence_study(out / 'convergence')
        run_baseline_comparison(out / 'baselines')
        run_robustness_study(out / 'robustness', num_cases=21)

if __name__ == '__main__':
    main()
