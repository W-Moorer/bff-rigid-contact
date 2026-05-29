#!/usr/bin/env python
from __future__ import annotations
import argparse
from calg.experiments.convergence import run_convergence_study


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--out-dir', default='results/v0_4_convergence')
    p.add_argument('--resolutions', default='4,6,8,10,12')
    p.add_argument('--families', default='sphere_plane,paraboloid_plane,cylinder_plane')
    args = p.parse_args()
    resolutions = [int(x) for x in args.resolutions.split(',') if x.strip()]
    families = [x.strip() for x in args.families.split(',') if x.strip()]
    run_convergence_study(args.out_dir, resolutions=resolutions, families=families)

if __name__ == '__main__':
    main()
