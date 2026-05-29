from __future__ import annotations

import argparse
from pathlib import Path

from calg.experiments.runner import run_cases


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run CALG validation examples")
    parser.add_argument("--out-dir", default="results/validation", help="Output directory for CSV/JSON/figures")
    parser.add_argument("--plots", action="store_true", help="Enable matplotlib figures")
    args = parser.parse_args(argv)
    metrics = run_cases(Path(args.out_dir), make_plots=args.plots)
    print(f"Wrote validation outputs to {args.out_dir}")
    for m in metrics:
        print(f"{m.case}: min_gap={m.min_gap:.8g}, expected={m.expected_gap}, contacts={m.contacts}, fast_path={m.fast_path_ratio:.3f}")
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
