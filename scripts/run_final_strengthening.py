from __future__ import annotations

import argparse
import json
from pathlib import Path

from calg.experiments.final_strengthening import run_final_strengthening_suite


def main() -> int:
    parser = argparse.ArgumentParser(description="Run CMAME final-strengthening CALG validation suite.")
    parser.add_argument("--out-dir", default="results/v0_7_final_strengthening", help="Output directory")
    args = parser.parse_args()
    out_dir = Path(args.out_dir)
    summary = run_final_strengthening_suite(out_dir)
    print(json.dumps(summary, indent=2))
    print(f"Wrote final-strengthening validation results to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
