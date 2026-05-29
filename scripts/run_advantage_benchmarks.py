from __future__ import annotations

import argparse
import json
from pathlib import Path

from calg.experiments.advantage import run_advantage_suite


def main() -> int:
    parser = argparse.ArgumentParser(description="Run advantage-focused CALG validation cases.")
    parser.add_argument("--out-dir", default="results/v0_6_advantage", help="Output directory")
    args = parser.parse_args()
    out_dir = Path(args.out_dir)
    summary = run_advantage_suite(out_dir)
    (out_dir / "advantage_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf8")
    print(f"Wrote advantage validation results to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
