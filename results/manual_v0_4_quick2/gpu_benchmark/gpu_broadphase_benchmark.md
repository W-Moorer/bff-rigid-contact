# GPU broad-phase benchmark

This run did not detect a CUDA backend. The numbers below are NumPy fallback timings and must not be reported as GPU acceleration. Re-run `scripts/run_gpu_benchmark.py` on the CUDA workstation to generate true GPU data.

| nA | nB | backend | CUDA? | pairs | time (s) | M tests/s | notes |
|---:|---:|---|---|---:|---:|---:|---|
| 256 | 256 | numpy | False | 3 | 0.00683173 | 9.593 | CPU/NumPy fallback; run this script on a CUDA workstation for true GPU acceleration data |
| 512 | 512 | numpy | False | 5 | 0.0202065 | 12.973 | CPU/NumPy fallback; run this script on a CUDA workstation for true GPU acceleration data |
