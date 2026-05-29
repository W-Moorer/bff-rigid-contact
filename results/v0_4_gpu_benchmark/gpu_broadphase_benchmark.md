# GPU broad-phase benchmark

This run did not detect a CUDA backend. The numbers below are NumPy fallback timings and must not be reported as GPU acceleration. Re-run `scripts/run_gpu_benchmark.py` on the CUDA workstation to generate true GPU data.

| nA | nB | backend | CUDA? | pairs | time (s) | M tests/s | notes |
|---:|---:|---|---|---:|---:|---:|---|
| 512 | 512 | numpy | False | 11 | 0.0167994 | 15.604 | CPU/NumPy fallback; run this script on a CUDA workstation for true GPU acceleration data |
| 1024 | 1024 | numpy | False | 35 | 0.0730011 | 14.364 | CPU/NumPy fallback; run this script on a CUDA workstation for true GPU acceleration data |
| 2048 | 2048 | numpy | False | 118 | 0.23322 | 17.984 | CPU/NumPy fallback; run this script on a CUDA workstation for true GPU acceleration data |
