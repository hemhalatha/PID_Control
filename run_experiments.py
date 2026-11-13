"""Run all report experiments and check their output files."""

import os
from pathlib import Path
import subprocess
import sys

EXPERIMENTS = (
    ("exp01_step_response", "04_metrics.png"),
    ("exp02_proportional", "05a_p_only.png"),
    ("exp03_integral", "05b_integral.png"),
    ("exp04_derivative", "05c_derivative.png"),
    ("exp05_comparison", "06_comparison.png"),
    ("exp06_windup", "07a_windup.png"),
    ("exp07_sensor_noise", "07b_noise.png"),
    ("exp08_derivative_kick", "07c_kick.png"),
    ("exp09_sampling_time", "07d_sampling.png"),
    ("exp10_ziegler_nichols", "08a_ultimate_gain.png"),
    ("exp11_improved_tuning", "08b_zn_tuning.png"),
)


def main():
    root = Path(__file__).resolve().parent
    graphs = root / "graphs"
    graphs.mkdir(exist_ok=True)
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    env["PYTHONIOENCODING"] = "utf-8"
    failed = []
    for i, (module, filename) in enumerate(EXPERIMENTS, 1):
        print(f"\n[{i}/{len(EXPERIMENTS)}] {module}", flush=True)
        output = graphs / filename
        before = output.stat().st_mtime_ns if output.exists() else None
        result = subprocess.run([sys.executable, "-m", f"experiments.{module}"],
                                cwd=root, env=env)
        fresh = output.is_file() and output.stat().st_size > 0 and (
            before is None or output.stat().st_mtime_ns != before)
        if result.returncode != 0 or not fresh:
            failed.append(module)
    print(f"\nCompleted: {len(EXPERIMENTS) - len(failed)}/{len(EXPERIMENTS)}")
    if failed:
        print("Failed or missing fresh output:", ", ".join(failed))
        return 1
    print("All 11 report figures regenerated in graphs/.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
