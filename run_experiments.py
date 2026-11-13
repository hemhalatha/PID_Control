
import os
import subprocess
import sys
from pathlib import Path


EXPERIMENTS = [
    "exp01_step_response",
    "exp02_proportional",
    "exp03_integral",
    "exp04_derivative",
    "exp05_comparison",
    "exp06_windup",
    "exp07_sensor_noise",
    "exp08_derivative_kick",
    "exp09_sampling_time",
    "exp10_ziegler_nichols",
    "exp11_improved_tuning",
]


def main():
    project_dir = Path(__file__).resolve().parent

    # Ensure the output directory exists.
    graph_dir = project_dir / "graphs"
    graph_dir.mkdir(exist_ok=True)

    # Use a non-interactive backend to save plots without
    # opening Matplotlib windows.
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    env["PYTHONUNBUFFERED"] = "1"

    completed = 0
    failed = []

    print("=" * 60)
    print("PID CONTROL - EXPERIMENT RUNNER")
    print("=" * 60)

    for index, experiment in enumerate(EXPERIMENTS, start=1):
        print(
            f"\n[{index}/{len(EXPERIMENTS)}] "
            f"Running {experiment}"
        )
        print("-" * 60, flush=True)

        result = subprocess.run(
            [
                sys.executable,
                "-m",
                f"experiments.{experiment}",
            ],
            cwd=project_dir,
            env=env,
        )

        if result.returncode == 0:
            completed += 1
            print(f"\nCompleted: {experiment}")
        else:
            failed.append(experiment)
            print(f"\nFailed: {experiment}")

    # Display the execution summary.
    print("\n" + "=" * 60)
    print("EXPERIMENT SUMMARY")
    print("=" * 60)

    print(f"Total experiments : {len(EXPERIMENTS)}")
    print(f"Completed         : {completed}")
    print(f"Failed            : {len(failed)}")

    if failed:
        print("\nFailed experiments:")

        for experiment in failed:
            print(f"  - {experiment}")

    graphs = sorted(graph_dir.glob("*.png"))

    print(f"\nGenerated graphs: {len(graphs)}")

    for graph in graphs:
        print(f"  - {graph.name}")

    print(f"\nOutput directory: {graph_dir}")

    if failed:
        return 1

    print("\nAll experiments completed successfully.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
