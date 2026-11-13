"""Section 6: compare four controllers and export the report table."""

import csv

from experiments.common import ROOT, base, save, target, trace
from metrics import format_number, print_metrics, response_metrics
from plant import run_response

CASES = {"P": (100, 0, 0), "PI": (100, 120, 0),
         "PD": (100, 0, 18), "PID": (100, 100, 15)}


def main():
    fig, ax = base("6. P, PI, PD and PID on the same spring system")
    rows = []
    for label, gains in CASES.items():
        run = run_response(*gains, duration=8)
        trace(ax, run, f"{label}: {gains}")
        met = response_metrics(run)
        print_metrics(label, met)
        rows.append([label, *gains, *[format_number(met[key]) for key in (
            "rise_time", "overshoot", "settling_time", "steady_state_error")]])
    target(ax, 8)
    ax.legend(fontsize=8)
    save(fig, "06_comparison.png", "Matched plant and initial conditions.")

    with (ROOT / "results.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["Controller", "Kp_N_per_m", "Ki_N_per_m_s", "Kd_N_s_per_m",
                         "Rise_10_90_s", "Overshoot_percent", "Settling_2pct_s",
                         "Steady_state_error_m"])
        writer.writerows(rows)
    print("Comparison table saved to results.csv")


if __name__ == "__main__":
    main()
