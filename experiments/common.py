"""Shared plotting helpers for consistent, standalone report figures."""

from pathlib import Path

import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "graphs"


def base(title, ylabel="Position (m)", xlabel="Time (s)"):
    fig, ax = plt.subplots(figsize=(8, 4.4))
    ax.set(title=title, xlabel=xlabel, ylabel=ylabel)
    ax.grid(alpha=0.25)
    return fig, ax


def trace(ax, run, label, field="position", **kwargs):
    return ax.plot(run["time"], run[field], label=label, linewidth=1.7, **kwargs)


def target(ax, xmax, level=1.0, units="m"):
    label = f"Target ({level:g} {units})" if units else f"Target ({level:g})"
    ax.axhline(level, color="black", linestyle="--", linewidth=1.2, label=label)
    ax.set_xlim(0, xmax)


def save(fig, filename, summary):
    OUT.mkdir(exist_ok=True)
    fig.tight_layout()
    fig.savefig(OUT / filename, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"{filename}: {summary}")
