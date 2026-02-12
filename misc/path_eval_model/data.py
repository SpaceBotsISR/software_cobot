#!/usr/bin/env python3
import argparse
import os
from typing import Iterable

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from path_eval_model import load_goal_mode


def compute_success_components(
    df: pd.DataFrame, kc: float = 1.0, kt: float = 10.0, eps: float = 1e-12
) -> dict[str, np.ndarray]:
    """
    Components of:
      q_completion = EXE / PLN
      q_collision = exp(-kc * NRC)
      q_tracking = -kt * RMS
    """
    L = df["PLN"].to_numpy(float)
    Lexec = df["EXE"].to_numpy(float)
    Nc = df["NRC"].to_numpy(float)
    erms = df["RMS"].to_numpy(float)

    q_completion = Lexec / np.maximum(L, eps)
    q_collision = np.exp(-kc * Nc)
    q_tracking = -kt * erms

    return {
        "q_completion (EXE/PLN)": np.clip(q_completion, 0.0, 1.0),
        "q_collision exp(-kc*NRC)": np.clip(q_collision, 0.0, 1.0),
        "q_tracking (-kt*RMS)": q_tracking,
    }


def _ensure_columns(df: pd.DataFrame, cols: Iterable[str]) -> None:
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise RuntimeError(f"Missing columns: {missing}")


def plot_inputs_vs_components(
    df: pd.DataFrame,
    inputs: list[str],
    kc: float,
    kt: float,
    out_path: str | None,
) -> None:
    components = compute_success_components(df, kc=kc, kt=kt)
    input_labels = {
        "CLR": "minimum clearance (CLR)",
        "NAR": "narrow passage exposure (NAR)",
        "TRN": "maximum turn angle (TRN)",
        "EFF": "path efficiency (EFF)",
    }

    for comp_name, y in components.items():
        fig, axes = plt.subplots(
            2,
            2,
            figsize=(8.4, 6.0),
            sharey=True,
        )
        flat_axes = axes.ravel()

        for j, x_name in enumerate(inputs):
            ax = flat_axes[j]
            x = df[x_name].to_numpy(float)
            order = np.argsort(x)
            xs = x[order]
            ys = y[order]

            ax.plot(
                xs,
                ys,
                color="#1f77b4",
                alpha=0.35,
                linewidth=1.2,
            )
            ax.scatter(
                xs,
                ys,
                s=20,
                alpha=0.75,
                edgecolors="none",
                color="#1f77b4",
            )
            ax.set_xlabel(input_labels.get(x_name, x_name))
            if j % 2 == 0:
                ax.set_ylabel(comp_name)
            ax.set_title(f"{input_labels.get(x_name, x_name)} x {comp_name}")
            ax.grid(alpha=0.25)

        if len(inputs) < len(flat_axes):
            for ax in flat_axes[len(inputs) :]:
                ax.set_visible(False)

        fig.suptitle(f"{comp_name} vs Inputs (kc={kc}, kt={kt})", y=1.03)
        fig.tight_layout()

        if out_path:
            base, ext = os.path.splitext(out_path)
            ext = ext or ".png"
            safe = comp_name.replace("/", "_").replace(" ", "_").replace("(", "").replace(")", "")
            fig.savefig(f"{base}_{safe}{ext}", dpi=150)
        else:
            plt.show()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="Path to goal_mode.txt")
    args = ap.parse_args()

    kc = 1.0
    kt = 10.0
    inputs = ["CLR", "NAR", "TRN", "EFF"]

    df = load_goal_mode(args.path)
    _ensure_columns(df, inputs + ["PLN", "EXE", "RMS", "NRC"])

    plot_inputs_vs_components(
        df=df,
        inputs=inputs,
        kc=kc,
        kt=kt,
        out_path=None,
    )


if __name__ == "__main__":
    main()
