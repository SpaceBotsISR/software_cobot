#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd
import os
import joblib

from sklearn.model_selection import train_test_split
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import StandardScaler, PolynomialFeatures
from sklearn.linear_model import (
    Ridge,
    ElasticNet,
    Lasso,
    HuberRegressor,
)
from sklearn.svm import SVR
from sklearn.neighbors import KNeighborsRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.metrics import mean_squared_error, r2_score


# --------------------------------------------------
# Success score definition (ground truth y)
# --------------------------------------------------
def compute_success_y(
    df: pd.DataFrame, kc: float = 1.0, kt: float = 5.0, eps: float = 1e-12
) -> np.ndarray:
    """
    y = (EXE / PLN) * exp(-kc * NRC) * exp(-kt * RMS)
    """
    L = df["PLN"].to_numpy(float)
    Lexec = df["EXE"].to_numpy(float)
    Nc = df["NRC"].to_numpy(float)
    erms = df["RMS"].to_numpy(float)

    q_completion = Lexec / np.maximum(L, eps)
    q_collision = np.exp(-kc * Nc)
    q_tracking = np.exp(-kt * erms)

    y = q_completion * q_collision * q_tracking
    return np.clip(y, 0.0, 1.0)


# --------------------------------------------------
# Model zoo
# --------------------------------------------------
def get_models(random_state: int = 42) -> dict[str, object]:
    return {
        # --------------------
        # Linear / robust
        # --------------------
        "Ridge": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", Ridge(alpha=1.0)),
            ]
        ),
        "ElasticNet": Pipeline(
            [
                ("scaler", StandardScaler()),
                (
                    "model",
                    ElasticNet(
                        alpha=1e-1,
                        l1_ratio=0.3,
                        max_iter=200000,
                        tol=1e-3,
                        random_state=random_state,
                    ),
                ),
            ]
        ),
        "Lasso": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", Lasso(alpha=1e-2, max_iter=200000, tol=1e-3)),
            ]
        ),
        "Huber": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", HuberRegressor(max_iter=10000)),
            ]
        ),
        # --------------------
        # Smooth nonlinear
        # --------------------
        "PolyRidge(d2)": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("poly", PolynomialFeatures(degree=2, include_bias=False)),
                ("model", Ridge(alpha=1.0)),
            ]
        ),
        "SVR(RBF)": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", SVR(kernel="rbf", C=1.0, epsilon=0.05)),
            ]
        ),
        "SVR(Linear)": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", SVR(kernel="linear", C=1.0, epsilon=0.05)),
            ]
        ),
        "KNN": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", KNeighborsRegressor(n_neighbors=7, weights="distance")),
            ]
        ),
        "MLP": Pipeline(
            [
                ("scaler", StandardScaler()),
                (
                    "model",
                    MLPRegressor(
                        hidden_layer_sizes=(32, 16),
                        activation="relu",
                        alpha=1e-4,
                        max_iter=2000,
                        random_state=random_state,
                    ),
                ),
            ]
        ),
        "HistGBR": HistGradientBoostingRegressor(
            random_state=random_state,
            max_iter=500,
            learning_rate=0.05,
            max_depth=4,
            min_samples_leaf=8,
        ),
    }


# --------------------------------------------------
# File loader (robust, no pandas magic)
# --------------------------------------------------
def load_goal_mode(path: str) -> pd.DataFrame:
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip()]

    if not lines:
        raise RuntimeError("Empty data file")

    header = lines[0].split()
    rows = []

    for ln in lines[1:]:
        parts = ln.split()
        if len(parts) != len(header):
            raise RuntimeError(
                f"Row has {len(parts)} fields, expected {len(header)}: {ln}"
            )
        rows.append([float(x) for x in parts])

    return pd.DataFrame(rows, columns=header)


def build_feature_matrix(df: pd.DataFrame, eps: float = 1e-12) -> tuple[np.ndarray, list[str]]:
    base_features = ["CLR", "NAR", "TRN", "EFF"]
    raw_features = ["MCLR", "NFR", "MTR", "PLN", "STD", "DTR"]

    if "DTR" not in df.columns and "PLN" in df.columns and "STD" in df.columns:
        dtr = df["PLN"].to_numpy(float) / np.maximum(df["STD"].to_numpy(float), eps)
        df["DTR"] = dtr

    selected: list[str] = []
    for name in base_features + raw_features:
        if name in df.columns:
            selected.append(name)

    if not selected:
        raise RuntimeError("No feature columns found.")
    return df[selected].to_numpy(float), selected


# --------------------------------------------------
# Main
# --------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="Path to goal_mode.txt")
    ap.add_argument("--kc", type=float, default=1.0)
    ap.add_argument("--kt", type=float, default=5.0)
    args = ap.parse_args()

    df = load_goal_mode(args.path)

    required = ["PLN", "EXE", "RMS", "NRC"]
    for c in required:
        if c not in df.columns:
            raise RuntimeError(f"Missing column {c}")

    # Inputs (normalized + raw metrics when available).
    X, selected_features = build_feature_matrix(df)
    print("Using features:", ", ".join(selected_features))

    # Target (computed success)
    y = compute_success_y(df, kc=args.kc, kt=args.kt)

    # Train / test split
    Xtr, Xte, ytr, yte = train_test_split(
        X, y, test_size=0.2, shuffle=True, random_state=42
    )

    models = get_models()
    results: dict[str, tuple[object, float, float]] = {}

    print("\n=== Fixed-parameter model fit (train), then test eval ===")
    print(f"kc={args.kc}, kt={args.kt}\n")

    for name, model in models.items():
        model.fit(Xtr, ytr)
        yhat = np.clip(model.predict(Xte), 0.0, 1.0)

        mse = mean_squared_error(yte, yhat)
        r2 = r2_score(yte, yhat)

        results[name] = (model, mse, r2)
        print(f"{name:18s} | MSE = {mse:.6e} | R² = {r2:.4f}")

    best_name = min(results, key=lambda k: results[k][1])
    best_model = results[best_name][0]
    best_mse = results[best_name][1]
    best_r2 = results[best_name][2]
    print(
        f"\nBest model (by MSE): {best_name} | MSE = {best_mse:.6e} | R² = {best_r2:.4f}"
    )

    base = os.path.splitext(os.path.basename(args.path))[0]
    out_path = f"{base}_model.joblib"
    joblib.dump(best_model, out_path)
    print(f"Saved best model to: {out_path}")

    # If a linear model wins, print coefficients and save them to a numpy file.
    if isinstance(best_model, Pipeline):
        model_step = best_model.named_steps.get("model")
        if hasattr(model_step, "coef_"):
            coefs = np.asarray(model_step.coef_, dtype=float).ravel()
            coef_path = f"{base}_coefs.npy"
            np.save(coef_path, coefs)
            print(f"Saved coefficients to: {coef_path}")

            # Build a readable formula: H = b0 + c1*var1 + ...
            poly = best_model.named_steps.get("poly")
            if poly is not None:
                feature_names = poly.get_feature_names_out(selected_features)
            else:
                feature_names = selected_features

            intercept = getattr(model_step, "intercept_", 0.0)
            terms = [f"{intercept:.6g}"]
            for name, coef in zip(feature_names, coefs):
                terms.append(f"{coef:.6g}*{name}")
            formula = "H = " + " + ".join(terms)
            print("Best linear model formula:")
            print(formula)


if __name__ == "__main__":
    main()
