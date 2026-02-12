#!/usr/bin/env python3
import argparse
import numpy as np
import pandas as pd
import os
import joblib

from sklearn.model_selection import train_test_split, GridSearchCV
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
from sklearn.metrics import mean_squared_error, r2_score


# --------------------------------------------------
# Success score definition (ground truth y)
# --------------------------------------------------
def compute_success_y(
    df: pd.DataFrame, kc: float = 1.0, kt: float = 10.0, eps: float = 1e-12
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
                        alpha=1e-2,
                        l1_ratio=0.3,
                        max_iter=1000000,
                        random_state=random_state,
                    ),
                ),
            ]
        ),
        "Lasso": Pipeline(
            [
                ("scaler", StandardScaler()),
                ("model", Lasso(alpha=1e-3, max_iter=1000000)),
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
    }


def get_param_grids() -> dict[str, dict[str, list[object]]]:
    return {
        "Ridge": {
            "model__alpha": [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0, 1e3],
        },
        "ElasticNet": {
            "model__alpha": [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0],
            "model__l1_ratio": [0.01, 0.05, 0.1, 0.3, 0.6, 0.9, 0.98, 0.995],
        },
        "Lasso": {
            "model__alpha": [1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0],
        },
        "Huber": {
            "model__epsilon": [1.01, 1.05, 1.1, 1.2, 1.35, 1.5, 1.75, 2.0, 2.5],
            "model__alpha": [1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1],
        },
        "PolyRidge(d2)": {
            "poly__degree": [2, 3, 4, 5],
            "model__alpha": [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0],
        },
        "SVR(RBF)": {
            "model__C": [1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0, 1e3],
            "model__gamma": ["scale", "auto", 1e-3, 1e-2, 1e-1, 1.0, 10.0],
            "model__epsilon": [1e-4, 5e-4, 1e-3, 5e-3, 0.01, 0.05, 0.1, 0.2, 0.3],
        },
        "SVR(Linear)": {
            "model__C": [1e-3, 1e-2, 1e-1, 1.0, 10.0, 100.0, 1e3],
            "model__epsilon": [1e-4, 5e-4, 1e-3, 5e-3, 0.01, 0.05, 0.1, 0.2, 0.3],
        },
        "KNN": {
            "model__n_neighbors": [1, 3, 5, 7, 9, 11, 15, 21, 31, 45, 61],
            "model__weights": ["uniform", "distance"],
            "model__p": [1, 2, 3],
        },
        "MLP": {
            "model__hidden_layer_sizes": [(16,), (32,), (64,), (32, 16), (64, 32)],
            "model__alpha": [1e-5, 1e-4, 1e-3, 1e-2],
            "model__learning_rate_init": [1e-4, 1e-3, 1e-2],
        },
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


# --------------------------------------------------
# Main
# --------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="Path to goal_mode.txt")
    ap.add_argument("--kc", type=float, default=1.0)
    ap.add_argument("--kt", type=float, default=10.0)
    ap.add_argument("--jobs", type=int, default=1)
    args = ap.parse_args()

    df = load_goal_mode(args.path)

    required = ["CLR", "NAR", "TRN", "EFF", "PLN", "EXE", "RMS", "NRC"]
    for c in required:
        if c not in df.columns:
            raise RuntimeError(f"Missing column {c}")

    # Inputs (planner metrics)
    X = df[["CLR", "TRN", "EFF"]].to_numpy(float)

    # Target (computed success)
    y = compute_success_y(df, kc=args.kc, kt=args.kt)

    # Train / test split
    Xtr, Xte, ytr, yte = train_test_split(
        X, y, test_size=0.2, shuffle=True, random_state=42
    )

    models = get_models()
    grids = get_param_grids()
    results: dict[str, tuple[object, dict[str, object], float, float]] = {}

    print("\n=== Grid search (CV on train), then test eval ===")
    print(f"kc={args.kc}, kt={args.kt}\n")

    for name, model in models.items():
        grid = grids.get(name, {})
        search = GridSearchCV(
            model,
            grid,
            scoring="neg_mean_squared_error",
            cv=5,
            n_jobs=1,
            refit=True,
        )
        search.fit(Xtr, ytr)

        best_model = search.best_estimator_
        yhat = np.clip(best_model.predict(Xte), 0.0, 1.0)

        mse = mean_squared_error(yte, yhat)
        r2 = r2_score(yte, yhat)

        results[name] = (best_model, search.best_params_, mse, r2)
        print(
            f"{name:18s} | MSE = {mse:.6e} | R² = {r2:.4f} | best={search.best_params_}"
        )

    best_name = min(results, key=lambda k: results[k][2])
    best_model = results[best_name][0]
    best_mse = results[best_name][2]
    best_r2 = results[best_name][3]
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
            base_features = ["CLR", "TRN", "EFF"]
            poly = best_model.named_steps.get("poly")
            if poly is not None:
                feature_names = poly.get_feature_names_out(base_features)
            else:
                feature_names = base_features

            intercept = getattr(model_step, "intercept_", 0.0)
            terms = [f"{intercept:.6g}"]
            for name, coef in zip(feature_names, coefs):
                terms.append(f"{coef:.6g}*{name}")
            formula = "H = " + " + ".join(terms)
            print("Best linear model formula:")
            print(formula)


if __name__ == "__main__":
    main()
