# Path Evaluation Metrics and Predictor Models

This document defines the dataset columns, target `y`, and predictor models used by `path_eval_model.py`.

## 1. Dataset Columns

Current row layout (collection):

`CLR NAR TRN EFF MCLR NFR MTR PLN STD DTR EXE RMS NRC MEV`

### Normalized planner metrics (inputs)

- `CLR` (clearance score):  
  `CLR = clip(min_clearance_m / (alpha * robot_radius), 0, 1)`
- `NAR` (narrow score):  
  `NAR = 1 - narrow_fraction`
- `TRN` (turn score):  
  `TRN = 1 - clip(max_turn_rad / pi, 0, 1)`
- `EFF` (efficiency score):  
  `EFF = clip(straight_dist_m / path_length_m, 0, 1)`

### Raw planner metrics (inputs)

- `MCLR`: minimum clearance in meters (`min_clearance_m`)
- `NFR`: fraction of sampled path points considered narrow (`narrow_fraction`)
- `MTR`: maximum turn angle in radians (`max_turn_rad`)
- `PLN`: planned path length in meters (`path_length_m`)
- `STD`: start-to-goal straight-line distance in meters (`straight_dist_m`)
- `DTR`: detour ratio (`path_length_m / straight_dist_m`)

### Execution metrics (target components / logging)

- `EXE`: executed path length in meters (`executed_length`)
- `RMS`: RMS tracking error (`rms_tracking_error`)
- `NRC`: number of collisions
- `MEV`: reserved/manual extra metric (currently optional)

Note: in current logger flow, `NRC` is intentionally printed as `0` during data collection, so it can be manually annotated later.

## 2. Target Quality Score `y`

Training target:

`y = clip((EXE / PLN) * exp(-kc * NRC) * exp(-kt * RMS), 0, 1)`

Default values in the current trainer:

- `kc = 1.0`
- `kt = 5.0`

Interpretation:

- `EXE/PLN` rewards completion.
- `exp(-kc*NRC)` penalizes collisions.
- `exp(-kt*RMS)` penalizes tracking error.

## 3. Predictor `H`

The model learns:

`H = f(X)`

where `X` is built from available input columns:

- normalized set: `CLR, NAR, TRN, EFF`
- raw set: `MCLR, NFR, MTR, PLN, STD, DTR`

If a linear model is best, this is reported as:

`H = b0 + b1*x1 + ... + bn*xn`

and predictions are clipped to `[0, 1]` during evaluation.

## 4. Model Set (Current)

### Linear / robust

- `Ridge`
- `ElasticNet`
- `Lasso`
- `Huber`

### Nonlinear

- `PolyRidge(d2)` (polynomial features + ridge)
- `SVR(RBF)`
- `SVR(Linear)`
- `KNN`
- `MLP`
- `HistGBR` (HistGradientBoostingRegressor)

Grid search is run per model, selecting best by test MSE.

## 5. Practical Notes

- If `DTR` is missing but `PLN` and `STD` exist, trainer computes `DTR = PLN / STD`.
- Minimum required columns for training target are: `PLN, EXE, RMS, NRC`.
- More stable results usually come from balanced data collection (easy/medium/hard path cases).
