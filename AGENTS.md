# Repository Guidelines

## Project Structure & Module Organization

Primary development happens in `ros2_ws/`, a ROS 2 workspace. Source packages live in `ros2_ws/src/` and include in-house packages such as `cobot_gz`, `cobot_launch`, `controller`, `slam`, `teleop`, `space_cobot_controller`, and `space_cobot_interface`, plus third-party/vendor packages (`mavros`, `mavlink`, `aruco_opencv*`).

Package layouts follow ROS 2 conventions:
- C++ nodes/libraries: `src/`, `include/`, `launch/`, `msg/`
- Python packages: `<pkg>/<pkg>/`, `launch/`, `config/`, `test/`

Supporting assets are in `docker/` (containerized ROS 2 Humble environment) and `misc/path_eval_model/` (standalone Python model utilities).

## Build, Test, and Development Commands

From repo root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

- `colcon build --symlink-install`: builds all ROS 2 packages for local development.
- `colcon test`: runs package tests (ament + pytest-based checks).
- `colcon test-result --verbose`: prints failing test details.
- `ros2 launch cobot_gz cobot_gz.launch.py`: starts Gazebo simulation (example).

Docker workflow (from `docker/`):

```bash
docker compose up -d --build
docker exec -it space_cobot_container zsh
```

## Coding Style & Naming Conventions

- Use 4-space indentation in Python and C++.
- Follow ROS 2 package naming (`snake_case` package names, launch files ending in `.launch.py`).
- Python modules/functions: `snake_case`; classes: `PascalCase`.
- C++ headers use package-scoped include paths (example: `controller_master/controller_master_handle.hpp`).
- Respect ament linters where configured (`ament_flake8`, `ament_pep257`, `ament_lint_auto`).

## Testing Guidelines

- Python package tests are in `ros2_ws/src/<pkg>/test/` and commonly include `test_flake8.py`, `test_pep257.py`, and `test_copyright.py`.
- Run targeted tests with `colcon test --packages-select slam teleop`.
- Add tests alongside the package you change; keep test filenames prefixed with `test_`.

## Commit & Pull Request Guidelines

Recent history uses short, lowercase commit subjects (for example, `minor updates`, `fixed robot radisu problem`). Prefer concise imperative messages with a clear scope, such as `slam: fix pose frame publishing`.

PRs should include:
- What changed and why
- Affected packages (for example, `controller`, `cobot_gz`)
- How it was tested (`colcon build`, `colcon test`, launch command)
- Screenshots/log snippets for simulation or visualization changes
