# AGENTS

## Fast-start commands (Docker-first workflow)
- This repo is expected to run inside Docker; run build/launch/test commands in container (workspace path is typically `/ros_ws`).
- The README and current `docker-compose.yaml` use the default dev container name `rv_devel_`; `docker exec -it rv_devel_ bash` is the expected shell entrypoint.
- If container shell does not auto-source ROS env, run `source /opt/ros/humble/setup.bash` first.
- Install deps in workspace root: `rosdep install --from-paths src --ignore-src -r -y` then `sudo apt install ros-humble-asio-cmake-module` (`README.md` notes this one is not auto-installed).
- Build whole workspace: `colcon build --symlink-install --parallel-workers 4` (thread cap is intentional for this repo).
- Build one package: `colcon build --symlink-install --packages-select <pkg>`.
- Always source before running ROS nodes: `source install/setup.bash`.
- Main runtime launch in this repo is `ros2 launch rm_bringup bringup_SingleProcess.launch.py` (there is no `bringup.launch.py` in `src/rm_bringup/launch/`).
- `docker-compose.yaml` already points at `Dockerfile` with the correct case. It also binds `${HOME}/rmvision2025` to `/ros_ws`, so adjust that host path if your checkout lives elsewhere.

## Real package boundaries
- `src/rm_auto_aim/rm_auto_aim` and `src/rm_rune/rm_rune` are meta-packages; actual nodes live in leaf packages (`armor_detector`, `armor_solver`, `rune_detector`, `rune_solver`).
- `src/rm_bringup/launch/bringup_SingleProcess.launch.py` is the integration entrypoint: it composes camera/serial/aim/rune/replay/record components into one `component_container_mt`.
- Message/service definitions are centralized in `src/rm_interfaces`; build this first when interface changes are involved.

## Launch/config gotchas that are easy to miss
- Toggle behavior is controlled by `src/rm_bringup/config/launch_params.yaml`; most runtime changes should happen there first.
- In launch logic, `replay: true` forcibly disables state-machine camera, video player, and virtual/physical serial inputs to avoid source conflicts.
- If both `record` and `auto_record` are true, launch code disables `record` at runtime.
- `rune: false` changes serial protocol handling (serial params get `protocol: hero`).

## Build/test expectations
- Many CMake targets compile with `-Wall -Werror` (warnings fail builds); keep changes warning-clean.
- Tests are package-local and sparse; verified gtests exist in:
  - `armor_detector` (`test/test_detector.cpp`)
  - `rune_detector` (`test/test_detector.cpp`, `test/test_node_startup.cpp`)
  - `rune_solver` (`test/test_node_startup.cpp`)
  - `rm_serial_driver` (`test/test_fixed_packet_tool.cpp`)
- Run focused tests with `colcon test --packages-select <pkg>` and inspect with `colcon test-result --verbose`.

## Code style conventions
- Follow the repo's `.clang-format` and `.clang-tidy` as the source of truth; do not introduce a local style that conflicts with them.
- Formatting highlights from `.clang-format`: 4-space indentation, no tabs, 100-column limit, left-aligned pointers/references, one include per line, and sorted includes/using declarations.
- Naming highlights from `.clang-tidy`: namespaces/functions/variables/class members use `lower_case`; classes/structs/enums use `CamelCase`; enum constants/macros/global constants use `UPPER_CASE`.
- Prefer warning-free modern C++ that passes the enabled tidy checks (`modernize-*`, `bugprone-*`, `readability-*`); avoid unnecessary raw `new/delete`, implicit bool conversions, and missing `override`.
- Keep ownership, units, and coordinate-frame semantics explicit in names and code paths, especially in solver, TF, and serial-related code.
- Use Chinese for newly added code comments unless the surrounding file already follows a different established convention.
- Do not reformat unrelated code while making a change. Keep formatting edits scoped to the touched logic unless a dedicated formatting cleanup is explicitly requested.
- Do not add commented-out code, broad `NOLINT` suppressions, or formatting-only churn outside the files needed for the task.

## Doxygen function comment conventions
- Use Doxygen `/** ... */` for public APIs, ROS callbacks/services/actions, non-trivial helpers, and functions whose behavior is not obvious from the signature alone.
- Prefer writing the canonical function comment on the declaration in headers. In `.cpp` files, document free functions or local helpers at the definition site when there is no documented declaration.
- Start with a single-sentence `@brief` describing the function's responsibility, not its implementation steps.
- Add one `@param` for every non-obvious parameter. Describe meaning, units, coordinate frame, ownership, valid range, and input/output semantics when relevant.
- Add `@return` for every non-`void` function. State the semantic meaning of the return value, not just its C++ type.
- Add `@note` or `@warning` when behavior depends on threading, timing, blocking I/O, parameter side effects, frame conventions, or preconditions that callers must satisfy.
- Keep comments concise and synchronized with code. Do not leave empty tags such as bare `@param foo` or `@return std::vector<T>`.
- Recommended template:
  ```cpp
  /**
   * @brief Compute the predicted armor pose at the fire time.
   * @param target Current target state in odom frame.
   * @param dt Prediction horizon in seconds.
   * @return Predicted armor pose in odom frame.
   * @note Caller must ensure `dt >= 0`.
   */
  ```

## Toolchain/dependency quirks
- OpenVINO is a hard dependency for detector packages (`armor_detector`, `rune_detector`) via `find_package(OpenVINO ...)`.
- `hik_camera` links Hik SDK from repo-local path `src/rm_utils/hikSDK` (arch-specific libs under `lib/amd64` or `lib/arm64`).

## Ignore generated/runtime artifacts
- Do not edit/commit runtime/build outputs under `build/`, `install/`, `log/`, `qd2026-log/`, `MvSdkLog/`, `.cache/` (already ignored in `.gitignore`).
