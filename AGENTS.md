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

### Docker workspace ownership
- `docker exec` normally runs as root. Do not use it to edit or format files under `/ros_ws/src`, otherwise source files can become root-owned and later host-side `apply_patch` edits will fail.
- Prefer host-side `apply_patch` for source edits. When a container command must rewrite a source file (for example `clang-format -i`), run it as the host UID/GID: `docker exec --user "$(id -u):$(id -g)" rv_devel_ bash -lc 'cd /ros_ws && ...'`.
- Build output ownership under `build/`, `install/`, and `log/` is unimportant because those directories are generated and ignored; source ownership is not. If a task-created source file is accidentally root-owned, repair only the exact known paths with `docker exec rv_devel_ chown 1000:1000 /ros_ws/<exact-path>`; never recursively `chown` the workspace.
- Before editing an unexpectedly unwritable source file, inspect ownership with `ls -l <path>` rather than assuming a compiler or patch failure is a code issue.

## Real package boundaries
- `src/rm_auto_aim/rm_auto_aim` and `src/rm_rune/rm_rune` are meta-packages; actual nodes live in leaf packages (`armor_detector`, `armor_solver`, `rune_detector`, `rune_solver`).
- `src/rm_bringup/launch/bringup_SingleProcess.launch.py` is the integration entrypoint: it composes camera/serial/aim/rune/replay/record components into one `component_container_mt`.
- Message/service definitions are centralized in `src/rm_interfaces`; build this first when interface changes are involved.

## Launch/config gotchas that are easy to miss
- Toggle behavior is controlled by `src/rm_bringup/config/launch_params.yaml`; most runtime changes should happen there first.
- In launch logic, `replay: true` forcibly disables state-machine camera, video player, and virtual/physical serial inputs to avoid source conflicts.
- If both `record` and `auto_record` are true, launch code disables `record` at runtime.
- `rune: false` changes serial protocol handling (serial params get `protocol: hero`).

### Standalone serial and 0-order gimbal test
- `ros2 launch rm_bringup zero_order_gimbal_test.launch.py` is deliberately a small hardware test launch: serial driver + zero-order command publisher + Foxglove, without camera, armor, or rune nodes. It is not a replacement for `bringup_SingleProcess.launch.py`.
- The serial driver normally maps received MCU mode to visual-node `*/set_mode` services. This behavior is controlled by `enable_mode_sync`, whose default is `true` so the main bringup keeps its existing behavior.
- Any launch that starts `SerialDriverNode` without armor/rune service providers must pass `enable_mode_sync: false`. Otherwise the first valid RX packet can block the serial receive thread in `wait_for_service()`, causing `/serial/receive` and terminal RX output to stop. Setting only `has_rune: false` skips rune services but still leaves armor services waiting.
- The dedicated zero-order launch sets both `has_rune: false` and `enable_mode_sync: false`. Its expected startup log includes `Vision mode service synchronization is disabled`.
- A process started before rebuilding keeps its old parameters and executable. Stop it with `Ctrl-C`, source `install/setup.bash`, then relaunch before judging a parameter or code change.
- Only one process may own `/dev/rm_usb0`. Do not run the standalone zero-order launch and the main bringup together. Also stop stale `zero_order_gimbal_test_node` processes before hardware tests, since multiple publishers to `armor_solver/cmd_gimbal` create conflicting commands.
- `ros2 param dump /serial_driver` verifies the active node's effective parameters; `ros2 launch ... --show-args` only verifies that launch-file parsing succeeds and does not verify serial I/O.

## Build/test expectations
- Many CMake targets compile with `-Wall -Werror` (warnings fail builds); keep changes warning-clean.
- Tests are package-local and sparse; verified gtests exist in:
  - `armor_detector` (`test/test_detector.cpp`)
  - `rune_detector` (`test/test_detector.cpp`, `test/test_node_startup.cpp`)
  - `rune_solver` (`test/test_node_startup.cpp`)
  - `rm_serial_driver` (`test/test_fixed_packet_tool.cpp`)
- Run focused tests with `colcon test --packages-select <pkg>` and inspect with `colcon test-result --verbose`.

### Build and verification lessons
- A first test-first build may fail because the asserted interface is intentionally not implemented yet. Record that expected failure, then distinguish it from later regressions.
- `ament_auto_add_library(DIRECTORY src)` discovers sources when CMake configures. After adding or deleting a `.cpp` covered by this rule, make the next package build run with `--cmake-force-configure` so the target source list is refreshed.
- If a full `colcon test` is blocked by pre-existing lint or formatting baseline failures, run and report focused tests plus formatting checks for touched files separately. Do not describe the full package suite as passing.
- The Docker development container may not include `rg`; use `grep` for in-container filtering rather than treating the missing command as a build or test failure.
- Execute CTest from the sourced workspace root, not by manually entering `build/<package>`: `cd /ros_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ctest --test-dir build/<package> -R '<test-regex>' --output-on-failure`. The latter can lose the workspace Python path and cause `ModuleNotFoundError: ament_cmake_test`, which is an environment setup error rather than a failed gtest assertion.
- Use `colcon test --packages-select <pkg> --return-code-on-test-failure` for package-level execution, then inspect only the intended result files or run focused CTest when the package has known lint baselines. In this repository, `rm_serial_driver` currently has many pre-existing `clang_format` failures; distinguish those from the gtest result in reports.
- Build success does not validate a ROS launch's Python syntax. After adding or changing a launch file, run `ros2 launch <package> <file>.launch.py --show-args` from a sourced workspace as a parse-level check; hardware and topic-flow checks still require an actual launch.
- `SerialDriverNode` owns a receive thread. The UART read path must have a finite timeout (currently `poll(..., 100 ms)`) so the thread can observe `rclcpp::ok() == false` during `SIGINT`; never replace it with an unbounded `read()` (`VMIN=1`) without an explicit shutdown wake-up mechanism, or a composable-node container can exceed launch's 5-second SIGINT timeout and be force-killed with `SIGTERM`.
- A UART timeout (`read() == 0`) means "no byte arrived yet", not a transport failure. Protocol receive loops should continue waiting on zero, reconnect only on negative reads / poll errors, and the serial node should suppress receive-failure warnings once ROS shutdown has started.

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
