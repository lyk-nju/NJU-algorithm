# AGENTS.md
For coding agents working in `/home/nvidia/NJU-algorithm`.

## Scope
- This repo is a collection of ROS 2 and CMake projects, not one unified workspace.
- Main areas:
- `decision_task/` - ROS 2 package `decision` with `ament_cmake`
- `navigation_task/` - ROS 2 workspace with many packages under `src/`
- `vision_task/hiki_ros2/` - ROS 2 package `hik_camera`
- `vision_task/armor_task/` - plain CMake/CUDA project with executable test targets
- `robot_bringup/` - Python ROS 2 package

## Editor / Agent Rules
- No top-level `AGENTS.md` existed when this file was created.
- No `.cursorrules` file was found.
- No `.cursor/rules/` directory was found.
- No `.github/copilot-instructions.md` file was found.
- Do not assume additional editor-specific policy beyond repo files and this document.

## Environment
- ROS 2 Humble is assumed by docs and package layout.
- Common setup:
```bash
source /opt/ros/humble/setup.bash
```
- After ROS builds, source the local overlay before running nodes/tests:
```bash
source install/setup.bash
```
- `decision_task` docs also export:
```bash
export AMENT_PREFIX_PATH=/home/nvidia/NJU-algorithm/decision_task/install/decision:$AMENT_PREFIX_PATH
```

## Build Commands
### decision_task
- Full build:
```bash
cd /home/nvidia/NJU-algorithm/decision_task
colcon build
```
- Single package:
```bash
cd /home/nvidia/NJU-algorithm/decision_task
colcon build --packages-select decision
```
### navigation_task
- Common focused build from project docs:
```bash
cd /home/nvidia/NJU-algorithm/navigation_task
colcon build --packages-select fast_lio rm_nav_bringup rm_navigation
```
- Single package:
```bash
cd /home/nvidia/NJU-algorithm/navigation_task
colcon build --packages-select rm_navigation
```
- Full workspace:
```bash
cd /home/nvidia/NJU-algorithm/navigation_task
colcon build --symlink-install
```

### vision_task/hiki_ros2
- Build camera package:
```bash
cd /home/nvidia/NJU-algorithm/vision_task/hiki_ros2
colcon build --packages-select hik_camera
```

### robot_bringup
- Build package:
```bash
cd /home/nvidia/NJU-algorithm/robot_bringup
colcon build --packages-select robot_bringup
```

### vision_task/armor_task
- This is plain CMake, not `colcon`.
- Full build:
```bash
cd /home/nvidia/NJU-algorithm/vision_task/armor_task
mkdir -p build
cd build
cmake ..
make -j"$(nproc)"
```
- Build one executable target:
```bash
cd /home/nvidia/NJU-algorithm/vision_task/armor_task
cmake --build build --target auto_aimer_test -j"$(nproc)"
```

## Test Commands
- Generic ROS package test flow:
```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```
- Many navigation packages only wire up `ament_lint_auto` under `BUILD_TESTING`.
- `decision_task` has buildable nodes but no obvious committed unit-test target.
- `robot_bringup` declares `pytest` in `setup.py`, but no dedicated test files were found.

### Single ROS package / single test
- Known explicit gtest target: `test_costmap_polygons` in `navigation_task/src/rm_navigation/costmap_converter/costmap_converter/`.
- Preferred filtered invocation:
```bash
cd /home/nvidia/NJU-algorithm/navigation_task
colcon test --packages-select costmap_converter --ctest-args -R test_costmap_polygons --output-on-failure
colcon test-result --verbose
```
- If needed, inspect `navigation_task/build/costmap_converter/` for the built test binary.

### armor_task executable tests
- `vision_task/armor_task` uses runnable executables instead of `ctest`.
- Known targets: `video_test`, `camera_sub_test`, `detector_test`, `pnp_solver_test`, `target_test`, `auto_aimer_test`, `communication_test`, `fake_publish`.
- Run one executable:
```bash
cd /home/nvidia/NJU-algorithm/vision_task/armor_task/build
./auto_aimer_test
```
- Rebuild a single target:
```bash
cd /home/nvidia/NJU-algorithm/vision_task/armor_task
cmake --build build --target detector_test -j"$(nproc)"
```

## Lint Commands
- There is no single repo-wide lint wrapper.
- For packages with `ament_lint_auto`, use normal test flow:
```bash
cd /home/nvidia/NJU-algorithm/navigation_task
colcon test --packages-select rm_navigation
colcon test-result --verbose
```
- Packages with explicit `ament_lint_auto` usage include `rm_navigation`, `rm_nav_bringup`, `fake_vel_transform`, `icp_registration`, `pointcloud_to_laserscan`, `pb_rm_simulation`, and `livox_ros_driver2/src`.
- `vision_task/hiki_ros2/` lists lint test dependencies in `package.xml`, but its `CMakeLists.txt` does not currently add a `BUILD_TESTING` lint section.

## Code Style: General Rule
- The repo is style-inconsistent across subprojects. Match the style of the file you touch.
- Avoid formatting-only rewrites unless the task is specifically about formatting.
- Prefer the smallest practical diff.

## Code Style: C++ Standards
- `decision_task` and `vision_task/armor_task` use C++17.
- Many navigation packages still use C++14.
- Do not raise a package standard just to add a small feature.
- Respect local `CMakeLists.txt` compile flags and dependency declarations.

## Code Style: Formatting
- Use 4-space indentation.
- Avoid tabs unless the file already uses them.
- Brace style is package-local: `decision_task` and much of `vision_task` lean Allman-style; `vision_task/hiki_ros2` follows its local `.clang-format` and Google-derived style.
- Column width is not uniform: `vision_task/.clang-format` sets `ColumnLimit: 220`; `vision_task/hiki_ros2/.clang-format` sets `ColumnLimit: 100`.
- When editing under `vision_task/`, check the nearest `.clang-format` first.

## Code Style: Includes / Imports
- Keep include groups stable and local to the file.
- Typical C++ grouping in this repo: related project header first for implementation files, then standard library, third-party, ROS, and project-local headers.
- This ordering is not perfectly uniform; prefer consistency with the surrounding file over normalization.
- Python launch files usually import stdlib first, then ROS/launch modules.

## Code Style: Naming
- Follow local naming, not one repo-wide rule.
- Common newer-code patterns: classes/enums `CamelCase`, functions/methods `camelCase` or `camelBack`, locals `snake_case` or simple lower-case names, private members trailing `_`, true constants `UPPER_CASE`.
- `vision_task/hiki_ros2/.clang-tidy` explicitly prefers `CamelCase` classes, `camelBack` methods, `lower_case` variables, `_` suffixed members, and `UPPER_CASE` constants.
- Older `decision_task` code mixes styles such as `wayPointMap` and `_distance_THR`; do not rename existing symbols unless required.

## Code Style: Types and APIs
- Prefer explicit types at public APIs, ROS interfaces, and parameter boundaries.
- Use `auto` when the type is obvious or very verbose.
- Keep ROS message, action, and parameter types explicit.
- Reuse local aliases and typedefs where they already exist.

## Code Style: Error Handling
- For ROS runtime issues, prefer `RCLCPP_INFO/WARN/ERROR/FATAL` logging.
- For recoverable config or parse failures, prefer logging and safe fallback behavior when that matches existing code.
- Use `message(FATAL_ERROR ...)` in CMake only for hard dependency/build failures.

## Code Style: Paths, Config, and Launch
- Prefer ROS parameters declared with `declare_parameter` for node configuration.
- Prefer package-share lookup helpers like `get_package_share_directory` over adding new hard-coded absolute paths.
- Be careful: some existing launch files already hard-code absolute paths; preserve behavior unless cleanup is part of the task.
- JSON and YAML config files are runtime-critical; keep schema compatibility when editing them.

## Code Style: Headers and Comments
- Existing headers usually use include guards instead of `#pragma once`; match the local file.
- Add comments only for non-obvious logic, ROS wiring, hardware quirks, or tricky math.
- Brief Chinese comments already exist in several packages; bilingual comments are acceptable if they match the file.

## Working Advice
- Build and test only the affected workspace when possible.
- For navigation changes, prefer `--packages-select` rather than rebuilding the whole workspace.
- For `armor_task`, rebuild only the touched executable target.
- Do not silently refactor legacy naming, path handling, or formatting outside the requested scope.
