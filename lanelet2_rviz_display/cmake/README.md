# Package-scoped clang-tidy runner

`project_clang_tidy.cmake` registers a package-scoped clang-tidy test (`run_project_clang_tidy.py`)
that uses the shared `.clang-tidy` rules but only fails on diagnostics from **package-owned**
source files, dropping third-party macro-expansion noise.

This package needs it because it is a Qt/rviz plugin: the build compiles generated `moc_*.cpp`
translation units, which stock `ament_clang_tidy` lints and floods with `cppcoreguidelines-*`
findings it cannot exclude. The other packages in this repo instead use the
`target_dependencies_as_system()` helper exported by
[`lanelet2_map_interface`](../../lanelet2_map_interface) plus the stock `ament_cmake_clang_tidy`
test — but that helper only re-marks *dependency* include directories as `SYSTEM` and cannot
silence findings inside generated project translation units, so Qt packages keep this runner.

Wired up in `CMakeLists.txt` after excluding the default `ament_cmake_clang_tidy` test from
`ament_lint_auto`.
