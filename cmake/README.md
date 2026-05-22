# CMake helpers

This directory contains local test helpers shared by the packages in this workspace.

`project_clang_tidy.cmake` registers a package-scoped clang-tidy test that still uses the shared `.clang-tidy` rules from `openads-dev-environment`, but only fails on diagnostics from package-owned source files. This keeps third-party, generated, ROS, Qt, and system-header noise out of package lint results without weakening the shared lint configuration.

Use it from a package `CMakeLists.txt` after excluding the default `ament_cmake_clang_tidy` test from `ament_lint_auto`.
