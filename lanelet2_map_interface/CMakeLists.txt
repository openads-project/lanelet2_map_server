cmake_minimum_required(VERSION 3.8)
project(lanelet2_map_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC" )

find_package(ament_cmake REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(rclcpp REQUIRED)

set(TARGET_NAME lanelet2_map_interface)

add_library(${TARGET_NAME}
  src/lanelet2_map_interface.cpp
)

target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${TARGET_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${TARGET_NAME}
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  rclcpp
)

ament_export_targets(${TARGET_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(lanelet2_core)
ament_export_dependencies(lanelet2_io)
ament_export_dependencies(lanelet2_projection)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${TARGET_NAME}
  EXPORT ${TARGET_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()

  # Use a custom project-scoped clang-tidy wrapper to avoid linting failures induced by third-party code, e.g., Qt.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_tidy)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/project_clang_tidy.cmake)
  add_project_clang_tidy_test(
    CONFIG_FILE ${ament_cmake_clang_tidy_CONFIG_FILE}
    EXTRA_ARGS "-U_OPENMP"
  )
endif()

ament_package()
