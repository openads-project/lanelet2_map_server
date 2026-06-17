cmake_minimum_required(VERSION 3.5)

project(lanelet2_rviz_display)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake REQUIRED)

find_package(pluginlib REQUIRED)

find_package(rclcpp REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(lanelet2_map_interface REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_core REQUIRED)


find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

set(${PROJECT_NAME}_headers_to_moc
  include/${PROJECT_NAME}/lanelet2_display.hpp
  include/${PROJECT_NAME}/lanelet2_map.hpp
)

foreach(header "${${PROJECT_NAME}_headers_to_moc}")
  qt5_wrap_cpp(${PROJECT_NAME}_moc_files "${header}")
endforeach()

set(${PROJECT_NAME}_source_files
  src/lanelet2_display.cpp
  src/lanelet2_map.cpp
  src/lanelet2_map_helpers.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_moc_files}
  ${${PROJECT_NAME}_source_files}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "LANELET2_RVIZ_DISPLAY_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  lanelet2_map_interface
  lanelet2_io
  lanelet2_projection
  lanelet2_core
  rviz_common
  rviz_rendering
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_ogre_vendor
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES
    LICENSE.BSD-3-Clause
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/format/.clang-format)
  set(ament_cmake_clang_tidy_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/.clang-tidy)
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../.vscode/lint/ament_flake8.ini)
  ament_lint_auto_find_test_dependencies()

  # Qt packages compile generated moc_*.cpp translation units, which stock
  # ament_clang_tidy lints and floods with cppcoreguidelines findings it cannot
  # exclude. Use the project-scoped clang-tidy runner instead. 
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_clang_tidy)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/project_clang_tidy.cmake)
  add_project_clang_tidy_test(
    CONFIG_FILE ${ament_cmake_clang_tidy_CONFIG_FILE}
    EXTRA_ARGS "-U_OPENMP"
  )
endif()

ament_package(
  CONFIG_EXTRAS "${PROJECT_NAME}-extras.cmake"
)
