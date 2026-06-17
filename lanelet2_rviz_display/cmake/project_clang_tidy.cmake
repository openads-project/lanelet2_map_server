# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

function(add_project_clang_tidy_test)
  cmake_parse_arguments(ARG "" "CONFIG_FILE;TIMEOUT" "EXTRA_ARGS" ${ARGN})

  if(NOT ARG_CONFIG_FILE)
    message(FATAL_ERROR "add_project_clang_tidy_test() requires CONFIG_FILE")
  endif()
  if(NOT ARG_TIMEOUT)
    set(ARG_TIMEOUT 300)
  endif()

  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  set(result_file "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/clang_tidy.xunit.xml")
  set(cmd
    "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/run_project_clang_tidy.py"
    "--package-name" "${PROJECT_NAME}"
    "--build-dir" "${CMAKE_CURRENT_BINARY_DIR}"
    "--source-dir" "${CMAKE_CURRENT_SOURCE_DIR}"
    "--config-file" "${ARG_CONFIG_FILE}"
    "--xunit-file" "${result_file}"
  )
  foreach(extra_arg IN LISTS ARG_EXTRA_ARGS)
    list(APPEND cmd "--extra-arg=${extra_arg}")
  endforeach()

  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ament_clang_tidy")
  ament_add_test(
    clang_tidy
    COMMAND ${cmd}
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/ament_clang_tidy/clang_tidy.txt"
    RESULT_FILE "${result_file}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT "${ARG_TIMEOUT}"
  )
  set_tests_properties(clang_tidy PROPERTIES LABELS "clang_tidy;linter")
endfunction()
