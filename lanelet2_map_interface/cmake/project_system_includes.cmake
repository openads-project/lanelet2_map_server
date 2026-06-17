# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0
#
# target_dependencies_as_system(<target>)
#
# Marks every include directory pulled in via the target's (transitive) link
# dependencies as a SYSTEM include directory. The compiler and clang-tidy then
# treat third-party headers -- and diagnostics arising from their macros -- as
# system headers and skip linting them. The target's own include directories
# are left untouched, so project code is still fully linted.
#
# This is needed because some dependencies (e.g. lanelet2 via mrt_cmake_modules) export
# their interface includes as non-SYSTEM, which downgrades
# the whole include set to plain -I and surfaces third-party warnings.
#
# This file is installed as a CONFIG_EXTRA, so any package that
# find_package(lanelet2_map_interface) gets this function for free.
include_guard(GLOBAL)

function(target_dependencies_as_system target)
  set(_seen "")
  set(_pending "")
  get_target_property(_libs ${target} LINK_LIBRARIES)
  if(_libs)
    list(APPEND _pending ${_libs})
  endif()

  while(_pending)
    list(POP_FRONT _pending _dep)
    if(NOT TARGET ${_dep})
      continue()
    endif()
    if("${_dep}" IN_LIST _seen)
      continue()
    endif()
    list(APPEND _seen "${_dep}")

    get_target_property(_inc ${_dep} INTERFACE_INCLUDE_DIRECTORIES)
    if(_inc)
      set_target_properties(${_dep} PROPERTIES
        INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_inc}")
    endif()

    get_target_property(_tlibs ${_dep} INTERFACE_LINK_LIBRARIES)
    if(_tlibs)
      list(APPEND _pending ${_tlibs})
    endif()
  endwhile()
endfunction()
