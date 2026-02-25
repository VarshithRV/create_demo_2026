# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_create_demo_2026_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED create_demo_2026_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(create_demo_2026_FOUND FALSE)
  elseif(NOT create_demo_2026_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(create_demo_2026_FOUND FALSE)
  endif()
  return()
endif()
set(_create_demo_2026_CONFIG_INCLUDED TRUE)

# output package information
if(NOT create_demo_2026_FIND_QUIETLY)
  message(STATUS "Found create_demo_2026: 0.0.0 (${create_demo_2026_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'create_demo_2026' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${create_demo_2026_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(create_demo_2026_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${create_demo_2026_DIR}/${_extra}")
endforeach()
