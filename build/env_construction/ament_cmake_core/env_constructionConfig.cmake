# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_env_construction_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED env_construction_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(env_construction_FOUND FALSE)
  elseif(NOT env_construction_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(env_construction_FOUND FALSE)
  endif()
  return()
endif()
set(_env_construction_CONFIG_INCLUDED TRUE)

# output package information
if(NOT env_construction_FIND_QUIETLY)
  message(STATUS "Found env_construction: 0.0.0 (${env_construction_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'env_construction' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${env_construction_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(env_construction_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${env_construction_DIR}/${_extra}")
endforeach()
