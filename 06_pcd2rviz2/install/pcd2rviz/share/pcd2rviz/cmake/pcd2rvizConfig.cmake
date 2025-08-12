# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pcd2rviz_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pcd2rviz_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pcd2rviz_FOUND FALSE)
  elseif(NOT pcd2rviz_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pcd2rviz_FOUND FALSE)
  endif()
  return()
endif()
set(_pcd2rviz_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pcd2rviz_FIND_QUIETLY)
  message(STATUS "Found pcd2rviz: 0.0.0 (${pcd2rviz_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pcd2rviz' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pcd2rviz_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pcd2rviz_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pcd2rviz_DIR}/${_extra}")
endforeach()
