cmake_minimum_required(VERSION 2.6)

set(LIB_NAME tviewer)
project("lib${LIB_NAME}")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

option(TVIEWER_BUILD_DOC "Build tviewer documentation." OFF)

# Compilation
add_definitions(-std=c++11)
add_definitions(-Wall -pedantic)
# Boost Preprocessor library is very conservative in enabling variadics support.
# In fact, if a compiler implements c++11, then we may be sure that variadicts
# are supported as well. This define enables preprocessor macros that require
# variadics support (needed by 'named_parameters_def.h').
add_definitions(-DBOOST_PP_VARIADICS=1)

# Dependencies
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# TViewer library
add_library(${LIB_NAME}
  tviewer/color.cpp
  tviewer/utils.cpp
  tviewer/tviewer.cpp
  tviewer/factory.cpp
  tviewer/poly_data_object.cpp
  tviewer/point_cloud_object.cpp
  tviewer/normal_cloud_object.cpp
  tviewer/visualization_object.cpp
  tviewer/point_cloud_with_color_shuffling_object.cpp
  tviewer/keyboard_listeners/up_down_counter.cpp
)

add_subdirectory(examples)

# Export variables
set(TVIEWER_INCLUDE_DIRS ${libtviewer_SOURCE_DIR} CACHE STRING "TViewer include directories")
set(TVIEWER_LIBRARIES ${LIB_NAME} CACHE STRING "TViewer libraries")
set(TVIEWER_DEFINITIONS "-std=c++11 -DBOOST_PP_VARIADICS=1" CACHE STRING "TViewer compiler definitions")

# Documentation
if (TVIEWER_BUILD_DOC)
  set(DOXYFILE_LATEX false)
  include(UseDoxygen)
endif()

