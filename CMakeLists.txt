cmake_minimum_required(VERSION 2.6)

set(LIB_NAME tviewer)
project("lib${LIB_NAME}")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

# Documentation
set(DOXYFILE_LATEX false)
include(UseDoxygen)

# Compilation
add_definitions(-Wall -pedantic)
add_definitions(-std=c++11)

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
)

add_subdirectory(examples)

# Export variables
set(TVIEWER_INCLUDE_DIRS ${libtviewer_SOURCE_DIR} CACHE STRING "TViewer include directories")
set(TVIEWER_LIBRARIES ${LIB_NAME} CACHE STRING "TViewer libraries")
set(TVIEWER_DEFINITIONS "-std=c++11" CACHE STRING "TViewer compiler definitions")
