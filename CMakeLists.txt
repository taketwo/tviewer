cmake_minimum_required(VERSION 2.6)

set(LIB_NAME tviewer)
project("lib${LIB_NAME}")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

option(WITH_QT "Build tviewer with Qt." OFF)

option(BUILD_DOC "Build tviewer documentation." OFF)
option(BUILD_EXAMPLES "Build tviewer examples." OFF)

# Compilation
add_definitions(-std=c++11)
add_definitions(-Wall -Wextra -pedantic)
# Silence warnings from vtk includes
add_definitions(-Wno-extra-semi)
# Boost Preprocessor library is very conservative in enabling variadics support.
# In fact, if a compiler implements c++11, then we can be sure that variadics
# are supported as well. This define enables preprocessor macros that require
# variadics support (needed by 'named_parameters_def.h').
add_definitions(-DBOOST_PP_VARIADICS=1)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-fcolor-diagnostics)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (WITH_QT)
  find_package(Qt4 REQUIRED COMPONENTS QTCORE QTGUI)
  include(${QT_USE_FILE})
  add_definitions(${QT_DEFINITIONS} "-DQT_NO_KEYWORDS")
  include_directories(${CMAKE_BINARY_DIR})
  QT4_WRAP_CPP(TVIEWER_WIDGET_HEADERS_MOC
    tviewer/tviewer_widget/tviewer_widget.h
  )
  QT4_WRAP_UI(TVIEWER_WIDGET_FORMS_HEADERS
    tviewer/tviewer_widget/tviewer_widget.ui
  )
  set(TVIEWER_WIDGET_SOURCES
    tviewer/tviewer_widget/tviewer_widget.cpp
    ${TVIEWER_WIDGET_HEADERS_MOC}
    ${TVIEWER_WIDGET_FORMS_HEADERS}
  )
endif ()

# TViewer library
add_library(${LIB_NAME}
  tviewer/color.cpp
  tviewer/utils.cpp
  tviewer/tviewer.cpp
  tviewer/factory.cpp
  tviewer/keyboard_listeners/up_down_counter.cpp
  tviewer/keyboard_listeners/binary_state_switch.cpp
  tviewer/visualization_objects/poly_data_object.cpp
  tviewer/visualization_objects/arrow_array_object.cpp
  tviewer/visualization_objects/point_cloud_object.cpp
  tviewer/visualization_objects/normal_cloud_object.cpp
  tviewer/visualization_objects/visualization_object.cpp
  tviewer/visualization_objects/point_cloud_with_color_shuffling_object.cpp
  ${TVIEWER_WIDGET_SOURCES}
)

# Export variables
set(TVIEWER_INCLUDE_DIRS ${libtviewer_SOURCE_DIR} CACHE STRING "TViewer include directories")
set(TVIEWER_LIBRARIES ${LIB_NAME} CACHE STRING "TViewer libraries")
set(TVIEWER_DEFINITIONS "-std=c++11 -DBOOST_PP_VARIADICS=1" CACHE STRING "TViewer compiler definitions")

# Documentation
if (BUILD_DOC)
  set(DOXYFILE_LATEX false)
  include(UseDoxygen)
endif()

# Examples
if (BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

