cmake_minimum_required(VERSION 3.1.3)

set(LIB_NAME tviewer)
project("lib${LIB_NAME}")

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 11)

# Set build type to Release if not specified by user
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "-- No build type specified; defaulting to CMAKE_BUILD_TYPE=Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE)
endif()

# High level of warnings
add_definitions(-Wall -Wextra)
# But silence annoying noise from VTK
add_definitions(-Wno-comment)

# Boost Preprocessor library is very conservative in enabling variadics support.
# In fact, if a compiler implements c++11, then we can be sure that variadics
# are supported as well. This define enables preprocessor macros that require
# variadics support (needed by 'named_parameters_def.h').
list(APPEND TVIEWER_DEFINITIONS -DBOOST_PP_VARIADICS=1)

if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcolor-diagnostics")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.8.5)
  # Color diagnostics output was added in GCC 4.9.0 (and GCC 4.8.5 was the last release in the 4.8 series).
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color")
endif()

# Options
option(WITH_QT "Build tviewer with Qt support." OFF)
option(BUILD_DOC "Build tviewer documentation." OFF)
option(BUILD_EXAMPLES "Build tviewer examples." OFF)

# Dependencies
find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common visualization)

# Build a list of options for TViewer by cleaning up PCL_DEFINITIONS
foreach(_def ${PCL_DEFINITIONS})
  string(REPLACE " " ";" _def_filtered ${_def})
  list(APPEND TVIEWER_OPTIONS ${_def_filtered})
endforeach()

if(WITH_QT)
  if(VTK_VERSION VERSION_LESS "6.0")
    message(STATUS "VTK version is 5 or below, therefore using Qt4")
    find_package(Qt4 REQUIRED)
    include(${QT_USE_FILE})
    include(${VTK_USE_FILE})
    macro(qt_wrap_ui)
      qt4_wrap_ui(${ARGN})
    endmacro()
    list(APPEND VTK_LIBRARIES QVTK)
  else()
    message(STATUS "VTK version is 6 or above, therefore using Qt5")
    find_package(Qt5Widgets REQUIRED)
    macro(qt_wrap_ui)
      qt5_wrap_ui(${ARGN})
    endmacro()
    set(QT_LIBRARIES ${Qt5Widgets_LIBRARIES})
  endif()
  # Find includes in corresponding build directories
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
  # Instruct CMake to run moc automatically when needed.
  set(CMAKE_AUTOMOC ON)
  list(APPEND TVIEWER_DEFINITIONS ${QT_DEFINITIONS} "-DQT_NO_KEYWORDS")
  qt_wrap_ui(TVIEWER_WIDGET_FORMS_HEADERS
    tviewer/tviewer_widget/tviewer_widget.ui
  )
  set(TVIEWER_WIDGET_SOURCES
    tviewer/tviewer_widget/tviewer_widget.cpp
    ${TVIEWER_WIDGET_FORMS_HEADERS}
  )
endif()

# TViewer library
add_library(${LIB_NAME}
  tviewer/color.cpp
  tviewer/utils.cpp
  tviewer/tviewer.cpp
  tviewer/factory.cpp
  tviewer/keyboard_listeners/up_down_counter.cpp
  tviewer/keyboard_listeners/binary_state_switch.cpp
  tviewer/visualization_objects/normal_cloud_object.cpp
  tviewer/visualization_objects/point_cloud_object.cpp
  tviewer/visualization_objects/point_cloud_with_color_shuffling_object.cpp
  tviewer/visualization_objects/poly_data_object.cpp
  tviewer/visualization_objects/primitive_array_object.cpp
  tviewer/visualization_objects/visualization_object.cpp
  ${TVIEWER_WIDGET_SOURCES}
)
target_link_libraries(${LIB_NAME}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${QT_LIBRARIES}
)
target_include_directories(${LIB_NAME}
  SYSTEM PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_include_directories(${LIB_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
)
set_target_properties(${LIB_NAME} PROPERTIES CXX_STANDARD 11)
target_compile_options(${LIB_NAME} PUBLIC ${TVIEWER_OPTIONS})
target_compile_definitions(${LIB_NAME} PUBLIC ${TVIEWER_DEFINITIONS})

# Documentation
if(BUILD_DOC)
  set(DOXYFILE_LATEX false)
  include(UseDoxygen)
endif()

# Examples
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

install(
  TARGETS ${LIB_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY tviewer
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
                 PATTERN "*.hpp"
)
