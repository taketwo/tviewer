include_directories(..)

find_package(PCL
  1.7
  REQUIRED
  COMPONENTS common
             features
             visualization
)

add_executable(usage
  usage.cpp
)
target_link_libraries(usage
  ${LIB_NAME}
  ${PCL_LIBRARIES}
)

add_executable(example_point_clouds
  point_clouds.cpp
)
target_link_libraries(example_point_clouds
  ${LIB_NAME}
  ${PCL_LIBRARIES}
)

add_executable(example_keyboard_listeners
  example_keyboard_listeners.cpp
)
target_link_libraries(example_keyboard_listeners
  ${LIB_NAME}
)

if (WITH_QT)
  add_subdirectory(qt)
endif ()
