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
