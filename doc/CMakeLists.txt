find_program(DOXYGEN doxygen)

if(NOT DOXYGEN)
  message(STATUS "Target 'doc' disabled (requires doxygen)")
else()
  add_custom_target(doc
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/build.py
    SOURCES build.py conf.py index.rst
  )
  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/
    DESTINATION share/doc/tviewer OPTIONAL
  )
endif()
