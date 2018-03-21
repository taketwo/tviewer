set(QT_EXAMPLE_SOURCES
  main.cpp
  qt_example.cpp
)
set(QT_EXAMPLE_HEADERS
  qt_example.h
)

qt_wrap_ui(QT_EXAMPLE_FORMS_HEADERS qt_example.ui)

add_executable(qt_example
  main.cpp
  qt_example.cpp
  ${QT_EXAMPLE_FORMS_HEADERS}
)
target_link_libraries(qt_example
  ${LIB_NAME}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${QT_LIBRARIES}
)
