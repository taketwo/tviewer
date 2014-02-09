TViewer is a small visualization library built on top of PCL Visualizer. It
provides a point cloud viewer that will come in handy when you need to visualize
a number of different objects simultaneously with an ability to show/hide them
individually using keyboard shortcuts.

The viewer maintains a collection of "visualization objects", each of which has
a state (shown or hidden) and an associated keyboard shortcut that the user may
use to toggle the state. You do not need to write any boiler-plate code for
that, but rather simply create a visualization object and register it with the
viewer:

```cpp
viewer->add<PointCloudObject<pcl::PointXYZ>> ("cloud", "some point cloud", "c", cloud);
```

The presented code creates a new visualization object that displays `cloud`
point cloud and that the user may show or hide by pressing `c` key. The objects
themselves could be static or dynamic, which means that they might either have a
particular data (point cloud) associated with them, or a function that generates
new data.

The library also has a few more random features:

* saving/loading of camera viewpoint
* utility functions for handling keyboard input and point picking
* support for no GUI mode
* randomly shuffling colors in point clouds

Usage
-----

In order to use TViewer simply clone this repository to the root of your project
and drop the following lines into your '*CMakeLists.txt*':

```cmake
add_subdirectory(${CMAKE_SOURCE_DIR}/tviewer)
include_directories(${TVIEWER_INCLUDE_DIRS})
add_definitions(${TVIEWER_DEFINITIONS})
```

and do not forget to link your target with the library:

```cmake
add_executable(my_app
  my_app.cpp
)
target_link_libraries(my_app
  ${PCL_LIBRARIES}
  ${TVIEWER_LIBRARIES}
)
```

In the source code you only need to include '*tviewer/tviewer.h*'.
