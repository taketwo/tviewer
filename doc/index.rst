Overview
========

TViewer is a small visualization library built on top of PCL Visualizer. It
provides a point cloud viewer that will come in handy when you need to visualize
a number of different objects simultaneously with an ability to show/hide them
individually using keyboard shortcuts.

The viewer maintains a collection of "visualization objects", each of which has
a state (shown or hidden) and an associated keyboard shortcut that the user may
use to toggle the state. You do not need to write any boiler-plate code for
that, but rather simply create a visualization object and register it with the
viewer:

.. code:: cpp

  viewer->add<PointCloudObject<pcl::PointXYZ>> ("cloud", "some point cloud", "c", cloud);

The presented code creates a new visualization object that displays `cloud`
point cloud and that the user may show or hide by pressing `c` key. The objects
themselves could be static or dynamic, which means that they might either have a
particular data (point cloud) associated with them, or a function that generates
new data.

The library also has a few more random features:

- saving/loading of camera viewpoint
- utility functions for handling keyboard input and point picking
- support for no GUI mode
- randomly shuffling colors in point clouds

License
-------

TViewer is released under a permissive MIT license.

.. toctree::
   :maxdepth: 1
   :caption: API Reference

   tviewer
   visualization_objects

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
