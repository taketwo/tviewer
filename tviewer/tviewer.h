/******************************************************************************
 * Copyright (c) 2014 Sergey Alexandrov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#ifndef TVIEWER_TVIEWER_H
#define TVIEWER_TVIEWER_H

/** \file tviewer.h
  * Main header of the library */

/**

\mainpage TViewer

TViewer is a small visualization library built on top of PCL Visualizer. It
provides a point cloud viewer that will come in handy when you need to visualize
a number of different objects simultaneously with an ability to show/hide them
individually using keyboard shortcuts.

The viewer maintains a collection of "visualization objects", each of which has
a state (shown or hidden) and an associated keyboard shortcut that the user may
use to toggle the state. You do not need to write any boiler-plate code for
that, but rather simply create a visualization object and register it with the
viewer:

\code
viewer->add<PointCloudObject<pcl::PointXYZ>> ("cloud", "some point cloud", "c", cloud);
\endcode

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

\section License

TViewer is released under a permissive MIT license.

\defgroup public Public interface
\defgroup private Private members

*/

#include "tviewer_fwd.h"

#include "tviewer_interface.h"
#include "tviewer_dummy.h"
#include "tviewer_impl.h"

#include "keyboard_listeners/keyboard_listener.h"
#include "keyboard_listeners/up_down_counter.h"
#include "keyboard_listeners/binary_state_switch.h"

#include "visualization_objects/visualization_object.h"
#include "visualization_objects/poly_data_object.h"
#include "visualization_objects/arrow_array_object.h"
#include "visualization_objects/point_cloud_object.h"
#include "visualization_objects/normal_cloud_object.h"
#include "visualization_objects/point_cloud_with_color_shuffling_object.h"

#include "factory.h"

#endif /* TVIEWER_TVIEWER_H */

