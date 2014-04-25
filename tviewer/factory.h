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

#ifndef TVIEWER_FACTORY_H
#define TVIEWER_FACTORY_H

/** \file factory.h
  * Factory functions to instantiate TViewer */

#include "tviewer_fwd.h"

namespace tviewer
{

  /** Create TViewer.
    *
    * Depending on the \p with_gui flag either a "real" (TViewerImpl) or a
    * "dummy" (TViewerDummy) implementation of TViewer interface will be
    * instantiated. The latter provides the same interface as the functional
    * one, but all methods are no-op. Therefore, a client application can get
    * "no GUI" mode support for free.
    *
    * \ingroup public */
  TViewerPtr
  create (bool with_gui = true);

  /** Create TViewer.
    *
    * This factory function parses command line options to extract parameters
    * relevant for TViewer. These include:
    *
    * * <tt>\-\-tv-no-gui</tt>
    *
    *   If this option is supplied then a "no GUI" version of TViewer
    *   (TViewerDummy) will be instantiated.
    *
    * * <tt>\-\-tv-show-<b>xxx</b></tt>
    *
    *   This is a family of options, where <tt><b>xxx</b></tt> correponds to a
    *   name of some visualization object. Passing such an option forces
    *   displaying of a particular visualization object after it was added.
    *   Note that this only changes the initial state (shown/hidden) of the
    *   object, but does not prevent changing the state of the object later on
    *   (either through user input or programmatically).
    *
    * * <tt>\-\-tv-hide-<b>xxx</b></tt>
    *
    *   Same as "show" option, but to force hiding an object. Note that hiding
    *   has higher priority thas showing, so if a user supplies both "show" and
    *   "hide" options for the same object, then it will not be shown.
    *
    * * <tt>\-\-tv-background "color"</tt>
    *
    *   Force set the background color of TViewer window. Supported color
    *   formats are given in the documentation for getColorFromString().
    *
    * * <tt>\-\-tv-viewpoint filename</tt>
    *
    *   Load and apply camera parameters from a given file.
    *
    * \ingroup public */
  TViewerPtr
  create (int argc, char** argv);

}

#endif /* TVIEWER_FACTORY_H */

