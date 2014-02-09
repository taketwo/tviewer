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
    * Depending on the \p with_gui flag either a "real" or a "dummy"
    * implementation of TViewer interface will be instantiated. A client
    * application can therefore get "no GUI" mode support for free.
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
    * \ingroup public */
  TViewerPtr
  create (int argc, char** argv);

}

#endif /* TVIEWER_FACTORY_H */

