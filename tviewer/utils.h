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

#ifndef TVIEWER_UTILS_H
#define TVIEWER_UTILS_H

/** \file utils.h
  * Utility functions */

#include <pcl/visualization/keyboard_event.h>

namespace tviewer
{

  /** Check if a given keyboard event matches a given key pattern.
    *
    * Key pattern should have the same format as used in \c pcl::visualization
    * with an extension to support Emacs-style modifier keys.
    *
    * Examples:
    *
    * - "a"
    * - "A"
    * - "Escape"
    * - "C-x"
    * - "A-p"
    *
    * The last two examples will match pressing "x" with Control modifier key and
    * "p" with Alt modifier key respectively.
    *
    * \return \c true if the event matches the pattern, \c false otherwise
    *
    * \ingroup private */
  bool
  matchKeys (const pcl::visualization::KeyboardEvent& key_event, const std::string& key);

}

#endif /* TVIEWER_UTILS_H */

