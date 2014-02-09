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

#include "utils.h"

namespace tviewer
{

  bool
  matchKeys (const pcl::visualization::KeyboardEvent& key_event, const std::string& key)
  {
    // Special case, keys with Control or Alt: "C-x" or "A-x"
    if (key.size () >= 3 && key[1] == '-')
    {
      const std::string k = key.substr (2);
      if ((key[0] == 'A' && key_event.isAltPressed ()) || (key[0] == 'C' && key_event.isCtrlPressed ()))
        return k == key_event.getKeySym ();
      else
        return false;
    }
    // Regular case
    if (!key_event.isAltPressed () && !key_event.isCtrlPressed ())
      return key_event.getKeySym () == key;
    else
      return false;
  }

}

