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

#include <boost/algorithm/string/predicate.hpp>

#include <pcl/console/parse.h>

#include "color.h"
#include "factory.h"
#include "tviewer_impl.h"
#include "tviewer_dummy.h"

namespace tviewer
{

  TViewerPtr
  create (bool with_gui)
  {
    return with_gui ? TViewerPtr (new TViewerImpl) : TViewerPtr (new TViewerDummy);
  }

  TViewerPtr
  create (int argc, char** argv)
  {
    if (pcl::console::find_switch (argc, argv, "--tv-no-gui"))
    {
      return create (false);
    }
    else
    {
      auto viewer = std::shared_ptr<TViewerImpl> (new TViewerImpl);
      for (int i = 1; i < argc; ++i)
      {
        std::string arg (argv[i]);
        if (arg.size () > 10)
        {
          std::string object = arg.substr (10);
          if (boost::starts_with (arg, "--tv-show-"))
            viewer->force_show_when_added_.insert (object);
          if (boost::starts_with (arg, "--tv-hide-"))
            viewer->force_hide_when_added_.insert (object);
        }
      }
      if (pcl::console::find_switch (argc, argv, "--tv-background"))
      {
        std::string background;
        pcl::console::parse (argc, argv, "--tv-background", background);
        viewer->setBackgroundColor (getColorFromString (background));
      }
      if (pcl::console::find_switch (argc, argv, "--tv-viewpoint"))
      {
        std::string filename;
        pcl::console::parse (argc, argv, "--tv-viewpoint", filename);
        viewer->loadCameraParameters (filename);
      }
      return viewer;
    }
  }

}

