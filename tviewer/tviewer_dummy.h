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

#ifndef TVIEWER_TVIEWER_DUMMY_H
#define TVIEWER_TVIEWER_DUMMY_H

/** \file tviewer_dummy.h
  * Dummy implementation of TViewer interface */

#include "tviewer_interface.h"

namespace tviewer
{

  /** This is a dummy implementation of TViewer interface that my be used if no
    * GUI is needed.
    *
    * All the functions do nothing and return \c false. */
  class TViewerDummy : public TViewerInterface
  {

    public:

      /// Disabled copy constructor.
      TViewerDummy (const TViewerDummy&) = delete;

      /// Disabled assignment operator.
      TViewerDummy& operator= (const TViewerDummy&) = delete;

      virtual ~TViewerDummy ()
      { }

      virtual void
      run () override
      { }

      virtual void
      sleep (size_t) override
      { }

      virtual void
      add (const VisualizationObjectPtr&, bool = false, bool = false) override
      { }

      virtual void
      remove (const std::string&) override
      { }

      virtual void
      show (const std::string&) override
      { }

      virtual void
      show () override
      { }

      virtual void
      hide (const std::string&) override
      { }

      virtual void
      hide () override
      { }

      virtual void
      update (const std::string&) override
      { }

      virtual void
      update () override
      { }

      virtual void
      addListener (const KeyboardListenerPtr&,
                   const std::vector<std::string>& = {}) override
      { }

      virtual void
      removeListener (const std::string&) override
      { }

      virtual void
      saveCameraParameters (const std::string&) override
      { }

      virtual void
      loadCameraParameters (const std::string&) override
      { }

      virtual bool
      askYesNo (const std::string&, bool = true) override
      { return false; }

      virtual bool
      waitKeyPressed () override
      { return false; }

      virtual bool
      waitKeyPressed (std::string&) override
      { return false; }

      virtual bool
      waitKeyPressed (const std::vector<std::string>&) override
      { return false; }

      virtual bool
      waitKeyPressed (std::string&, const std::vector<std::string>&) override
      { return false; }

      virtual bool
      waitPointSelected (size_t&, float&, float&, float&) override
      { return false; }

      virtual bool
      waitPointSelected (size_t&) override
      { return false; }

      virtual bool
      waitPointSelected (Color&) override
      { return false; }

      virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>&,
                                       std::vector<pcl::PointIndices>&,
                                       bool) override
      { return false; }

      virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>&,
                                       bool) override
      { return false; }

      virtual bool waitPointsSelected (std::vector<pcl::PointIndices>&,
                                       bool) override
      { return false; }

      virtual void
      setBackgroundColor (Color) override
      { }

    private:

      TViewerDummy ()
      { }

      friend TViewerPtr create (bool);
      friend TViewerPtr create (int argc, char** argv);

  };

}

#endif /* TVIEWER_TVIEWER_DUMMY_H */

