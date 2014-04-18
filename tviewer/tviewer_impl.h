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

#ifndef TVIEWER_TVIEWER_IMPL_H
#define TVIEWER_TVIEWER_IMPL_H

/** \file tviewer_impl.h
  * Real implementation of TViewer interface */

#include <set>
#include <map>
#include <vector>

#include <boost/optional.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "tviewer_interface.h"

namespace tviewer
{

  /** This is a real implementation of TViewer interface. */
  class TViewerImpl : public TViewerInterface
  {

    public:

      /// Disabled copy constructor.
      TViewerImpl (const TViewerImpl&) = delete;

      /// Disabled assignment operator.
      TViewerImpl& operator= (const TViewerImpl&) = delete;

      virtual ~TViewerImpl ();

      virtual void
      run () override;

      virtual void
      sleep (size_t milliseconds) override;

      virtual void
      add (const VisualizationObjectPtr& object, bool show = false, bool update = false) override;

      virtual void
      remove (const std::string& object_name) override;

      virtual void
      show (const std::string& object_name) override;

      virtual void
      show () override;

      virtual void
      hide (const std::string& object_name) override;

      virtual void
      hide () override;

      virtual void
      update (const std::string& object_name) override;

      virtual void
      update () override;

      virtual void
      addListener (const KeyboardListenerPtr& listener,
                   const std::vector<std::string>& dependent_objects = {}) override;

      virtual void
      removeListener (const std::string& listener_name) override;

      virtual void
      saveCameraParameters (const std::string& filename) override;

      virtual void
      loadCameraParameters (const std::string& filename) override;

      virtual bool
      askYesNo (const std::string& question, bool no_with_any_key = true) override;

      virtual bool
      waitKeyPressed () override;

      virtual bool
      waitKeyPressed (std::string& key) override;

      virtual bool
      waitKeyPressed (const std::vector<std::string>& keys) override;

      virtual bool
      waitKeyPressed (std::string& key, const std::vector<std::string>& keys) override;

      virtual bool
      waitPointSelected (size_t& point_index, float& x, float& y, float& z) override;

      virtual bool
      waitPointSelected (size_t& point_index) override;

      virtual bool
      waitPointSelected (Color& point_color) override;

      virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                       std::vector<pcl::PointIndices>& indices,
                                       bool skip_duplicates) override;

      virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                       bool skip_duplicates) override;

      virtual bool waitPointsSelected (std::vector<pcl::PointIndices>& indices,
                                       bool skip_duplicates) override;

      virtual void
      setBackgroundColor (Color color) override;

    private:

      TViewerImpl ();

      void keyboardEventCallback (const pcl::visualization::KeyboardEvent& event);

      void pickPointEventCallback (const pcl::visualization::PointPickingEvent& event);

      void dispatch (const pcl::visualization::KeyboardEvent& key_event);

      void printHelp () const;

      /** A helper function to print a string to console highlighting the
        * characters for which a given function \a highlight returns \c true. */
      static void
      printWithHighlight (const std::string& str, std::function<bool (char)> highlight);

      std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

      typedef boost::optional<pcl::visualization::KeyboardEvent> OptionalKeyboardEvent;
      typedef boost::optional<pcl::visualization::PointPickingEvent> OptionalPointPickingEvent;

      OptionalKeyboardEvent last_keyboard_event_;
      OptionalPointPickingEvent last_point_picking_event_;

      std::vector<VisualizationObjectPtr> objects_;
      std::vector<KeyboardListenerPtr> listeners_;

      std::map<std::string, std::set<std::string>> listener_dependents_;

      bool mode_waiting_user_input_;

      // Names of the objects that should be shown/hidden when added, this is
      // supposed to be set in the factory create() function.
      std::set<std::string> force_show_when_added_;
      std::set<std::string> force_hide_when_added_;

      friend TViewerPtr create (bool);
      friend TViewerPtr create (int argc, char** argv);

  };

}

#endif /* TVIEWER_TVIEWER_IMPL_H */

