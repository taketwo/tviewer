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

#ifndef TVIEWER_VISUALIZATION_OBJECT_H
#define TVIEWER_VISUALIZATION_OBJECT_H

/** \file visualization_object.h
  * Base class for visualization objects */

#include <string>
#include <memory>

#include <boost/any.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "../tviewer_fwd.h"

namespace tviewer
{

  /** Purely abstract base class for visualization objects that may be displayed
    * using TViewer.
    *
    * \ingroup private */
  class VisualizationObject
  {

    public:

      /// Shared pointer to a visualization object.
      typedef std::shared_ptr<VisualizationObject> Ptr;

      /** Construct a visualization object.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        * displayed in help
        * \param[in] key key used to show/hide the object */
      VisualizationObject (const std::string& name,
                           const std::string& description,
                           const std::string& key);

      /** Show this object in the injected viewer. */
      void
      show ();

      /** Hide this object from the injected viewer. */
      void
      hide ();

      /** Toggle the display of this object. */
      void
      toggle ();

      /** Update this object (retrieve new data) and refresh the display. */
      void
      update ();

      /** Refresh the display. */
      void
      refresh ();

      /** Execute a command associated with given keyboard event (if any). */
      virtual bool
      execute (const pcl::visualization::KeyboardEvent& key_event);

      /** Get data element with given index.
        *
        * \return \c true if element with such index exists and can be cast to
        * the specified type, \c false otherwise */
      template <typename T> bool
      at (size_t index, T& item) const
      {
        boost::any any_item;
        if (at_ (index, any_item))
        {
          try
          {
            item = boost::any_cast<T> (any_item);
            return true;
          }
          catch (const boost::bad_any_cast&)
          {
            return false;
          }
        }
        return false;
      }

    protected:

      /** Real implementation for at() function, to be implemented in deriving
        * classes.
        *
        * A templated function can not be virtual, hence this trick with a
        * separate implementation and boost::any. */
      virtual bool
      at_ (size_t, boost::any&) const
      {
        return false;
      }

      /** Add the underlying data to the PCL Visualizer.
        *
        * This function is called from show() and has to be implemented in a
        * deriving class. */
      virtual void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) = 0;

      /** Remove the underlying data from the PCL Visualizer.
        *
        * This function is called from hide() and has to be implemented in a
        * deriving class. */
      virtual void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) = 0;

      /** Refresh the underlying data in the PCL Visualizer.
        *
        * This function is called from refresh(). Default implementation simply
        * invokes removeDataFromVisualizer() and addDataFromVisualizer(), but
        * there might be a more optimal way to refresh the display, in which
        * case a deriving class should override this function. */
      virtual void
      refreshDataInVisualizer (pcl::visualization::PCLVisualizer& v);

      /** Retrieve new underlying data.
        *
        * This function is called from update() and has to be implemented in a
        * deriving class. */
      virtual void
      updateData () = 0;

      /// Object identifier.
      std::string name_;

      /// Short description for help.
      std::string description_;

      /// Key used for toggling display of this object.
      std::string key_;

    private:

      /** Inject a PCL Visualizer that should be used to display the object.
        *
        * This is supposed to be used by TViewerImpl when the object is being
        * registered. */
      void
      injectViewer (std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer);

      // TViewerImpl is a friend because it will need to inject a PCL Visualizer
      friend TViewerImpl;

      /// Display status (visible/hidden).
      bool visible_ = false;

      /// Weak pointer to the injected PCL Visualizer.
      std::weak_ptr<pcl::visualization::PCLVisualizer> viewer_;

  };

}

#endif /* TVIEWER_VISUALIZATION_OBJECT_H */

