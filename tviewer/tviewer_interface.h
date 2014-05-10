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

#ifndef TVIEWER_TVIEWER_INTERFACE_H
#define TVIEWER_TVIEWER_INTERFACE_H

/** \file tviewer_interface.h
  * Public TViewer interface */

#include <memory>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>

#include "tviewer_fwd.h"

namespace tviewer
{

  /** Purely abstract class that declares an interface for TViewer.
    * \ingroup public */
  class TViewerInterface
  {

    public:

      /// Shared pointer to an object that implements TViewer interface.
      typedef std::shared_ptr<TViewerInterface> Ptr;

      virtual ~TViewerInterface ()
      { }

      /** Run internal loop updating the screen and waiting for user input. */
      virtual void
      run () = 0;

      /** Update the screen and sleep for the given amount of milliseconds. */
      virtual void
      sleep (size_t milliseconds) = 0;

      /** Register a new visualization object. */
      virtual void
      add (const VisualizationObjectPtr& object, bool show = false, bool update = false) = 0;

      /** Create and register a new visualization object.
        *
        * This function will forward the provided parameters to std::make_shared
        * to create a new visualization object and then register it. */
      template <typename VisualizationObjectT, typename... _Args> inline void
      add (_Args&&... __args)
      {
        add (std::make_shared<VisualizationObjectT> (std::forward<_Args> (__args)...));
      }

      /** Unregister a visualization object. */
      virtual void
      remove (const std::string& object_name) = 0;

      /** Show a given visualization object. */
      virtual void
      show (const std::string& object_name) = 0;

      /** Show all registered visualization objects. */
      virtual void
      show () = 0;

      /** Hide a given visualization object. */
      virtual void
      hide (const std::string& object_name) = 0;

      /** Hide all registered visualization objects. */
      virtual void
      hide () = 0;

      /** Update a given visualization object. */
      virtual void
      update (const std::string& object_name) = 0;

      /** Update all registered visualization objects. */
      virtual void
      update () = 0;

      /** Register a new keyboard listener. */
      virtual void
      addListener (const KeyboardListenerPtr& listener,
                   const std::vector<std::string>& dependent_objects = {}) = 0;

      /** Unregister a keyboard listener. */
      virtual void
      removeListener (const std::string& listener_name) = 0;

      /** Save the current camera parameters (viewpoint) to a given file. */
      virtual void
      saveCameraParameters (const std::string& filename) = 0;

      /** Load and apply camera parameters (viewpoint) from a given file. */
      virtual void
      loadCameraParameters (const std::string& filename) = 0;

      /** Print question and wait until the user presses 'y' or 'n'.
        *
        * This function is case insensitive, i.e. both 'y' and 'Y' mean a
        * positive answer.
        *
        * The "(y/n)" hint will be appended to the question text. Furthermore,
        * after the user answers the question, the answer ("YES" or "NO") will
        * be printed.
        *
        * \param[in] question a question to display
        * \param[in] no_with_any_key treat any key that is not 'y' or 'Y' as no
        *
        * \returns \c true if the user answered yes, \c false otherwise */
      virtual bool
      askYesNo (const std::string& question, bool no_with_any_key = true) = 0;


      /** \name Wait for keyboard input
        *
        * Functions in this group block execution until the user presses
        * (centain) keys. Note that in all cases the main loop will continue to
        * run, so any other interactions with the viewer (e.g. mouse input,
        * toggling display of visualization objects) will continue to work.
        *
        * All of these functions allow the user to interrupt by pressing
        * "Escape", in which case they return \c false.
        *
        * Example usage:
        *
        * \code
        * while (viewer->waitKeyPressed (key, {"i", "I"}))
        * {
        *   if (key == "i")
        *     value--;
        *   else if (key == "I")
        *     value++;
        *   std::cout << "Current value: " << value << std::endl;
        *   viewer->updateAll ();
        * }
        * \endcode */

      ///@{

      /** Wait for any key.
        *
        * This function will wait until the user presses any key and then
        * return. */
      virtual bool
      waitKeyPressed () = 0;

      /** Wait for any key and return the pressed key.
        *
        * \param[out] key key that was pressed */
      virtual bool
      waitKeyPressed (std::string& key) = 0;

      /** Wait for certain keys.
        *
        * This function will wait until the user presses any of the listed keys
        * and then return.
        *
        * \param[in] keys vector of keys to wait for */
      virtual bool
      waitKeyPressed (const std::vector<std::string>& keys) = 0;

      /** Wait for certain keys and return the pressed key.
        *
        * \param[in]  keys vector of keys to wait for
        * \param[out] key key that was pressed */
      virtual bool
      waitKeyPressed (std::string& key, const std::vector<std::string>& keys) = 0;

      ///@}

      /** Wait for a point to be selected and return its index and
        * coordinates.
        *
        * \return \c false if "Escape" was pressed, \c true if a point was
        * selected */
      virtual bool
      waitPointSelected (size_t& point_index, float& x, float& y, float& z) = 0;

      /** Wait for a point to be selected and return only its index.
        *
        * \return \c false if "Escape" was pressed, \c true if a point was
        * selected */
      virtual bool
      waitPointSelected (size_t& point_index) = 0;

      /** Wait for a point to be selected and return its (RGBA) color.
        *
        * \note If the selected point belongs to an object with color shuffling
        * capability, then the original color (i.e. the color that the point had
        * prior to shuffling (if it happened at all)) will be returned.
        *
        * \return \c false if "Escape" was pressed, \c true if a point was
        * selected */
      virtual bool
      waitPointSelected (Color& point_color) = 0;

      /** Wait for set of points to be selected and return them.
        *
        * The selected points are returned as both:
        * - a point cloud with labels
        * - a vector of vectors of point indices (in the original cloud)
        *
        * If only one type of output is desired use corresponding specialized
        * version.
        *
        * \param[out] cloud output cloud with selected points
        * \param[out] indices vector of vector of selected point indices
        * \param[in]  skip_duplicates whether to include a point in the output
        * several times if the user happend to select it several times
        *
        * \return \c false if "Escape" was pressed, \c true if a point was
        * selected */
      virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                       std::vector<pcl::PointIndices>& indices,
                                       bool skip_duplicates = true) = 0;

      /** Wait for set of points to be selected and return them as a point
        * cloud.
        *
        * \see waitPointsSelected() */
      virtual bool waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                       bool skip_duplicates = true) = 0;

      /** Wait for set of points to be selected and return them as a vector of
        * vectors of indices.
        *
        * \see waitPointsSelected() */
      virtual bool waitPointsSelected (std::vector<pcl::PointIndices>& indices,
                                       bool skip_duplicates = true) = 0;

      /** Set background color of the viewer. */
      virtual void
      setBackgroundColor (Color color) = 0;

  };

}

#endif /* TVIEWER_TVIEWER_INTERFACE_H */

