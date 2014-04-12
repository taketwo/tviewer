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

#ifndef TVIEWER_ARROW_ARRAY_OBJECT_H
#define TVIEWER_ARROW_ARRAY_OBJECT_H

/** \file visualization_objects/arrow_array_object.h
  * Visualization object for array of arrows */

#include "visualization_object.h"

namespace tviewer
{

  /// \addtogroup public
  /// @{

  /** Structure that represents a colored directed arrow.
    *
    * A set of arrows can be visualized using ArrowArrayObject. */
  struct Arrow
  {
    Eigen::Vector3f source;  ///< Source point of the arrow.
    Eigen::Vector3f target;  ///< Target point (tip) of the arrow.
    Color color;             ///< Color of the arrow.
  };

  /// Array of arrows.
  typedef std::vector<Arrow> Arrows;

  /// Shared pointer to an array of arrows.
  typedef std::shared_ptr<Arrows> ArrowsPtr;

  /** Visualization object that displays an array of arrows. */
  class ArrowArrayObject : public VisualizationObject
  {

    public:

      /// Function that retrieves data for visualization.
      typedef std::function<ArrowsPtr ()> RetrieveFunction;

      /** Construct arrow array visualization object.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        *            displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] data array of arrows to display
        * \param[in] retrieve function that returns an array of arrows that is
        *            to be displayed
        * \param[in] invert_direction if set to \c true the arrows will be
        *            pointing in the opposite direction (i.e. from \c target
        *            to \c source) */
      ArrowArrayObject (const std::string& name,
                        const std::string& description,
                        const std::string& key,
                        const ArrowsPtr& data,
                        const RetrieveFunction& retrieve,
                        bool invert_direction)
      : VisualizationObject (name, description, key)
      , data_ (data)
      , retrieve_ (retrieve)
      , invert_direction_ (invert_direction)
      {
      }

    protected:

      virtual void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) override;

      virtual void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) override;

      virtual void
      updateData () override;

    private:

      inline std::string
      createId (size_t i) const;

      ArrowsPtr data_;
      RetrieveFunction retrieve_;
      bool invert_direction_;
      std::vector<std::string> arrows_in_visualizer_;

  };

  /** Helper class that provides a fluent interface to simplify instantiation of
    * ArrowArrayObject. */
  class CreateArrowArrayObject
  {

    private:

      std::string name_;
      std::string key_;

      typedef Arrows Data;
      typedef ArrowsPtr DataPtr;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreateArrowArrayObject

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (DataPtr, data, DataPtr (new Data));
      NAMED_PARAMETER (ArrowArrayObject::RetrieveFunction, onUpdate);
      NAMED_PARAMETER (bool, invertDirection, false, true);

#include "../named_parameters/named_parameters_undef.h"

    public:

      /** Constructor that takes required configuration parameters. */
      CreateArrowArrayObject (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      /** Cast operator to ArrowArrayObject::Ptr.
        *
        * This function performs instantiation of an ArrowArrayObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      operator std::shared_ptr<ArrowArrayObject> ();

      /** Cast operator to VisualizationObject::Ptr.
        *
        * This function performs instantiation of an ArrowArrayObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      inline operator std::shared_ptr<VisualizationObject> ()
      {
        return this->operator std::shared_ptr<ArrowArrayObject> ();
      }

  };

  /// @}

}

#endif /* TVIEWER_ARROW_ARRAY_OBJECT_H */

