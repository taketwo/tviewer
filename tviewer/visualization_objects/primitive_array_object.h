/******************************************************************************
 * Copyright (c) 2018 Sergey Alexandrov
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

#pragma once

/** \file visualization_objects/primitive_array_object.h
  * Visualization object that manages an array of primitive shapes (spheres, arrows, etc.) */

#include <boost/variant.hpp>

#include "visualization_object.h"

namespace tviewer
{

  /// \addtogroup public
  /// @{

  /** Structure that represents a colored directed arrow. */
  struct Arrow
  {
    Eigen::Vector3f source;  ///< Source point of the arrow.
    Eigen::Vector3f target;  ///< Target point (tip) of the arrow.
    Color color;             ///< Color of the arrow.
  };

  /** Structure that represents a colored sphere. */
  struct Sphere
  {
    Eigen::Vector3f center;  ///< Center of the sphere.
    float radius;            ///< Radius of the sphere.
    Color color;             ///< Color of the sphere.
  };

  /** Structure that represents a cylinder. */
  struct Cylinder
  {
    Eigen::Vector3f center;  ///< Center of the cylinder.
    Eigen::Vector3f axis;    ///< Axis of the cylinder. The norm of the axis
                             ///< defines the height of the cylinder.
    float radius;            ///< Radius of the cylinder.
  };

  /// A primitive shape.
  using Primitive = boost::variant<Arrow, Sphere, Cylinder>;

  /// Array of primitive shapes.
  using Primitives = std::vector<Primitive>;

  /// Shared pointer to an array of primitive shapes.
  using PrimitivesPtr = std::shared_ptr<Primitives>;

  /** Visualization object that displays an array of primitive shapes. */
  class PrimitiveArrayObject : public VisualizationObject
  {

    public:

      using Data = Primitives;
      using DataPtr = PrimitivesPtr;

      /// Function that retrieves data for visualization.
      using RetrieveFunction = std::function<DataPtr ()>;

      /** Construct primitive array visualization object.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        *            displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] data array of primitive shapes to display
        * \param[in] retrieve function that returns an array of primitive shapes
        *            that is to be displayed
        * \param[in] flat_shading if set to \c true flat shading will be used
        *            when rendering primitives, otherwise Phong shading. */
      PrimitiveArrayObject (const std::string& name,
                            const std::string& description,
                            const std::string& key,
                            const DataPtr& data,
                            const RetrieveFunction& retrieve,
                            bool flat_shading)
      : VisualizationObject (name, description, key)
      , data_ (data)
      , retrieve_ (retrieve)
      , flat_shading_ (flat_shading)
      {
      }

    protected:

      void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      updateData () override;

    private:

      DataPtr data_;
      RetrieveFunction retrieve_;
      std::vector<std::string> objects_in_visualizer_;

      bool flat_shading_;

  };

  /** Helper class that provides a fluent interface to simplify instantiation of
    * PrimitiveArrayObject. */
  class CreatePrimitiveArrayObject
  {

    private:

      std::string name_;
      std::string key_;

      using Data = Primitives;
      using DataPtr = PrimitivesPtr;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreatePrimitiveArrayObject

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (DataPtr, data, DataPtr (new Data));
      NAMED_PARAMETER (PrimitiveArrayObject::RetrieveFunction, onUpdate);
      NAMED_PARAMETER (bool, flatShading, false, true);

#include "../named_parameters/named_parameters_undef.h"

    public:

      /** Constructor that takes required configuration parameters. */
      CreatePrimitiveArrayObject (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      /** Cast operator to PrimitiveArrayObject::Ptr.
        *
        * This function performs instantiation of an PrimitiveArrayObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      operator std::shared_ptr<PrimitiveArrayObject> ();

      /** Cast operator to VisualizationObject::Ptr.
        *
        * This function performs instantiation of an PrimitiveArrayObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      inline operator std::shared_ptr<VisualizationObject> ()
      {
        return this->operator std::shared_ptr<PrimitiveArrayObject> ();
      }

  };

  /// @}

}

