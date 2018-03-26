/******************************************************************************
 * Copyright (c) 2014, 2018 Sergey Alexandrov
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

/** \file normal_cloud_object.h
  * Visualization object for points with normals */

#include "visualization_object.h"

namespace tviewer
{

  /// \addtogroup public
  /// @{

  /** Visualization object that displays a cloud of points with normals. */
  template <typename PointT>
  class NormalCloudObject : public VisualizationObject
  {

    public:

      /// Cloud of points with normals.
      using NormalCloud = pcl::PointCloud<PointT>;

      /// Shared pointer to a const cloud of points with normals.
      using NormalCloudConstPtr = typename NormalCloud::ConstPtr;

      /// Function that retrieves data for visualization.
      using RetrieveFunction = std::function<NormalCloudConstPtr ()>;

      /** Construct normal cloud visualization object.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        *            displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] cloud cloud of points with normals to display
        * \param[in] retrieve function that returns a point cloud that is to be
        *            displayed
        * \param[in] level display only every \p level th point
        * \param[in] scale normal arrow scale */
      NormalCloudObject (const std::string& name,
                         const std::string& description,
                         const std::string& key,
                         const NormalCloudConstPtr& cloud,
                         const RetrieveFunction& retrieve,
                         int level = 100,
                         float scale = 0.02)
      : VisualizationObject (name, description, key)
      , data_ (cloud)
      , retrieve_ (retrieve)
      , level_ (level)
      , scale_ (scale)
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

      NormalCloudConstPtr data_;
      RetrieveFunction retrieve_;
      int level_;
      float scale_;

  };

  /** Helper class that provides a fluent interface to simplify instantiation of
    * NormalCloudObject. */
  template <typename T>
  class CreateNormalCloudObject
  {

    private:

      std::string name_;
      std::string key_;

      using Data = typename NormalCloudObject<T>::NormalCloud;
      using DataPtr = typename NormalCloudObject<T>::NormalCloudConstPtr;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreateNormalCloudObject<T>

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (DataPtr, data, DataPtr (new Data));
      NAMED_PARAMETER (typename NormalCloudObject<T>::RetrieveFunction, onUpdate);
      NAMED_PARAMETER (int, level, 100);
      NAMED_PARAMETER (float, scale, 0.02);

#include "../named_parameters/named_parameters_undef.h"

    public:

      /** Constructor that takes required configuration parameters. */
      CreateNormalCloudObject (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      /** Cast operator to NormalCloudObject::Ptr.
        *
        * This function performs instantiation of a NormalCloudObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      operator std::shared_ptr<NormalCloudObject<T>> ();

      /** Cast operator to VisualizationObject::Ptr.
        *
        * This function performs instantiation of a NormalCloudObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      inline operator std::shared_ptr<VisualizationObject> ()
      {
        return this->operator std::shared_ptr<NormalCloudObject<T>> ();
      }

  };

  /// @}

}

