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

/** \file point_cloud_object.h
  * Visualization object for points */

#include <boost/variant.hpp>

#include "../color.h"
#include "visualization_object.h"

namespace tviewer
{

  /// \addtogroup public
  /// @{

  /** Visualization object that displays a cloud of points. */
  class PointCloudObject : public VisualizationObject
  {

    public:

      /// Shared pointer to a cloud of points.
      using PointCloudPtr = boost::variant<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr,
                                           pcl::PointCloud<pcl::PointXYZL>::Ptr,
                                           pcl::PointCloud<pcl::PointNormal>::Ptr,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
                                           pcl::PointCloud<pcl::PointXYZRGBA>::Ptr,
                                           pcl::PointCloud<pcl::PointXYZRGBL>::Ptr>;

      /// Function that retrieves data for visualization.
      using RetrieveFunction = std::function<PointCloudPtr ()>;

      /** Construct point cloud visualization object.
        *
        * The last parameter controls the color used to display the points.
        * When set to non-zero, the points are displayed using the RGBA color
        * determined by that number. When set to zero, the color stored inside
        * the points is used, or white if the point type does not have color
        * information.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        *            displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] cloud cloud of points to display
        * \param[in] retrieve function that returns a point cloud that is to be
        *            displayed
        * \param[in] point_size size of points
        * \param[in] visibility opacity of points (0.0 is transparent, 1.0 is
        *            opaque)
        * \param[in] use_fixed_color if set to \c true will ignore the color
        *            data stored in points (if any) and display them using a
        *            fixed color (as given by \c color parameter)
        * \param[in] color color of points */
      PointCloudObject (const std::string& name,
                        const std::string& description,
                        const std::string& key,
                        const PointCloudPtr& cloud,
                        const RetrieveFunction& retrieve,
                        int point_size,
                        float visibility,
                        bool use_fixed_color,
                        Color color)
      : VisualizationObject (name, description, key)
      , data_ (cloud)
      , retrieve_ (retrieve)
      , point_size_ (point_size)
      , visibility_ (visibility)
      {
        if (use_fixed_color)
          fixed_color_ = color;
      }

      bool
      at (size_t index, float& x, float& y, float& z) const override;

    protected:

      void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      refreshDataInVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      updateData () override;

      /// Data for visualization.
      PointCloudPtr data_;

      /// Function which retrieves new data for visualization.
      RetrieveFunction retrieve_;

    private:

      int point_size_;
      float visibility_;
      boost::optional<Color> fixed_color_;

  };

  /** Helper class that provides a fluent interface to simplify instantiation of
    * PointCloudObject. */
  class CreatePointCloudObject
  {

    private:

      std::string name_;
      std::string key_;

      using DataPtr = typename PointCloudObject::PointCloudPtr;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreatePointCloudObject

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (DataPtr, data);
      NAMED_PARAMETER (typename PointCloudObject::RetrieveFunction, onUpdate);
      NAMED_PARAMETER (int, pointSize, 1);
      NAMED_PARAMETER (float, visibility, 1.0);
      NAMED_PARAMETER (Color, color);

#include "../named_parameters/named_parameters_undef.h"

    public:

      /** Constructor that takes required configuration parameters. */
      CreatePointCloudObject (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      /** Cast operator to PointCloudObject<T>::Ptr.
        *
        * This function performs instantiation of an PointCloudObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      operator std::shared_ptr<PointCloudObject> ();

      /** Cast operator to VisualizationObject::Ptr.
        *
        * This function performs instantiation of an PointCloudObject. It is
        * supposed to be called after configuring the object using the fluent
        * interface functions. */
      inline operator std::shared_ptr<VisualizationObject> ()
      {
        return this->operator std::shared_ptr<PointCloudObject> ();
      }

  };

  /// @}

}

