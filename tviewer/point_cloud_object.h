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

#ifndef TVIEWER_POINT_CLOUD_OBJECT_H
#define TVIEWER_POINT_CLOUD_OBJECT_H

/** \file point_cloud_object.h
  * Visualization object for points */

#include "color.h"
#include "visualization_object.h"

namespace tviewer
{

  /** Visualization object that displays a cloud of points.
    *
    * \ingroup public */
  template <typename PointT>
  class PointCloudObject : public VisualizationObject
  {

    public:

      /// Cloud of points.
      typedef pcl::PointCloud<PointT> PointCloud;

      /// Shared pointer to a cloud of points.
      typedef typename PointCloud::Ptr PointCloudPtr;

      /// Function that retrieves data for visualization.
      typedef std::function<PointCloudPtr ()> RetrieveFunction;

      /** Construct a visualization object using a pointer to data.
        *
        * The last parameter controls the color used to display the points.
        * When set to non-zero, the points are displayed using the RGBA color
        * determined by that number. When set to zero, the color stored inside
        * the points is used, or white if the point type does not have color
        * information.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        * displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] cloud cloud of points to display
        * \param[in] point_size size of points (default: 1.0)
        * \param[in] visibility opacity of points (default: 1.0, opaque)
        * \param[in] color color of points (default: 0) */
      PointCloudObject (const std::string& name,
                        const std::string& description,
                        const std::string& key,
                        const PointCloudPtr& cloud,
                        int point_size = 1.0,
                        float visibility = 1.0,
                        Color color = 0)
      : VisualizationObject (name, description, key)
      , data_ (cloud)
      , point_size_ (point_size)
      , visibility_ (visibility)
      , color_ (color)
      {
      }

      /** Construct a visualization object using a function that retrieves data.
        *
        * The last parameter controls the color used to display the points.
        * When set to non-zero, the points are displayed using the RGBA color
        * determined by that number. When set to zero, the color stored inside
        * the points is used, or white if the point type does not have color
        * information.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        * displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] retrieve function that returns point cloud data that is to
        * be displayed
        * \param[in] point_size size of points (default: 1.0)
        * \param[in] visibility opacity of points (default: 1.0, opaque)
        * \param[in] color color of points (default: 0) */
      PointCloudObject (const std::string& name,
                        const std::string& description,
                        const std::string& key,
                        const RetrieveFunction& retrieve,
                        int point_size = 1.0,
                        float visibility = 1.0,
                        Color color = 0)
      : VisualizationObject (name, description, key)
      , data_ (new PointCloud)
      , retrieve_ (retrieve)
      , point_size_ (point_size)
      , visibility_ (visibility)
      , color_ (color)
      {
      }

    protected:

      virtual bool
      at_ (size_t index, boost::any& item) const override;

      virtual void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) const override;

      virtual void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) const override;

      virtual void
      refreshDataInVisualizer (pcl::visualization::PCLVisualizer& v) const override;

      virtual void
      updateData () override;

      /// Data for visualization.
      PointCloudPtr data_;

    private:

      RetrieveFunction retrieve_;
      int point_size_;
      float visibility_;
      Color color_;

  };

}

#endif /* TVIEWER_POINT_CLOUD_OBJECT_H */

