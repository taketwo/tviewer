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

#ifndef TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H
#define TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H

/** \file point_cloud_with_color_shuffling_object.h
  * Visualization object for points with color shuffling capability */

#include <map>

#include "point_cloud_object.h"

namespace tviewer
{

  /** Visualization object that displays a cloud of points and allows to
    * randomly shuffle their colors.
    *
    * \ingroup public */
  class PointCloudWithColorShufflingObject : public PointCloudObject<pcl::PointXYZRGBA>
  {

    public:

      /// Cloud of points.
      typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

      /// Shared pointer to a cloud of points.
      typedef typename PointCloud::Ptr PointCloudPtr;

      /// Function that retrieves data for visualization.
      typedef std::function<PointCloudPtr ()> RetrieveFunction;

      /** Construct a visualization object using a pointer to data.
        *
        * See the documentation for PointCloudObject(). */
      PointCloudWithColorShufflingObject (const std::string& name,
                                          const std::string& description,
                                          const std::string& key,
                                          const PointCloudPtr& cloud,
                                          int point_size = 1.0,
                                          float visibility = 1.0,
                                          Color color = 0)
      : PointCloudObject<pcl::PointXYZRGBA> (name, description, key, cloud, point_size, color)
      {
        generateColorMap ();
      }

      /** Construct a visualization object using a function that retrieves data.
        *
        * See the documentation for PointCloudObject(). */
      PointCloudWithColorShufflingObject (const std::string& name,
                                          const std::string& description,
                                          const std::string& key,
                                          const RetrieveFunction& retrieve,
                                          int point_size = 1.0,
                                          float visibility = 1.0,
                                          Color color = 0)
      : PointCloudObject<pcl::PointXYZRGBA> (name, description, key, retrieve, point_size, color)
      {
      }

      virtual bool
      execute (const pcl::visualization::KeyboardEvent& key_event) override;

    protected:

      virtual bool
      at_ (size_t index, boost::any& item) const override;

      virtual void
      updateData () override;

      using PointCloudObject<pcl::PointXYZRGBA>::data_;

    private:

      void
      generateColorMap ();

      void
      shuffleColors ();

      // Maps current color to the original color.
      std::map<Color, Color> color_map_;

  };

}

#endif /* TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H */

