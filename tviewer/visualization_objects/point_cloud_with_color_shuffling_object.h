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

      /** Construct point cloud with shuffling visualization object.
        *
        * See the documentation for PointCloudObject(). */
      PointCloudWithColorShufflingObject (const std::string& name,
                                          const std::string& description,
                                          const std::string& key,
                                          const PointCloudPtr& cloud,
                                          const RetrieveFunction& retrieve,
                                          int point_size,
                                          float visibility)
      : PointCloudObject<pcl::PointXYZRGBA> (name,
                                             description,
                                             key,
                                             cloud,
                                             retrieve,
                                             point_size,
                                             visibility,
                                             false,
                                             0)
      {
        generateColorMap ();
      }

      virtual bool
      execute (const pcl::visualization::KeyboardEvent& key_event) override;

    protected:

      virtual bool
      at_ (size_t index, boost::any& item) const override;

      virtual void
      updateData () override;

      using PointCloudObject<pcl::PointXYZRGBA>::data_;
      using PointCloudObject<pcl::PointXYZRGBA>::retrieve_;

    private:

      void
      generateColorMap ();

      void
      shuffleColors ();

      // Maps current color to the original color.
      std::map<Color, Color> color_map_;

  };

  class CreatePointCloudWithColorShufflingObject
  {

    private:

      std::string name_;
      std::string key_;

      typedef PointCloudWithColorShufflingObject::PointCloud Data;
      typedef PointCloudWithColorShufflingObject::PointCloudPtr DataPtr;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreatePointCloudWithColorShufflingObject

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (DataPtr, data, DataPtr (new Data));
      NAMED_PARAMETER (PointCloudWithColorShufflingObject::RetrieveFunction, onUpdate);
      NAMED_PARAMETER (int, pointSize, 1);
      NAMED_PARAMETER (float, visibility, 1.0);

#include "../named_parameters/named_parameters_undef.h"

    public:

      CreatePointCloudWithColorShufflingObject (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      operator std::shared_ptr<PointCloudWithColorShufflingObject> ();

      inline operator std::shared_ptr<VisualizationObject> ()
      {
        return this->operator std::shared_ptr<PointCloudWithColorShufflingObject> ();
      }

  };

}

#endif /* TVIEWER_POINT_CLOUD_WITH_COLOR_SHUFFLING_OBJECT_H */

