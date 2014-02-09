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

#ifndef TVIEWER_NORMAL_CLOUD_OBJECT_H
#define TVIEWER_NORMAL_CLOUD_OBJECT_H

/** \file normal_cloud_object.h
  * Visualization object for points with normals */

#include "visualization_object.h"

namespace tviewer
{

  /** Visualization object that displays a cloud of points with normals.
    *
    * \ingroup public */
  class NormalCloudObject : public VisualizationObject
  {

    public:

      /// Cloud of points with normals.
      typedef pcl::PointCloud<pcl::PointNormal> NormalCloud;

      /// Shared pointer to a const cloud of points with normals.
      typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

      /// Function that retrieves data for visualization.
      typedef std::function<NormalCloudConstPtr ()> RetrieveFunction;

      /** Construct a visualization object using a pointer to data.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        * displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] cloud cloud of points with normals to display
        * \param[in] level display only every \p level th point (default: 100)
        * \param[in] scale normal arrow scale (default: 0.02 m) */
      NormalCloudObject (const std::string& name,
                         const std::string& description,
                         const std::string& key,
                         const NormalCloudConstPtr& cloud,
                         int level = 100,
                         float scale = 0.02)
      : VisualizationObject (name, description, key)
      , data_ (cloud)
      , level_ (level)
      , scale_ (scale)
      {
      }

      /** Construct a visualization object using a function that retrieves data.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        * displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] retrieve function that returns point cloud data that is to
        * be displayed
        * \param[in] level display only every \p level th point (default: 100)
        * \param[in] scale normal arrow scale (default: 0.02 m) */
      NormalCloudObject (const std::string& name,
                         const std::string& description,
                         const std::string& key,
                         const RetrieveFunction& retrieve,
                         int level = 100,
                         float scale = 0.02)
      : VisualizationObject (name, description, key)
      , data_ (new NormalCloud)
      , retrieve_ (retrieve)
      , level_ (level)
      , scale_ (scale)
      {
      }

    protected:

      virtual void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) const override
      {
        v.addPointCloudNormals<pcl::PointNormal> (data_, level_, scale_, name_);
      }

      virtual void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) const override
      {
        v.removePointCloud (name_);
      }

      virtual void
      updateData () override
      {
        if (retrieve_)
          data_ = retrieve_ ();
      }

    private:

      NormalCloudConstPtr data_;
      RetrieveFunction retrieve_;
      int level_;
      float scale_;

  };

}

#endif /* TVIEWER_NORMAL_CLOUD_OBJECT_H */

