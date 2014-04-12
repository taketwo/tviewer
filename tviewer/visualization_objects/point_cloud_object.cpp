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

#include "point_cloud_object.h"

template <typename PointT> bool
tviewer::PointCloudObject<PointT>::at_ (size_t index, boost::any& item) const
{
  if (data_ && index < data_->size ())
  {
    item = data_->at (index);
    return true;
  }
  return false;
}

template <typename PointT> void
tviewer::PointCloudObject<PointT>::addDataToVisualizer (pcl::visualization::PCLVisualizer& v)
{
  v.addPointCloud (data_, name_);
  v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name_);
  v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, visibility_, name_);
  if (use_fixed_color_ != 0)
  {
    float r, g, b;
    std::tie (r, g, b) = getRGBFromColor (color_);
    v.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name_);
  }
}

template <typename PointT> void
tviewer::PointCloudObject<PointT>::removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v)
{
  v.removePointCloud (name_);
}

template <typename PointT> void
tviewer::PointCloudObject<PointT>::refreshDataInVisualizer (pcl::visualization::PCLVisualizer& v)
{
  v.updatePointCloud (data_, name_);
}

template <typename PointT> void
tviewer::PointCloudObject<PointT>::updateData ()
{
  data_ = retrieve_ ();
}

template <typename T>
tviewer::CreatePointCloudObject<T>::operator std::shared_ptr<PointCloudObject<T>> ()
{
  // Need to turn data_ into a local variable, otherwise the lambda does not
  // seem to capture it by value properly.
  auto d = *data_;
  auto l = [=] { return d; };

  return std::make_shared<PointCloudObject<T>> (name_,
                                                description_ ? *description_ : name_,
                                                key_,
                                                *data_,
                                                onUpdate_ ? *onUpdate_ : l,
                                                *pointSize_,
                                                *visibility_,
                                                color_ ? true : false,
                                                color_ ? *color_ : 0
                                                );
}

template class tviewer::PointCloudObject<pcl::PointXYZ>;
//template class tviewer::PointCloudObject<pcl::PointXYZL>;
template class tviewer::PointCloudObject<pcl::PointXYZRGB>;
template class tviewer::PointCloudObject<pcl::PointXYZRGBA>;

template class tviewer::CreatePointCloudObject<pcl::PointXYZ>;
//template class tviewer::CreatePointCloudObject<pcl::PointXYZL>;
template class tviewer::CreatePointCloudObject<pcl::PointXYZRGB>;
template class tviewer::CreatePointCloudObject<pcl::PointXYZRGBA>;

