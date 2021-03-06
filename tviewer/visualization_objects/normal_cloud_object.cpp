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

#include "normal_cloud_object.h"

template <typename PointT> void
tviewer::NormalCloudObject<PointT>::addDataToVisualizer (pcl::visualization::PCLVisualizer& v)
{
  v.addPointCloudNormals<PointT> (data_, level_, scale_, name_);
}

template <typename PointT> void
tviewer::NormalCloudObject<PointT>::removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v)
{
  v.removePointCloud (name_);
}

template <typename PointT> void
tviewer::NormalCloudObject<PointT>::updateData ()
{
  data_ = retrieve_ ();
}

template <typename T>
tviewer::CreateNormalCloudObject<T>::operator std::shared_ptr<NormalCloudObject<T>> ()
{
  // Need to turn data_ into a local variable, otherwise the lambda does not
  // seem to capture it by value properly.
  auto d = *data_;
  auto l = [=] { return d; };

  return std::make_shared<NormalCloudObject<T>> (name_,
                                                 description_ ? *description_ : name_,
                                                 key_,
                                                 *data_,
                                                 onUpdate_ ? *onUpdate_ : l,
                                                 *level_,
                                                 *scale_);
}

template class tviewer::NormalCloudObject<pcl::PointNormal>;
template class tviewer::NormalCloudObject<pcl::PointXYZRGBNormal>;

template class tviewer::CreateNormalCloudObject<pcl::PointNormal>;
template class tviewer::CreateNormalCloudObject<pcl::PointXYZRGBNormal>;

