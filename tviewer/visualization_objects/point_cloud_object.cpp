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

#include "point_cloud_object.h"

namespace
{

  struct AtVisitor : public boost::static_visitor<bool>
  {

    size_t index_;
    float& x_;
    float& y_;
    float& z_;

    AtVisitor (size_t index, float& x, float& y, float& z)
    : index_ (index)
    , x_ (x)
    , y_ (y)
    , z_ (z)
    {
    }

    template <typename T> bool
    operator() (const T& cloud)
    {
      if (index_ < cloud->size ())
      {
        const auto& item = cloud->at (index_);
        x_ = item.x;
        y_ = item.y;
        z_ = item.z;
        return true;
      }
      return false;
    }

  };

  struct AddVisitor : public boost::static_visitor<>
  {

    pcl::visualization::PCLVisualizer& v_;
    const std::string& name_;
    int point_size_;
    float visibility_;
    boost::optional<tviewer::Color> fixed_color_;

    AddVisitor (pcl::visualization::PCLVisualizer& v, const std::string& name, int point_size, float visibility, boost::optional<tviewer::Color> fixed_color)
    : v_ (v)
    , name_ (name)
    , point_size_ (point_size)
    , visibility_ (visibility)
    , fixed_color_ (fixed_color)
    {
    }

    template <typename T> void
    operator() (const T& cloud)
    {
      using PointType = typename T::element_type::PointType;
      v_.addPointCloud<PointType> (cloud, name_);
      v_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name_);
      v_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, visibility_, name_);
      if (fixed_color_)
      {
        float r, g, b;
        std::tie (r, g, b) = tviewer::getRGBFromColor (*fixed_color_);
        v_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, name_);
      }
    }

  };

  struct UpdateVisitor : public boost::static_visitor<>
  {

    pcl::visualization::PCLVisualizer& v_;
    const std::string& name_;

    UpdateVisitor (pcl::visualization::PCLVisualizer& v, const std::string& name)
    : v_ (v)
    , name_ (name)
    {
    }

    template <typename T> void
    operator() (const T& cloud)
    {
      using PointType = typename T::element_type::PointType;
      v_.updatePointCloud<PointType> (cloud, name_);
    }

  };
}

bool
tviewer::PointCloudObject::at (size_t index, float& x, float& y, float& z) const
{
  AtVisitor at (index, x, y, z);
  return boost::apply_visitor(at, data_);
}

void
tviewer::PointCloudObject::addDataToVisualizer (pcl::visualization::PCLVisualizer& v)
{
  AddVisitor add (v, name_, point_size_, visibility_, fixed_color_);
  boost::apply_visitor (add, data_);
}

void
tviewer::PointCloudObject::removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v)
{
  v.removePointCloud (name_);
}

void
tviewer::PointCloudObject::refreshDataInVisualizer (pcl::visualization::PCLVisualizer& v)
{
  UpdateVisitor update (v, name_);
  boost::apply_visitor (update, data_);
}

void
tviewer::PointCloudObject::updateData ()
{
  data_ = retrieve_ ();
}

tviewer::CreatePointCloudObject::operator std::shared_ptr<PointCloudObject> ()
{
  // Need to turn data_ into a local variable, otherwise the lambda does not
  // seem to capture it by value properly.
  auto d = data_ ? *data_ : pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  auto l = [=] { return d; };

  return std::make_shared<PointCloudObject> (name_,
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

