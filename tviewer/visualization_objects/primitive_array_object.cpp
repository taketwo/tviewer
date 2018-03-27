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

#include <iostream>

#include <boost/format.hpp>

#include "../color.h"
#include "primitive_array_object.h"

namespace
{

  struct AddVisitor : public boost::static_visitor<>
  {

    pcl::visualization::PCLVisualizer& v_;
    const std::string& name_;
    std::vector<std::string>& objects_;
    boost::format fmt_;
    pcl::visualization::ShadingRepresentationProperties shading_;

    AddVisitor (pcl::visualization::PCLVisualizer& v, const std::string& name, std::vector<std::string>& objects, bool flat_shading)
    : v_ (v)
    , name_ (name)
    , objects_ (objects)
    , fmt_ ("%s-%i-%s")
    , shading_ (flat_shading ? pcl::visualization::PCL_VISUALIZER_SHADING_FLAT : pcl::visualization::PCL_VISUALIZER_SHADING_PHONG)
    {
    }

    const std::string&
    makeId (const std::string& type)
    {
      objects_.push_back (boost::str (fmt_ % name_ % objects_.size () % type));
      return objects_.back ();
    }

    void operator() (tviewer::Arrow arrow)
    {
      pcl::PointXYZ p1, p2;
      p1.getVector3fMap () = arrow.source;
      p2.getVector3fMap () = arrow.target;
      float r, g, b;
      std::tie (r, g, b) = tviewer::getRGBFromColor (arrow.color);
      // The arrow head is attached to the first point, so we pass the target first.
      v_.addArrow (p2, p1, r, g, b, false, makeId ("arrow"));
    }

    void operator() (tviewer::Sphere sphere)
    {
        pcl::PointXYZ p;
        p.getVector3fMap () = sphere.center;
        float r, g, b;
        std::tie (r, g, b) = tviewer::getRGBFromColor (sphere.color);
        const auto& id = makeId ("sphere");
        v_.addSphere (p, sphere.radius, r, g, b, id);
        v_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, shading_, id);
    }

    void operator() (tviewer::Cylinder cylinder)
    {
      pcl::ModelCoefficients coefficients;
      coefficients.values.resize (7);
      cylinder.center -= (cylinder.axis / 2);
      std::memcpy (&coefficients.values[0], cylinder.center.data (), 3 * sizeof (float));
      std::memcpy (&coefficients.values[3], cylinder.axis.data (), 3 * sizeof (float));
      coefficients.values[6] = cylinder.radius;
      const auto& id = makeId ("cylinder");
      v_.addCylinder (coefficients, id);
      v_.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, shading_, id);
    }

  };

}

void
tviewer::PrimitiveArrayObject::addDataToVisualizer (pcl::visualization::PCLVisualizer& v)
{
  AddVisitor add (v, name_, objects_in_visualizer_, flat_shading_);
  for (const auto& item : *data_)
    boost::apply_visitor (add, item);
}

void
tviewer::PrimitiveArrayObject::removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v)
{
  for (const auto& object : objects_in_visualizer_)
    v.removeShape (object);
  objects_in_visualizer_.clear ();
}

void
tviewer::PrimitiveArrayObject::updateData ()
{
  data_ = retrieve_ ();
}

tviewer::CreatePrimitiveArrayObject::operator std::shared_ptr<PrimitiveArrayObject> ()
{
  // Need to turn data_ into a local variable, otherwise the lambda does not
  // seem to capture it by value properly.
  auto d = *data_;
  auto l = [=] { return d; };

  return std::make_shared<PrimitiveArrayObject> (name_,
                                                 description_ ? *description_ : name_,
                                                 key_,
                                                 *data_,
                                                 onUpdate_ ? *onUpdate_ : l,
                                                 *flatShading_);
}

