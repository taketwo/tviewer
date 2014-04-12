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

#include <boost/format.hpp>

#include "../color.h"
#include "arrow_array_object.h"

inline std::string
tviewer::ArrowArrayObject::createId (size_t i) const
{
  return boost::str (boost::format ("%s%i") % name_ % i);
}

void
tviewer::ArrowArrayObject::addDataToVisualizer (pcl::visualization::PCLVisualizer& v)
{
  for (size_t i = 0; i < data_->size (); ++i)
  {
    pcl::PointXYZ p1, p2;
    auto& arrow = data_->at (i);
    p1.getVector3fMap () = invert_direction_ ? arrow.target : arrow.source;
    p2.getVector3fMap () = invert_direction_ ? arrow.source : arrow.target;
    float r, g, b;
    std::tie (r, g, b) = getRGBFromColor (arrow.color);
    arrows_in_visualizer_.push_back (createId (i));
    // Counter-intuitively, the arrow head is attached to the first point, so
    // we pass the target first.
    v.addArrow (p2, p1, r, g, b, false, arrows_in_visualizer_.back ());
  }
}

void
tviewer::ArrowArrayObject::removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v)
{
  for (const auto& id : arrows_in_visualizer_)
    v.removeShape (id);
  arrows_in_visualizer_.clear ();
}

void
tviewer::ArrowArrayObject::updateData ()
{
  data_ = retrieve_ ();
}

tviewer::CreateArrowArrayObject::operator std::shared_ptr<ArrowArrayObject> ()
{
  // Need to turn data_ into a local variable, otherwise the lambda does not
  // seem to capture it by value properly.
  auto d = *data_;
  auto l = [=] { return d; };

  return std::make_shared<ArrowArrayObject> (name_,
                                             description_ ? *description_ : name_,
                                             key_,
                                             *data_,
                                             onUpdate_ ? *onUpdate_ : l,
                                             *invertDirection_);
}

