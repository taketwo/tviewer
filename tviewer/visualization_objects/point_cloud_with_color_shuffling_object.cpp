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

#include "../utils.h"
#include "point_cloud_with_color_shuffling_object.h"

bool
tviewer::PointCloudWithColorShufflingObject::execute (const pcl::visualization::KeyboardEvent& key_event)
{
  if (matchKeys (key_event, "C-" + key_))
  {
    shuffleColors ();
    return true;
  }
  return VisualizationObject::execute (key_event);
}

bool
tviewer::PointCloudWithColorShufflingObject::at_ (size_t index, boost::any& item) const
{
  if (data_ && index < data_->size ())
  {
    pcl::PointXYZRGBA pt = data_->at (index);
    pt.rgba = color_map_.at (pt.rgba);
    item = pt;
    return true;
  }
  return false;
}

void
tviewer::PointCloudWithColorShufflingObject::updateData ()
{
  data_ = retrieve_ ();
  generateColorMap ();
}

void
tviewer::PointCloudWithColorShufflingObject::generateColorMap ()
{
  color_map_.clear ();

  if (!data_ || !data_->size ())
    return;

  for (const auto& point : data_->points)
    if (!color_map_.count (point.rgba))
      color_map_[point.rgba] = point.rgba;
}

void
tviewer::PointCloudWithColorShufflingObject::shuffleColors ()
{
  if (!data_ || !data_->size ())
    return;

  std::cout << name_ << ": shuffling colors\n";

  // Generate new random colors
  std::map<Color, Color> new_colors;
  for (const auto& current_original_pair : color_map_)
    new_colors[current_original_pair.first] = generateRandomColor ();

  // Apply new colors
  for (auto& point : data_->points)
    point.rgba = new_colors[point.rgba];

  // Update color map
  std::map<Color, Color> new_color_map;
  for (const auto& current_original_pair : color_map_)
    new_color_map.insert (std::make_pair (new_colors[current_original_pair.first], current_original_pair.second));
  color_map_.swap (new_color_map);

  refresh ();
}

tviewer::CreatePointCloudWithColorShufflingObject::operator std::shared_ptr<PointCloudWithColorShufflingObject> ()
{
  // Need to turn data_ into a local variable, otherwise the lambda does not
  // seem to capture it by value properly.
  auto d = *data_;
  auto l = [=] { return d; };

  return std::make_shared<PointCloudWithColorShufflingObject> (name_,
                                                               description_ ? *description_ : name_,
                                                               key_,
                                                               *data_,
                                                               onUpdate_ ? *onUpdate_ : l,
                                                               *pointSize_,
                                                               *visibility_);
}

