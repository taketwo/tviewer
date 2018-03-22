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

#include <boost/format.hpp>

#include "array_visualization_object.h"

namespace tviewer
{

  namespace detail
  {

    /// Strategy traits that should be specialized for deriving classes.
    /// Should implement addition and removal from a visualizer.
    template <typename T>
    struct ArrayVisualizationObjectStrategy;

  }

}

template <typename T> inline std::string
tviewer::ArrayVisualizationObject<T>::createId (size_t i) const
{
  return boost::str (boost::format ("%s%i") % name_ % i);
}

template <typename T> void
tviewer::ArrayVisualizationObject<T>::addDataToVisualizer (pcl::visualization::PCLVisualizer& v)
{
  objects_in_visualizer_.resize (data_->size ());
  for (size_t i = 0; i < data_->size (); ++i)
  {
    objects_in_visualizer_[i] = createId (i);
    detail::ArrayVisualizationObjectStrategy<T>::add (v, data_->at (i), params_, objects_in_visualizer_[i]);
  }
}

template <typename T> void
tviewer::ArrayVisualizationObject<T>::removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v)
{
  for (const auto& id : objects_in_visualizer_)
    detail::ArrayVisualizationObjectStrategy<T>::remove (v, id);
  objects_in_visualizer_.clear ();
}

template <typename T> void
tviewer::ArrayVisualizationObject<T>::updateData ()
{
  data_ = retrieve_ ();
}

