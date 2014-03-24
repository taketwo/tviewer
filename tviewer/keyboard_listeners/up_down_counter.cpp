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

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "up_down_counter.h"
#include "../utils.h"

template <typename T>
tviewer::UpDownCounter<T>::UpDownCounter (const std::string& name,
                                          const std::string& description,
                                          const std::string& key,
                                          const OnChangeCallback on_change_callback,
                                          bool print_on_change,
                                          bool wrap_around,
                                          T init,
                                          T step,
                                          T min,
                                          T max)
: KeyboardListener (name)
, description_ (description)
, key_up_ (key)
, key_down_ (boost::to_upper_copy (key))
, counter_ (init)
, on_change_callback_ (on_change_callback)
, print_on_change_ (print_on_change)
, wrap_around_ (wrap_around)
, init_ (init)
, step_ (step)
, min_ (min)
, max_ (max)
{
}

template <typename T> bool
tviewer::UpDownCounter<T>::execute (const pcl::visualization::KeyboardEvent& key_event)
{
  if (matchKeys (key_event, key_up_))
  {
    set (counter_ + step_);
    return true;
  }
  else if (matchKeys (key_event, key_down_))
  {
    set (counter_ - step_);
    return true;
  }
  return false;
}

template <typename T> void
tviewer::UpDownCounter<T>::getInfo (std::string& diagram,
                                    std::string& description,
                                    std::string& keys,
                                    std::string& extra)
{
  diagram = "  ↑↓ ";
  keys = key_up_ + "↑ " + key_down_ + "↓";
  description = description_;
  extra = "Current: " + boost::lexical_cast<std::string> (counter_);
}

template <typename T> void
tviewer::UpDownCounter<T>::set (T value)
{
  if (value > max_)
  {
    counter_ = wrap_around_ ? min_ : max_;
  }
  else if (value < min_)
  {
    counter_ = wrap_around_ ? max_ : min_;
  }
  else
  {
    counter_ = value;
  }
  on_change_callback_ (counter_);
  if (print_on_change_)
  {
    pcl::console::print_info ("%s: ", description_.c_str ());
    pcl::console::print_value ("%s\n", boost::lexical_cast<std::string> (counter_).c_str ());
  }
}

template class tviewer::UpDownCounter<int>;
template class tviewer::UpDownCounter<uint32_t>;
template class tviewer::UpDownCounter<float>;
template class tviewer::UpDownCounter<double>;

