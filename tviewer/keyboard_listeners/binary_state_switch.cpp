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

#include "binary_state_switch.h"
#include "../tviewer_interface.h"
#include "../utils.h"

tviewer::BinaryStateSwitch::BinaryStateSwitch (const std::string& name,
                                               const std::string& description,
                                               const std::string& key,
                                               const OnChangeCallback on_change_callback,
                                               const StateNames& state_names,
                                               bool print_on_change,
                                               bool init)
: KeyboardListener (name)
, description_ (description)
, key_ (key)
, on_change_callback_ (on_change_callback)
, state_names_ (state_names)
, print_on_change_ (print_on_change)
, state_ (init)
{
}

bool
tviewer::BinaryStateSwitch::execute (const pcl::visualization::KeyboardEvent& key_event)
{
  if (matchKeys (key_event, key_))
  {
    flip ();
    return true;
  }
  return false;
}

void
tviewer::BinaryStateSwitch::getInfo (std::string& diagram,
                                     std::string& description,
                                     std::string& keys,
                                     std::string&)
{
  diagram = state_ ? "  ☒  " : "  ☐  ";
  keys = key_;
  description = description_ + " (" + state_names_[0] + "/" + state_names_[1] + ")";
}

void
tviewer::BinaryStateSwitch::set (bool value)
{
  if (value != state_)
  {
    state_ = value;
    on_change_callback_ (state_);
    if (print_on_change_)
    {
      pcl::console::print_info ("%s: ", description_.c_str ());
      pcl::console::print_value ("%s\n", state_names_[!state_].c_str ());
    }
  }
}

void
tviewer::BinaryStateSwitch::flip ()
{
  set (!state_);
}

tviewer::BinaryStateSwitch::Ptr
tviewer::CreateBinaryStateSwitch::addToViewer (TViewerPtr viewer, std::initializer_list<std::string> dependent_objects)
{
  auto instance = this->operator BinaryStateSwitch::Ptr ();
  viewer->addListener (instance, dependent_objects);
  return instance;
}

