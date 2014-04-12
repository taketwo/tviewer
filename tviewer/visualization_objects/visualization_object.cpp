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

#include "visualization_object.h"
#include "../utils.h"

tviewer::VisualizationObject::VisualizationObject (const std::string& name,
                                                   const std::string& description,
                                                   const std::string& key)
: name_ (name)
, description_ (description)
, key_ (key)
{
}

void
tviewer::VisualizationObject::show ()
{
  if (!visible_)
  {
    if (auto v = viewer_.lock ())
    {
      this->addDataToVisualizer (*v);
      visible_ = true;
    }
  }
}

void
tviewer::VisualizationObject::hide ()
{
  if (visible_)
  {
    if (auto v = viewer_.lock ())
    {
      this->removeDataFromVisualizer (*v);
      visible_ = false;
    }
  }
}

void
tviewer::VisualizationObject::update ()
{
  this->updateData ();
  refresh ();
}

void
tviewer::VisualizationObject::refresh ()
{
  if (visible_)
    if (auto v = viewer_.lock ())
      this->refreshDataInVisualizer (*v);
}

void
tviewer::VisualizationObject::toggle ()
{
  visible_ ? hide () : show ();
  pcl::console::print_info (" %s  %s\n", (visible_ ? "◉" : "○"), name_.c_str ());
}

bool
tviewer::VisualizationObject::execute (const pcl::visualization::KeyboardEvent& key_event)
{
  if (matchKeys (key_event, key_))
  {
    toggle ();
    return true;
  }
  return false;
}

void
tviewer::VisualizationObject::refreshDataInVisualizer (pcl::visualization::PCLVisualizer& v)
{
  this->removeDataFromVisualizer (v);
  this->addDataToVisualizer (v);
}

void
tviewer::VisualizationObject::injectViewer (std::shared_ptr<pcl::visualization::PCLVisualizer>& viewer)
{
  viewer_ = viewer;
}

