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

#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <boost/none.hpp>
#include <boost/format.hpp>

#include <pcl/common/utils.h>
#include <pcl/console/print.h>

#include "tviewer.h"
#include "utils.h"

tviewer::TViewerImpl::TViewerImpl ()
: viewer_ (new pcl::visualization::PCLVisualizer ("T Viewer"))
, mode_waiting_user_input_ (false)
{
  viewer_->registerKeyboardCallback (boost::bind (&TViewerImpl::keyboardEventCallback, this, _1));
  viewer_->registerPointPickingCallback (boost::bind (&TViewerImpl::pickPointEventCallback, this, _1));
  loadCameraParameters ("viewpoint.cam");
}

tviewer::TViewerImpl::~TViewerImpl ()
{
  hide ();
}

void
tviewer::TViewerImpl::run ()
{
  while (!viewer_->wasStopped ())
  {
    viewer_->spinOnce (10);
  }
}

void
tviewer::TViewerImpl::sleep (size_t milliseconds)
{
  viewer_->spinOnce (10);
  std::this_thread::sleep_for (std::chrono::milliseconds (milliseconds));
}

void
tviewer::TViewerImpl::add (const VisualizationObject::Ptr& object, bool show, bool update)
{
  for (const auto& obj : objects_)
    if (obj->name_ == object->name_)
      assert (false); // TODO: throw exception
  objects_.push_back (object);
  object->injectViewer (viewer_);

  if (update)
    object->update ();

  bool force_show = force_show_when_added_.count (object->name_);
  bool force_hide = force_hide_when_added_.count (object->name_);
  if (!force_hide && (force_show || show))
    object->show ();
}

void
tviewer::TViewerImpl::remove (const std::string& object_name)
{
  for (auto iter = objects_.begin (); iter != objects_.end (); ++iter)
    if ((*iter)->name_ == object_name)
    {
      (*iter)->hide ();
      objects_.erase (iter);
      break;
    }
}

void
tviewer::TViewerImpl::show (const std::string& object_name)
{
  for (const auto& object : objects_)
    if (object->name_ == object_name)
      object->show ();
}

void
tviewer::TViewerImpl::show ()
{
  for (const auto& object : objects_)
    object->show ();
}

void
tviewer::TViewerImpl::hide (const std::string& object_name)
{
  for (const auto& object : objects_)
    if (object->name_ == object_name)
      object->hide ();
}

void
tviewer::TViewerImpl::hide ()
{
  for (const auto& object : objects_)
    object->hide ();
}

void
tviewer::TViewerImpl::update (const std::string& object_name)
{
  for (const auto& object : objects_)
    if (object->name_ == object_name)
      object->update ();
}

void
tviewer::TViewerImpl::update ()
{
  for (const auto& object : objects_)
    object->update ();
}

void
tviewer::TViewerImpl::addListener (const KeyboardListener::Ptr& listener,
                                   const std::vector<std::string>& dependent_objects)
{
  for (const auto& lst : listeners_)
    if (lst->getName () == listener->getName ())
      assert (false); // TODO: throw exception
  listeners_.push_back (listener);
  // Insert all the dependent object names in the corresponding dependents set
  auto& dependents = listener_dependents_[listener->getName ()];
  std::copy (dependent_objects.begin (), dependent_objects.end (),
             std::inserter (dependents, dependents.begin ()));
}

void
tviewer::TViewerImpl::removeListener (const std::string& listener_name)
{
  for (auto iter = listeners_.begin (); iter != listeners_.end (); ++iter)
    if ((*iter)->getName () == listener_name)
    {
      listeners_.erase (iter);
      break;
    }
}

void
tviewer::TViewerImpl::saveCameraParameters (const std::string& filename)
{
  std::vector<pcl::visualization::Camera> cameras;
  viewer_->getCameras (cameras);
  std::ofstream file (filename);
  if (file.is_open ())
  {
    const auto& clip = cameras[0].clip;
    const auto& focal = cameras[0].focal;
    const auto& pos = cameras[0].pos;
    const auto& view = cameras[0].view;
    const auto& fovy = cameras[0].fovy;
    const auto& win_size = cameras[0].window_size;
    const auto& win_pos = cameras[0].window_pos;
    file << clip[0]     << "," << clip[1]     << "/"
         << focal[0]    << "," << focal[1]    << "," << focal[2] << "/"
         << pos[0]      << "," << pos[1]      << "," << pos[2]   << "/"
         << view[0]     << "," << view[1]     << "," << view[2]  << "/"
         << fovy        << "/"
         << win_size[0] << "," << win_size[1] << "/"
         << win_pos[0]  << "," << win_pos[1]
         << endl;
    file.close ();
  }
  pcl::console::print_info ("Saved camera parameters to: %s\n", filename.c_str ());
}

void
tviewer::TViewerImpl::loadCameraParameters (const std::string& filename)
{
  int argc = 3;
  struct stat buffer;
  if (stat (filename.c_str (), &buffer) == 0)
  {
    const char* argv[] = {"dummy", "-cam", filename.c_str ()};
    if (viewer_->getCameraParameters (argc, const_cast<char**> (argv)))
      pcl::console::print_info ("Loaded camera parameters from: %s\n", filename.c_str ());
  }
  else
  {
    pcl::console::print_warn ("Failed to load camera parameters from: %s "
                              "(file not accessible)\n", filename.c_str ());
  }
}

bool
tviewer::TViewerImpl::askYesNo (const std::string& question, bool no_with_any_key)
{
  mode_waiting_user_input_ = true;
  pcl::console::print_info (question.c_str ());
  pcl::console::print_value (" (y/n) ");
  std::cout << std::flush;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyUp ())
    {
      std::string key = last_keyboard_event_->getKeySym ();
      last_keyboard_event_ = boost::none;
      if (key == "y" || key == "Y")
      {
        pcl::console::print_error ("YES\n");
        return true;
      }
      if (no_with_any_key || key == "n" || key == "N")
      {
        pcl::console::print_error ("NO\n");
        return false;
      }
    }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitKeyPressed ()
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
    {
      last_keyboard_event_ = boost::none;
      return true;
    }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitKeyPressed (std::string& key)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
    {
      key = last_keyboard_event_->getKeySym ();
      last_keyboard_event_ = boost::none;
      return true;
    }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitKeyPressed (const std::vector<std::string>& keys)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
      for (const auto& k : keys)
        if (matchKeys (*last_keyboard_event_, k))
        {
          last_keyboard_event_ = boost::none;
          return true;
        }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitKeyPressed (std::string& key, const std::vector<std::string>& keys)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_keyboard_event_ && last_keyboard_event_->keyDown ())
      for (const auto& k : keys)
        if (matchKeys (*last_keyboard_event_, k))
        {
          key = k;
          last_keyboard_event_ = boost::none;
          return true;
        }
    last_keyboard_event_ = boost::none;
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitPointSelected (size_t& point_index, float& x, float& y, float& z)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_point_picking_event_)
    {
      point_index = last_point_picking_event_->getPointIndex ();
      last_point_picking_event_->getPoint (x, y, z);
      last_point_picking_event_ = boost::none;
      return true;
    }
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitPointSelected (size_t& point_index)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_point_picking_event_)
    {
      point_index = last_point_picking_event_->getPointIndex ();
      last_point_picking_event_ = boost::none;
      return true;
    }
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitPointSelected (Color& point_color)
{
  mode_waiting_user_input_ = true;
  while (!viewer_->wasStopped () && mode_waiting_user_input_)
  {
    if (last_point_picking_event_)
    {
      size_t index;
      float x, y, z;
      index = last_point_picking_event_->getPointIndex ();
      last_point_picking_event_->getPoint (x, y, z);
      last_point_picking_event_ = boost::none;
      // Iterate over visualization objects and find which one owns the point
      for (const auto& object : objects_)
      {
        if (!object->visible_)
          continue;
        pcl::PointXYZRGBA pt;
        if (object->at (index, pt))  // will return false if object is not a point cloud
        {
          using namespace pcl::utils;
          if (equal (x, pt.x) && equal (y, pt.y) && equal (z, pt.z))
          {
            point_color = pt.rgba;
            return true;
          }
        }
      }
    }
    viewer_->spinOnce (100);
  }
  return false;
}

bool
tviewer::TViewerImpl::waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud,
                                          std::vector<pcl::PointIndices>& indices,
                                          bool skip_duplicates)
{
  cloud.clear ();
  indices.clear ();

  // Set up point cloud display
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr all_points (new pcl::PointCloud<pcl::PointXYZRGBA>);
  const std::string id = "picked points";

  add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> (id, "u")
  . description                               ("User selected points")
  . data                                      (all_points)
  . pointSize                                 (8)
  . visibility                                (0.9)
  );

  show (id);

  pcl::console::print_info ("Please select points:\n");

  // User input
  size_t label = 0;
  while (true)
  {
    size_t num_points = 0;
    pcl::PointIndices indexes;
    Color color = generateRandomColor ();
    mode_waiting_user_input_ = true;
    while (!viewer_->wasStopped () && mode_waiting_user_input_)
    {
      if (last_point_picking_event_)
      {
        pcl::PointXYZL pt;
        int index = last_point_picking_event_->getPointIndex ();
        last_point_picking_event_->getPoint (pt.x, pt.y, pt.z);
        last_point_picking_event_ = boost::none;
        if (skip_duplicates)
        {
          using namespace pcl::utils;
          bool duplicate = false;
          for (const auto& p : cloud)
          {
            if (equal (p.x, pt.x) && equal (p.y, pt.y) && equal (p.z, pt.z))
            {
              duplicate = true;
              break;
            }
          }
          if (duplicate)
            continue;
        }
        pt.label = label;
        cloud.push_back (pt);
        indexes.indices.push_back (index);
        ++num_points;
        // Visualize the selected point
        pcl::PointXYZRGBA vpoint;
        vpoint.x = pt.x;
        vpoint.y = pt.y;
        vpoint.z = pt.z;
        vpoint.rgba = color;
        all_points->push_back (vpoint);
        update (id);
      }
      viewer_->spinOnce (100);
    }
    // If the user did not input any points and pressed "Escape", which means
    // it is time to shut things down
    if (!num_points)
      break;
    pcl::console::print_info ("  [%zu] %zu points\n", label, num_points);
    // Now waiting for the next group of points
    ++label;
    indices.push_back (indexes);
  }
  remove (id);
  return true;
}

bool
tviewer::TViewerImpl::waitPointsSelected (pcl::PointCloud<pcl::PointXYZL>& cloud, bool skip_duplicates)
{
  std::vector<pcl::PointIndices> indices;
  return waitPointsSelected (cloud, indices, skip_duplicates);
}

bool
tviewer::TViewerImpl::waitPointsSelected (std::vector<pcl::PointIndices>& indices, bool skip_duplicates)
{
  pcl::PointCloud<pcl::PointXYZL> cloud;
  return waitPointsSelected (cloud, indices, skip_duplicates);
}

void
tviewer::TViewerImpl::setBackgroundColor (Color color)
{
  float r, g, b;
  std::tie (r, g, b) = getRGBFromColor (color);
  viewer_->setBackgroundColor (r, g, b);
}

void
tviewer::TViewerImpl::keyboardEventCallback (const pcl::visualization::KeyboardEvent& event)
{
  last_keyboard_event_ = event;
  if (event.keyDown ())
  {
    // save camera parameters
    if (matchKeys (event, "z"))
      saveCameraParameters ("viewpoint.cam");
    // load camera parameters
    if (matchKeys (event, "Z"))
      loadCameraParameters ("viewpoint.cam");
    // print (H)elp
    if (matchKeys (event, "h"))
      printHelp ();
    dispatch (event);
    if (matchKeys (event, "Escape"))
      mode_waiting_user_input_ = false;
  }
}

void
tviewer::TViewerImpl::pickPointEventCallback (const pcl::visualization::PointPickingEvent& event)
{
  last_point_picking_event_ = event;
}

void
tviewer::TViewerImpl::dispatch (const pcl::visualization::KeyboardEvent& key_event)
{
  for (const auto& object : objects_)
    object->execute (key_event);
  for (const auto& listener : listeners_)
    if (listener->execute (key_event))
      for (const auto& dependent : listener_dependents_[listener->getName ()])
        update (dependent);
}

void
tviewer::TViewerImpl::printHelp () const
{
  boost::format fmt_header ("\n%=60s"
                            "\n─────┬╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌┬──────\n");
  boost::format fmt_footer ("─────┴╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌┴──────\n");
  boost::format fmt_info ("%s│ %-46s │ ");
  boost::format fmt_extra ("     │ %-46s │\n");

  // Print the list of visualization objects
  if (objects_.size())
  {
    std::cout << fmt_header % "Visualization objects";
    for (size_t i = 0; i < objects_.size (); ++i)
    {
      std::cout << fmt_info % (objects_[i]->visible_ ? "  ☒  " : "  ☐  ") % objects_[i]->description_;
      printWithHighlight (objects_[i]->key_, isalpha);
      std::cout << std::endl;
    }
    std::cout << fmt_footer;
  }

  // Print the list of keyboard listeners
  if (listeners_.size())
  {
    std::cout << fmt_header % "Keyboard listeners";
    for (size_t i = 0; i < listeners_.size (); ++i)
    {
      std::string diagram, description, keys, extra;
      listeners_[i]->getInfo (diagram, description, keys, extra);
      std::cout << fmt_info % diagram % description;
      printWithHighlight (keys, isalpha);
      std::cout << std::endl;
      if (!extra.empty ())
        std::cout << fmt_extra % extra;
    }
    std::cout << fmt_footer;
  }
  //boost::format lfmt2 ("   ⦿      │ %s\n");
  //out += "\nPress Ctrl+<key> to shuffle colors";
  //out += "\nPress z/Z to save/load camera viewpoint\n";
}

void
tviewer::TViewerImpl::printWithHighlight (const std::string& str, std::function<bool (char)> highlight)
{
  for (size_t i = 0; i < str.size (); ++i)
  {
    if (highlight (str[i]))
      pcl::console::change_text_color (stdout, pcl::console::TT_RESET, pcl::console::TT_MAGENTA);
    std::cout << str[i];
    if (highlight (str[i]))
      pcl::console::reset_text_color (stdout);
  }
}

