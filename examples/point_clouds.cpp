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

#include <ctime>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>
#include <pcl/features/normal_3d.h>

#include "tviewer/tviewer.h"

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
generatePlanarPointCloud (const Eigen::Vector3f& offset,
                          const Eigen::Vector2i& dims,
                          float step,
                          std::function<void (PointT&)> func)
{
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT> (dims[0], dims[1]));
  for (int i = 0; i < dims[0]; ++i)
    for (int j = 0; j < dims[1]; ++j)
    {
      cloud->at (i, j).getVector3fMap () = offset + Eigen::Vector3f (i, j, 0) * step;
      func (cloud->at (i, j));
    }
  return cloud;
}

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
generateRandomPointCloud (const Eigen::Vector3f& offset = {0, 0, 0},
                          const Eigen::Vector3f& dims = {1, 1, 1},
                          size_t num_points = 1000)
{
  using Generator = pcl::common::UniformGenerator<float>;
  std::vector<Generator::Parameters> p;
  for (size_t i = 0; i < 3; ++i)
    p.emplace_back (offset[i], offset[i] + dims[i], time (0) + i);
  typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::common::CloudGenerator<PointT, Generator> (p[0], p[1], p[2]).fill (num_points, 1, *cloud);
  return cloud;
}

int main (int argc, char** argv)
{
  // Generate several point clouds with different fields
  auto cloud_xyz1 = generateRandomPointCloud<pcl::PointXYZ> ({0, 0, 0});
  auto cloud_xyz2 = generateRandomPointCloud<pcl::PointXYZ> ({1, 0, 0});
  auto cloud_xyz3 = generateRandomPointCloud<pcl::PointXYZ> ({2, 0, 0});

  auto cloud_xyzn = generatePlanarPointCloud<pcl::PointNormal> ({0, 1, 0.5}, {30, 40}, 0.03f, [](pcl::PointNormal& p) {
      p.getNormalVector3fMap () = Eigen::Vector3f (0, 0, 1);
      p.curvature = 0;
  });
  auto cloud_xyzi = generatePlanarPointCloud<pcl::PointXYZI> ({1, 1, 0.5}, {30, 40}, 0.03f, [](pcl::PointXYZI& p) {
      p.intensity = std::sin ((p.getVector3fMap () - Eigen::Vector3f (1, 1, 0)).norm ());
  });
  auto cloud_xyzl = generatePlanarPointCloud<pcl::PointXYZL> ({2, 1, 0.5}, {30, 40}, 0.03f, [](pcl::PointXYZL& p) {
      p.label = (std::sin (p.x + p.y) + 1.0) * 10;
  });
  auto cloud_xyzrgb = generatePlanarPointCloud<pcl::PointXYZRGB> ({3, 1, 0.5}, {30, 40}, 0.03f, [](pcl::PointXYZRGB& p) {
      p.rgba = pcl::getRandomColor ().rgba;
  });

  using namespace tviewer;

  // Create an instance of TViewer

  auto viewer = create (argc, argv);

  // Now let's add some point clouds to the viewer
  // The simplest way to add a point cloud is as follows:
  // viewer->add ("cloud_xyz", "a", cloud_xyz);
  // This creates and registers a point cloud visualization object named "cloud_xyz".
  // The visibility of this object can be toggled using the "a" key.
  // By default, this object will be shown. We could have changed this by changing the last argument to hidden.
  // The point cloud that we used does not have any interesting fields, so the the point cloud will be displayed with plain white color.

  // The previous function created a point cloud visualization object behind the scenes.
  // Now let's add the same point cloud, but this time explicitly create a visualization object add it to the viewer.

  // PointCloudObject::Ptr cloud_obj(new PointCloudObject ("cloud_xyz_2", "XYZ point cloud", "A", cloud_xyz, []{}, 2, 1.0f, pcl::getRandomColor ()));
  // viewer->add (cloud_obj, true, false);

  // This gave us full control over the created object, however we had to specify each and every parameter, including those that we don't need.

  // There is a more convenient way to create a visualization object using CreatePointCloudObject helper class.

  viewer->add                                                            // Creates and adds a point cloud object
  ( CreatePointCloudObject ("cloud_xyz", "C-a")                          // The only required arguments are object name and toggle key
                                                                         // Everything else is optional
  . description            ("XYZ point cloud (color: z coordinate)")     // Description for the help (the name will be used if not specified)
  . data                   (cloud_xyz3)                                  // Use this point cloud as a source
  . pointSize              (2)                                           // Initial point size
  . color                  ("z")                                         // Color according to z coordinate
  );

  // Now let's add all other point clouds using different options

  viewer->add                                                            // Creates and adds a point cloud object
  ( CreatePointCloudObject ("cloud_xyzn", "a")                           // Name and a short-cut
  . description            ("XYZNormal point cloud (color: random)")     // Optional description for the help
  . data                   (cloud_xyzn)                                  // Use this point cloud as a source
  . pointSize              (2)                                           // Initial point size
  . color                  (pcl::getRandomColor ())                      // Color
  );

  // viewer->add
  // ( CreatePointCloudObject ("cloud_xyzn", "a")
  // . description            ("XYZNormal point cloud (color: random)")
  // . data                   (cloud_xyzn)
  // . pointSize              (2)
  // . color                  (pcl::getRandomColor ())
  // );

  viewer->add
  ( CreatePointCloudObject ("cloud_xyzi", "i")
  . description            ("XYZI point cloud (color: intensity field)")
  . data                   (cloud_xyzi)
  . pointSize              (4)
  . color                  ("intensity")
  );

  viewer->add
  ( CreatePointCloudObject ("cloud_xyzl", "b")
  . description            ("XYZL point cloud (color: label field)")
  . data                   (cloud_xyzl)
  . pointSize              (4)
  . color                  ("label")
  );

  viewer->add
  ( CreatePointCloudObject ("cloud_xyzrgb", "B")
  . description            ("XYZRGB point cloud (color: rgb field)")
  . data                   (cloud_xyzrgb)
  . pointSize              (4)
  . visibility             (0.8)
  . color                  ("foobar")
  );

  viewer->update ();

  // Show all added visualization objects
  viewer->show ();

  while (viewer->waitKeyPressed ({"space"}))
  {
    if (cloud_xyzl->size () > 1)
      cloud_xyzl->erase (cloud_xyzl->end () - 1);
    viewer->update ();
  }

  return 0;
}

