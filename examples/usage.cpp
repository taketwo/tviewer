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

#include <ctime>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/random.h>
#include <pcl/common/generate.h>
#include <pcl/features/normal_3d.h>

#include "tviewer/tviewer.h"

void
generateRandomPlanarCloud (pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  using namespace pcl::common;
  UniformGenerator<float>::Parameters x (-1.0f, 1.0f, time (0) + 0);
  UniformGenerator<float>::Parameters y (-1.0f, 1.0f, time (0) + 1);
  UniformGenerator<float>::Parameters z ( 1.0f, 1.05f, time (0) + 2);
  CloudGenerator<pcl::PointXYZ, UniformGenerator<float>> generator (x, y, z);
  cloud.clear ();
  generator.fill (2000, 1, cloud);
}

int main (int argc, char** argv)
{
  using namespace tviewer;
  auto viewer = create (argc, argv);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  generateRandomPlanarCloud (*cloud);

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZ> ("cloud", "c")
  . description                           ("Random point cloud")
  . data                                  (cloud)
  . pointSize                             (4)
  . visibility                            (0.9)
  . color                                 (generateRandomColor ())
  );

  viewer->add
  ( CreateNormalCloudObject ("normals", "n")
  . description             ("Normals cloud")
  . level                   (1)
  . scale                   (0.05)
  . onUpdate                ([&]
    {
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::PointCloud<pcl::PointNormal>::Ptr point_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setRadiusSearch (0.08);
      ne.setInputCloud (cloud);
      ne.compute (*normals);
      pcl::concatenateFields (*cloud, *normals, *point_normals);
      return point_normals;
    })
  );

  ArrowsPtr arrows (new Arrows);
  arrows->push_back ({ Eigen::Vector3f (1, 1, 1), Eigen::Vector3f (2, 2, 1), 0xFF0000 });
  arrows->push_back ({ Eigen::Vector3f (0, 1, 0), Eigen::Vector3f (0, 2, 0), 0x00FFFF });
  viewer->add
  ( CreateArrowArrayObject ("arrows", "a")
  . description            ("Arrows")
  . data                   (arrows)
  );

  viewer->update ();
  viewer->show ();

  while (viewer->waitKeyPressed ({"i"}))
  {
    generateRandomPlanarCloud (*cloud);
    viewer->update ();
  }

  return 0;
}

