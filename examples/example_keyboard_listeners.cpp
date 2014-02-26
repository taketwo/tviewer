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
  auto viewer = create ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  generateRandomPlanarCloud (*cloud);

  viewer->add<PointCloudObject<pcl::PointXYZ>> (
    "cloud",
    "random point cloud",
    "c",
    [&] { generateRandomPlanarCloud (*cloud); return cloud; },
    4,
    1.0,
    generateRandomColor ()
  );

  viewer->addListener (
    CreateUpDownCounter<float> ("smoothing sigma", "s")
  . description                ("Smoothing sigma for mesh bilateral filter")
  . min                        (0)
  . max                        (2)
  . step                       (0.1)
  . printOnChange              ()
  . onChange                   ([](float v) { std::cout << "This is a callback!\n"; })
  , { "cloud" }
  );

  UpDownCounter<int>::Ptr foobar = CreateUpDownCounter<int> ("foobar", "f").printOnChange ();
  viewer->addListener (foobar);

  viewer->update ();
  viewer->show ();

  while (viewer->waitKeyPressed ({"i"}))
  {
    int k = *foobar;
    std::cout << "Foobar = " << k << std::endl;
    generateRandomPlanarCloud (*cloud);
    viewer->update ();
  }

  return 0;
}

