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

#include "tviewer/tviewer.h"

using namespace tviewer;

template <typename PointT> void
generatePointCube (pcl::PointCloud<PointT>& cloud, int n, float step, bool random_colors)
{
  cloud.clear ();
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      for (int k = 0; k < n; ++k)
      {
        PointT point;
        point.x = i * step;
        point.y = j * step;
        point.z = k * step;
        point.rgba = random_colors ? generateRandomColor () : 0xFFF000;
        cloud.push_back (point);
      }
    }
  }
}

int main (int argc, char** argv)
{
  auto viewer = create (argc, argv);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cube (new pcl::PointCloud<pcl::PointXYZRGBA>);
  generatePointCube (*cube, 5, 0.1f, false);

  viewer->add
  ( CreatePointCloudObject<pcl::PointXYZRGBA> ("cube", "p")
  . description ("Point cube")
  . data (cube)
  . pointSize (5)
  . visibility (0.9)
  );

  BinaryStateSwitch::Ptr random_colors
  = CreateBinaryStateSwitch ("mode", "m")
  . description             ("Use random colors")
  . init                    (false)
  . printOnChange           ();

  viewer->addListener
  ( CreateUpDownCounter<int> ("side", "s")
  . description              ("Cube side length")
  . init                     (5)
  . min                      (2)
  . max                      (20)
  . step                     (1)
  . wrapAround               ()
  . onChange                 ([&](float v){ generatePointCube (*cube, v, 0.1f, *random_colors); })
  , { "cube" }
  );

  viewer->addListener (random_colors);

  viewer->show ("cube");

  return 0;
}

