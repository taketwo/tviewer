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

#include <cmath>
#include <string>
#include <random>
#include <algorithm>
#include <stdexcept>

#include "color.h"

namespace tviewer
{

  template <typename T> const T&
  clamp (const T& value, const T& low, const T& high)
  {
    return (value < low ? low : (value > high ? high : value));
  }

  Color
  getColorFromRGB (uint8_t r, uint8_t g, uint8_t b)
  {
    return static_cast<uint32_t> (r) << 16 |
           static_cast<uint32_t> (g) <<  8 |
           static_cast<uint32_t> (b);
  }

  Color
  getColorFromRGB (float r, float g, float b)
  {
    return getColorFromRGB (static_cast<uint8_t> (std::round (clamp (r, 0.0f, 1.0f) * 255)),
                            static_cast<uint8_t> (std::round (clamp (g, 0.0f, 1.0f) * 255)),
                            static_cast<uint8_t> (std::round (clamp (b, 0.0f, 1.0f) * 255)));
  }

  Color
  getColorFromString (const std::string& color)
  {
    try
    {
      return std::stoul (color, 0, 0);
    }
    catch (std::invalid_argument& e)
    {
      return 0;
    }
  }

  std::tuple<float, float, float>
  getRGBFromColor (Color color)
  {
    float r = static_cast<float> (((color >> 16) & 0xFF)) / 255.0;
    float g = static_cast<float> (((color >>  8) & 0xFF)) / 255.0;
    float b = static_cast<float> (((color >>  0) & 0xFF)) / 255.0;
    return std::make_tuple (r, g, b);
  }

  void
  getRGBFromColor (Color color, float* rgb)
  {
    rgb[0] = static_cast<float> (((color >> 16) & 0xFF)) / 255.0;
    rgb[1] = static_cast<float> (((color >>  8) & 0xFF)) / 255.0;
    rgb[2] = static_cast<float> (((color >>  0) & 0xFF)) / 255.0;
  }

  void
  getRGBFromColor (Color color, uint8_t* rgb)
  {
    rgb[0] = (color >> 16) & 0xFF;
    rgb[1] = (color >>  8) & 0xFF;
    rgb[2] = (color >>  0) & 0xFF;
  }

  Color
  generateRandomColor ()
  {
    static std::random_device rnd;
    static std::mt19937 gen (rnd ());
    static std::uniform_int_distribution<> dist(0, 255);
    uint8_t r = static_cast<uint8_t> ((dist (gen) % 256));
    uint8_t g = static_cast<uint8_t> ((dist (gen) % 256));
    uint8_t b = static_cast<uint8_t> ((dist (gen) % 256));
    return getColorFromRGB (r, g, b);
  }

  Color
  getColor (float value, ColorMap colormap)
  {
    float v = clamp (value, 0.0f, 1.0f);
    switch (colormap)
    {
      case ColorMap::JET:
        {
          float four_value = v * 4;
          float r = std::min (four_value - 1.5, -four_value + 4.5);
          float g = std::min (four_value - 0.5, -four_value + 3.5);
          float b = std::min (four_value + 0.5, -four_value + 2.5);
          return getColorFromRGB (r, g, b);
        }
      case ColorMap::SUMMER:
        {
          return getColorFromRGB (v, (1.0f + v) * 0.5f, 0.4f);
        }
      case ColorMap::AUTUMN:
        {
          return getColorFromRGB (1.0f, v, 0.0f);
        }
      case ColorMap::COOL:
        {
          return getColorFromRGB (v, 1.0f - v, 1.0f);
        }
    }
    return generateRandomColor ();
  }

}
