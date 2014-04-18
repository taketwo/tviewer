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

#ifndef TVIEWER_COLOR_H
#define TVIEWER_COLOR_H

/** \file color.h
  * Functions to manipulate RGBA colors */

#include <tuple>
#include <cstdint>

namespace tviewer
{

  /// \addtogroup public
  /// @{

  /// RGBA color.
  typedef uint32_t Color;

  /// Color maps that associate a color to every float from [0..1].
  /// This mimics some of the standard MATLAB® color maps, see
  /// http://www.mathworks.de/de/help/matlab/ref/colormap.html.
  enum class ColorMap
  {
    /// Ranges from blue to red, and passes through the colors cyan, yellow,
    /// and orange
    JET,
    /// Consists of colors that are shades of green and yellow
    SUMMER,
    /// Varies smoothly from red, through orange, to yellow
    AUTUMN,
    /// Consists of colors that are shades of cyan and magenta
    COOL
  };

  /** Get a color from R, G, and B values (chars). */
  Color
  getColorFromRGB (uint8_t r, uint8_t g, uint8_t b);

  /** Get a color from R, G, and B values (floats).
    *
    * The input values are mapped from [0..1] to [0..255]. Input value that are
    * outside of the [0..1] region are clamped automatically. */
  Color
  getColorFromRGB (float r, float g, float b);

  /** Get a color from a string representation.
    *
    * Supported formats:
    *
    * * Sequence of digits representing an unsigned integer
    *
    *   Uses `std::stoul()` internally, therefore supports numbers given in
    *   octal or hexadecimal base.
    *
    *   Examples:
    *
    *     * <tt>"255"</tt> : blue
    *     * <tt>"0177400"</tt> : green
    *     * <tt>"0xFF0000"</tt> : red
    *
    * For invalid string representations black color will be returned. */
  Color
  getColorFromString (const std::string& color);

  /** Get R, G, and B values (floats) from a color. */
  std::tuple<float, float, float>
  getRGBFromColor (Color color);

  /** Get R, G, and B values (floats) as array from a color. */
  void
  getRGBFromColor (Color color, float* rgb);

  /** Get R, G, and B values (chars) as array from a color. */
  void
  getRGBFromColor (Color color, uint8_t* rgb);

  /** Generate a random color. */
  Color
  generateRandomColor ();

  /** Get the color for a given number in [0..1] using a given colormap.
    *
    * Input values that are outside of the [0..1] region are clamped
    * automatically.
    *
    * \see ColorMap */
  Color
  getColor (float value, ColorMap map = ColorMap::JET);

  /// @}

}

#endif /* TVIEWER_COLOR_H */

