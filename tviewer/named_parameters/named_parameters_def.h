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

#ifndef NAMED_PARAMETERS_DEF_H
#define NAMED_PARAMETERS_DEF_H

#include <boost/optional.hpp>
#include <boost/preprocessor/facilities/overload.hpp>

#endif /* NAMED_PARAMETERS_DEF_H */

#define __FIELD(TYPE, NAME)                                       \
  private:                                                        \
    boost::optional<TYPE> NAME ## _

#define __SETTER_2(TYPE, NAME)                                    \
  public:                                                         \
    inline OWNER_TYPE& NAME (TYPE NAME ## __)                     \
    {                                                             \
      NAME ## _ = NAME ## __;                                     \
      return *this;                                               \
    }

#define __SETTER_3(TYPE, NAME, DEFAULT)                           \
  public:                                                         \
    inline OWNER_TYPE& NAME (TYPE NAME ## __ = DEFAULT)           \
    {                                                             \
      NAME ## _ = NAME ## __;                                     \
      return *this;                                               \
    }

#define __SETTER(...)                                             \
  BOOST_PP_OVERLOAD(__SETTER_, __VA_ARGS__)(__VA_ARGS__)

#define __NAMED_PARAMETER_4(TYPE, NAME, DEFAULT, SETTER_DEFAULT)  \
  __SETTER(TYPE, NAME, SETTER_DEFAULT)                            \
  __FIELD(TYPE, NAME) = static_cast<TYPE> (DEFAULT)

#define __NAMED_PARAMETER_3(TYPE, NAME, DEFAULT)                  \
  __SETTER(TYPE, NAME)                                            \
  __FIELD(TYPE, NAME) = static_cast<TYPE> (DEFAULT)

#define __NAMED_PARAMETER_2(TYPE, NAME)                           \
  __SETTER(TYPE, NAME)                                            \
  __FIELD(TYPE, NAME)

#define NAMED_PARAMETER(...)                                      \
  BOOST_PP_OVERLOAD(__NAMED_PARAMETER_, __VA_ARGS__)(__VA_ARGS__)

