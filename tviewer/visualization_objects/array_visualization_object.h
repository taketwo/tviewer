/******************************************************************************
 * Copyright (c) 2014, 2018 Sergey Alexandrov
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

#pragma once

/** \file visualization_objects/array_visualization_object.h
  * (Template) visualization object that manages an array of something */

#include "visualization_object.h"

namespace tviewer
{

  /// \addtogroup public
  /// @{

  /** Parameters of an array visualization object.
    * This struct should be specialized by for each deriving array class that needs parameters. */
  template <typename T>
  struct ArrayVisualizationObjectParameters
  {
  };

  /** Visualization object that displays an array of something. */
  template <typename T>
  class ArrayVisualizationObject : public VisualizationObject
  {

    public:

      using Data = std::vector<T>;

      using DataPtr = std::shared_ptr<Data>;

      /// Function that retrieves data for visualization.
      using RetrieveFunction = std::function<DataPtr ()>;

      template <typename ...Args>
      ArrayVisualizationObject (const std::string& name,
                                const std::string& description,
                                const std::string& key,
                                const DataPtr& data,
                                const RetrieveFunction& retrieve,
                                Args&&... args)
      : VisualizationObject (name, description, key)
      , data_ (data)
      , retrieve_ (retrieve)
      , params_ {std::forward<Args> (args)...}
      {
      }

    protected:

      void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) override;

      void
      updateData () override;

    private:

      inline std::string
      createId (size_t i) const;

      DataPtr data_;
      RetrieveFunction retrieve_;
      std::vector<std::string> objects_in_visualizer_;

      /// Additional visualization parameters, to be specialized by deriving classes.c
      ArrayVisualizationObjectParameters<T> params_;

  };

  /// @}

}

