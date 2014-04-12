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

#ifndef TVIEWER_POLY_DATA_OBJECT_H
#define TVIEWER_POLY_DATA_OBJECT_H

/** \file poly_data_object.h
  * Visualization object for polygonal data */

#include "visualization_object.h"

namespace tviewer
{

  /** Visualization object that displays a VTK poly data.
    *
    * \ingroup public */
  class PolyDataObject : public VisualizationObject
  {

    public:

      /// Poly data.
      typedef vtkPolyData PolyData;

      /// Shared pointer to poly data.
      typedef vtkSmartPointer<PolyData> PolyDataPtr;

      /// Function that retrieves data for visualization.
      typedef std::function<PolyDataPtr ()> RetrieveFunction;

      /** Construct a visualization object using a pointer to data.
        *
        * \param[in] name unique name (identifier) for the object
        * \param[in] description short description of the object that will be
        * displayed in help
        * \param[in] key key used to show/hide the object
        * \param[in] data poly data to display
        * \param[in] retrieve function that returns poly data that is to be
        * displayed */
      PolyDataObject (const std::string& name,
                      const std::string& description,
                      const std::string& key,
                      const PolyDataPtr& data,
                      const RetrieveFunction& retrieve)
      : VisualizationObject (name, description, key)
      , data_ (data)
      , retrieve_ (retrieve)
      {
      }

    protected:

      virtual void
      addDataToVisualizer (pcl::visualization::PCLVisualizer& v) override
      {
        v.addModelFromPolyData (data_, name_);
      }

      virtual void
      removeDataFromVisualizer (pcl::visualization::PCLVisualizer& v) override
      {
        v.removeShape (name_);
      }

      virtual void
      updateData () override
      {
        data_ = retrieve_ ();
      }

    private:

      /// Data for visualization.
      PolyDataPtr data_;

      /// Function which retrieves new data for visualization.
      RetrieveFunction retrieve_;

  };

  class CreatePolyDataObject
  {

    private:

      std::string name_;
      std::string key_;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreatePolyDataObject

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (PolyDataObject::PolyDataPtr, data, PolyDataObject::PolyDataPtr::New ());
      NAMED_PARAMETER (PolyDataObject::RetrieveFunction, onUpdate);

#include "../named_parameters/named_parameters_undef.h"

    public:

      CreatePolyDataObject (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      operator std::shared_ptr<PolyDataObject> ();

      inline operator std::shared_ptr<VisualizationObject> ()
      {
        return this->operator std::shared_ptr<PolyDataObject> ();
      }

  };

}

#endif /* TVIEWER_POLY_DATA_OBJECT_H */

