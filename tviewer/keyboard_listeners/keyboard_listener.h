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

#ifndef TVIEWER_KEYBOARD_LISTENER_H
#define TVIEWER_KEYBOARD_LISTENER_H

/** \file keyboard_listener.h
  * Base class for keyboard listeners */

#include <string>
#include <memory>

#include <boost/format.hpp>

#include <pcl/visualization/pcl_visualizer.h>

namespace tviewer
{

  /** Purely abstract base class for keyboard listeners that may be registered
    * with a TViewer to drive user interactions.
    *
    * TViewer maintains a collection of keyboard listeners. When a keyboard
    * event occurs each of these listerens is given a chance to react to it.
    *
    * \ingroup private */
  class KeyboardListener
  {

    public:

      /// Shared pointer to a keyboard listener.
      typedef std::shared_ptr<KeyboardListener> Ptr;

      /** Construct a keyboard listener.
        *
        * \param[in] name unique name (identifier) for the object */
      KeyboardListener (const std::string& name)
      : name_ (name)
      {
      }

      /** Execute the command associated with this keyboard listener if a given
        * keyboard event matches.
        *
        * \return \c false if no command is associated and/or no action was
        * taken, \c true otherwise */
      virtual bool
      execute (const pcl::visualization::KeyboardEvent& key_event) = 0;

      /** Get the name (identifier) of this keyboard listener. */
      inline const std::string&
      getName ()
      {
        return name_;
      }

      /** Get information about this listener that is to be displayed in help.
        *
        * \param[out] diagram 5-character wide string with a digram which
        * reflects the state and/or the type of the listener
        * \param[out] description short description of the listener
        * \param[out] keys the keys on which this listener reacts
        * \param[out] extra additional line of information (optional) */
      virtual void
      getInfo (std::string& diagram,
               std::string& description,
               std::string& keys,
               std::string& extra) = 0;

    protected:

      /// Listener identifier.
      std::string name_;

  };

}

#endif /* TVIEWER_KEYBOARD_LISTENER_H */

