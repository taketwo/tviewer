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

#ifndef TVIEWER_BINARY_STATE_SWITCH_H
#define TVIEWER_BINARY_STATE_SWITCH_H

/** \file binary_state_switch.h
  * Binary state switching keyboard listener */

#include <array>

#include "../tviewer_fwd.h"
#include "keyboard_listener.h"

namespace tviewer
{

  /** A keyboard listener which maintains a binary state that may be flipped.
    *
    * \ingroup public */
  class BinaryStateSwitch : public KeyboardListener
  {

    public:

      /// Shared pointer to a binary state switch.
      typedef std::shared_ptr<BinaryStateSwitch> Ptr;

      /// Function to be called when the state changes.
      typedef std::function<void (bool)> OnChangeCallback;

      /// Names (descriptions) of the states.
      typedef std::array<std::string, 2> StateNames;

      /** Construct a binary state switch.
        *
        * \param[in] name unique name (identifier) for the keyboard listener
        * \param[in] description short description of the switch that will be
        * displayed in help
        * \param[in] key alpha character used to trigger the switch
        * \param[in] on_change_callback function to be executed when the state
        * changes. The function should accept a single parameter -- the new
        * state
        * \param[in] print_on_change controls whether the name and the new
        * state of the conter should be printed when it changes
        * \param[in] init initial state of the switch */
      BinaryStateSwitch (const std::string& name,
                         const std::string& description,
                         const std::string& key,
                         const OnChangeCallback on_change_callback,
                         const StateNames& state_names,
                         bool print_on_change,
                         bool init);

      /// Disabled copy constructor.
      BinaryStateSwitch (const BinaryStateSwitch&) = delete;

      /// Disabled assignment operator.
      BinaryStateSwitch& operator= (const BinaryStateSwitch&) = delete;

      virtual bool
      execute (const pcl::visualization::KeyboardEvent& key_event) override;

      virtual void
      getInfo (std::string& diagram,
               std::string& description,
               std::string& keys,
               std::string& extra) override;

      /** Retrieve the current state. */
      inline operator bool () const
      {
        return state_;
      }

      /** Set the state of the switch.
        *
        * This function also triggers "onChange" callback and prints the new
        * state (if enabled with \c print_on_change_). */
      void
      set (bool state);

      /** Flip the state of the switch.
        *
        * This function also triggers "onChange" callback and prints the new
        * state (if enabled with \c print_on_change_). */
      void
      flip ();

    private:

      std::string description_;
      std::string key_;
      OnChangeCallback on_change_callback_;
      StateNames state_names_;
      bool print_on_change_;
      bool state_;

  };

  /** Helper class that provides a fluent interface to simplify instantiation of
    * a binary state switch.
    *
    * \ingroup public */
  class CreateBinaryStateSwitch
  {

    private:

      std::string name_;
      std::string key_;

      const BinaryStateSwitch::StateNames YESNO = {{"yes", "no"}};

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreateBinaryStateSwitch

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (bool, init, false);
      NAMED_PARAMETER (bool, printOnChange, false, true);
      NAMED_PARAMETER (BinaryStateSwitch::StateNames, stateNames, YESNO);
      NAMED_PARAMETER (BinaryStateSwitch::OnChangeCallback, onChange, ([](bool){}));

#include "../named_parameters/named_parameters_undef.h"

    public:

      CreateBinaryStateSwitch (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      operator BinaryStateSwitch::Ptr ()
      {
        return std::make_shared<BinaryStateSwitch> (name_,
                                                    description_ ? *description_ : name_,
                                                    key_,
                                                    *onChange_,
                                                    *stateNames_,
                                                    *printOnChange_,
                                                    *init_);
      }

      operator std::shared_ptr<KeyboardListener> ()
      {
        return this->operator std::shared_ptr<BinaryStateSwitch> ();
      }

      BinaryStateSwitch::Ptr
      addToViewer (TViewerPtr viewer, std::initializer_list<std::string> dependent_objects = {});

  };

}

#endif /* TVIEWER_BINARY_STATE_SWITCH_H */

