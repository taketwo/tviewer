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

#ifndef TVIEWER_UP_DOWN_COUNTER_H
#define TVIEWER_UP_DOWN_COUNTER_H

/** \file up_down_counter.h
  * Up-down counting keyboard listener */

#include "keyboard_listener.h"

namespace tviewer
{

  /** A keyboard listener which maintains a counter that may be increased and
    * decreased by a fixed step amount.
    *
    * \ingroup public */
  template <typename T>
  class UpDownCounter : public KeyboardListener
  {

    public:

      /// Shared pointer to an up-down counter.
      typedef std::shared_ptr<UpDownCounter> Ptr;

      /// Function to be called when counter value changes.
      typedef std::function<void (T)> OnChangeCallback;

      /** Construct an up-down counter.
        *
        * \param[in] name unique name (identifier) for the keyboard listener
        * \param[in] description short description of the counter that will be
        * displayed in help
        * \param[in] key alpha character used to trigger counter update (only
        * the "up" direction, "down" will be automatically assigned to the
        * upper version of this character)
        * \param[in] on_change_callback function to be executed when the value
        * of the counter changes. The function should accept a single
        * parameter -- the new value of the counter
        * \param[in] print_on_change controls whether the name and the new
        * value of the conter should be printed when it changes
        * \param[in] wrap_around controls what happens after the counter hits
        * its lower/upper limit. If set to \c true then the counter will be
        * wrapped to the opposite limit, otherwise nothing happens
        * \param[in] init initial value of the counter
        * \param[in] step step size
        * \param[in] min minimum allowed value
        * \param[in] max maximum allowed value */
      UpDownCounter (const std::string& name,
                     const std::string& description,
                     const std::string& key,
                     const OnChangeCallback on_change_callback,
                     bool print_on_change,
                     bool wrap_around,
                     T init,
                     T step,
                     T min,
                     T max);

      /// Disabled copy constructor.
      UpDownCounter (const UpDownCounter<T>&) = delete;

      /// Disabled assignment operator.
      UpDownCounter<T>& operator= (const UpDownCounter<T>&) = delete;

      virtual bool
      execute (const pcl::visualization::KeyboardEvent& key_event) override;

      virtual void
      getInfo (std::string& diagram,
               std::string& description,
               std::string& keys,
               std::string& extra) override;

      /** Retrieve the current value of the counter. */
      inline operator T () const
      {
        return counter_;
      }

      /** Set new current value of the counter.
        *
        * If the new value is outside of the valid region (defined by \c min_
        * and \c max_), then the counter will be set to the corresponding
        * boundary or (if the wrap around option is enbled) to the opposite
        * boundary.
        *
        * This function also triggers "onChange" callback and prints the new
        * value (if enabled with \c print_on_change_). */
      void
      set (T value);

    private:

      std::string description_;
      std::string key_up_;
      std::string key_down_;
      T counter_;
      OnChangeCallback on_change_callback_;
      bool print_on_change_;
      bool wrap_around_;
      T init_;
      T step_;
      T min_;
      T max_;

  };

  /** Helper class that provides a fluent interface to simplify instantiation of
    * an up-down counter.
    *
    * \ingroup public */
  template <typename T>
  class CreateUpDownCounter
  {

    private:

      std::string name_;
      std::string key_;

#include "../named_parameters/named_parameters_def.h"
#define OWNER_TYPE CreateUpDownCounter<T>

      NAMED_PARAMETER (std::string, description);
      NAMED_PARAMETER (T, init);
      NAMED_PARAMETER (T, step, 1);
      NAMED_PARAMETER (T, min, 0);
      NAMED_PARAMETER (T, max, std::numeric_limits<T>::max ());
      NAMED_PARAMETER (typename UpDownCounter<T>::OnChangeCallback, onChange, ([](T){}));
      NAMED_PARAMETER (bool, printOnChange, false, true);
      NAMED_PARAMETER (bool, wrapAround, false, true);

#include "../named_parameters/named_parameters_undef.h"

    public:

      CreateUpDownCounter (const std::string& name, const std::string& key)
      : name_ (name)
      , key_ (key)
      {
      }

      operator std::shared_ptr<UpDownCounter<T>> ()
      {
        return std::make_shared<UpDownCounter<T>> (name_,
                                                   description_ ? *description_ : name_,
                                                   key_,
                                                   *onChange_,
                                                   *printOnChange_,
                                                   *wrapAround_,
                                                   init_ ? *init_ : *min_,
                                                   *step_,
                                                   *min_,
                                                   *max_);
      }

      operator std::shared_ptr<KeyboardListener> ()
      {
        return this->operator std::shared_ptr<UpDownCounter<T>> ();
      }

  };

}

#endif /* TVIEWER_UP_DOWN_COUNTER_H */

