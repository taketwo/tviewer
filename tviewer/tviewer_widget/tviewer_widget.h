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

#pragma once

#include <map>
#include <memory>
#include <string>

#include <QMenu>
#include <QAction>
#include <QSignalMapper>
#include <QContextMenuEvent>
#include <QVTKWidget.h>

#include "../tviewer_fwd.h"

namespace Ui
{
  class TViewerWidget;
}

namespace detail
{
  class TViewerWidgetImpl;
}

class TViewerWidget : public QVTKWidget
{

    Q_OBJECT

  public:

    explicit TViewerWidget (QWidget* parent = 0);

    ~TViewerWidget ();

    /** \name TViewer interface
      *
      * Functions in this group (partially) emulate TViewerInterface. */

    ///@{

    /** Register a new visualization object. */
    void
    add (tviewer::VisualizationObjectPtr object, bool show = false, bool update = false);

    /** Unregister a visualization object. */
    void
    remove (const std::string& object_name);

    /** Update a given visualization object. */
    void
    update (const std::string& object_name);

    /** Show a given visualization object. */
    void
    show (const std::string& object_name);

    /** Hide a given visualization object. */
    void
    hide (const std::string& object_name);

    ///@}

    /** Get a menu that contains actions to toggle visibility of visualization
      * objects that are registered in the widget.
      *
      * A non-owning pointer is returned; the memory is managed by the widget.
      * The menu title is set to "TViewer". An overload is available that
      * assigns a custom title. */
    QMenu*
    getMenu () const;

    /** Get a menu that contains actions to toggle visibility of visualization
      * objects that are registered in the widget.
      *
      * A non-owning pointer is returned; the memory is managed by the widget.
      *
      * \param[in] title custom menu title */
    QMenu*
    getMenu (const std::string& title);

    /** Receive widget context menu events. */
    virtual void
    contextMenuEvent (QContextMenuEvent* event) override;

  public Q_SLOTS:

    void
    onVisibilityActionToggled (const QString& name);

    void
    onSaveViewpointActionTriggered ();

    void
    onLoadViewpointActionTriggered ();

  Q_SIGNALS:

    void
    pointPicked (QString name, int index);

  private:

    Ui::TViewerWidget* ui_;

    std::unique_ptr<detail::TViewerWidgetImpl> tviewer_;

    std::unique_ptr<QMenu> menu_;
    std::unique_ptr<QSignalMapper> action_signal_mapper_;
    std::map<std::string, std::unique_ptr<QAction>> actions_;

    std::unique_ptr<QAction> save_viewpoint_action_;
    std::unique_ptr<QAction> load_viewpoint_action_;

};

