#ifndef TVIEWER_TVIEWER_WIDGET_H
#define TVIEWER_TVIEWER_WIDGET_H

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
    onMenuActionToggled (const QString& name);

  private:

    Ui::TViewerWidget* ui_;

    std::unique_ptr<detail::TViewerWidgetImpl> tviewer_;

    std::unique_ptr<QMenu> menu_;
    std::unique_ptr<QSignalMapper> action_signal_mapper_;
    std::map<std::string, std::unique_ptr<QAction>> actions_;

};

#endif // TVIEWER_TVIEWER_WIDGET_H
