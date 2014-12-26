#include <vtkRenderWindow.h>

#include "../tviewer.h"
#include "tviewer_widget.h"
#include "ui_tviewer_widget.h"

namespace detail
{

  class TViewerWidgetImpl : public tviewer::TViewerImpl
  {

    public:

      TViewerWidgetImpl (QVTKWidget* parent)
      : tviewer::TViewerImpl (false)
      , qvtk_widget_ (parent)
      {
        parent->SetRenderWindow (viewer_->getRenderWindow ());
        viewer_->setupInteractor (parent->GetInteractor (),
                                  parent->GetRenderWindow ());
      }

      virtual ~TViewerWidgetImpl ()
      {
      }

      virtual void
      show (const std::string& object_name) override
      {
        TViewerImpl::show (object_name);
        qvtk_widget_->update ();
      }

      virtual void
      hide (const std::string& object_name) override
      {
        TViewerImpl::hide (object_name);
        qvtk_widget_->update ();
      }

      virtual void
      update (const std::string& object_name) override
      {
        TViewerImpl::update (object_name);
        qvtk_widget_->update ();
      }

    private:

      TViewerWidgetImpl () = delete;

      QVTKWidget* qvtk_widget_;

  };

}

TViewerWidget::TViewerWidget (QWidget* parent)
: QVTKWidget (parent)
, ui_ (new Ui::TViewerWidget)
{
  ui_->setupUi (this);
  tviewer_.reset (new detail::TViewerWidgetImpl (this));
  menu_.reset (new QMenu ("TViewer"));
  action_signal_mapper_.reset (new QSignalMapper (this));
  connect (action_signal_mapper_.get (),
           SIGNAL (mapped (const QString&)),
           this,
           SLOT (onMenuActionToggled (const QString&)));
}

TViewerWidget::~TViewerWidget ()
{
  tviewer_.reset ();
  delete ui_;
}

void
TViewerWidget::add (tviewer::VisualizationObjectPtr object, bool show, bool update)
{
  tviewer_->add (object, show, update);
  QString name (object->getName ().c_str ());
  auto& action = actions_[object->getName ()];
  action.reset (new QAction (QString ("Display ") + name, 0));
  action->setEnabled (true);
  action->setCheckable (true);
  action->setChecked (object->isVisible ());
  // TODO: key to shortcut conversion should take care of modifier keys
  action->setShortcut (QString (object->getKey ().c_str ()));
  connect (action.get (),
           SIGNAL (toggled (bool)),
           action_signal_mapper_.get (),
           SLOT (map ()));
  action_signal_mapper_->setMapping (action.get (), name);
  menu_->addAction (action.get ());
}

void
TViewerWidget::remove (const std::string& object_name)
{
  tviewer_->remove (object_name);
  actions_.erase (actions_.find (object_name));
}

void
TViewerWidget::update (const std::string& object_name)
{
  tviewer_->update (object_name);
}

void
TViewerWidget::show (const std::string& object_name)
{
  tviewer_->show (object_name);
  actions_[object_name]->setChecked (true);
}

void
TViewerWidget::hide (const std::string& object_name)
{
  tviewer_->hide (object_name);
  actions_[object_name]->setChecked (false);
}

QMenu*
TViewerWidget::getMenu () const
{
  return menu_.get ();
}

QMenu*
TViewerWidget::getMenu (const std::string& title)
{
  menu_->setTitle (QString (title.c_str ()));
  return menu_.get ();
}

void
TViewerWidget::contextMenuEvent (QContextMenuEvent* event)
{
  event->setAccepted (true);
  menu_->popup (event->globalPos ());
}

void
TViewerWidget::onMenuActionToggled (const QString& name)
{
  auto object_name = name.toStdString ();
  if (actions_[object_name]->isChecked ())
    tviewer_->show (object_name);
  else
    tviewer_->hide (object_name);
}

