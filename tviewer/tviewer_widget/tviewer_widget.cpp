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
      add (tviewer::VisualizationObjectPtr object, bool show = false, bool update = false) override
      {
        TViewerImpl::add (object, show, update);
        qvtk_widget_->update ();
      }

      virtual void
      remove (const std::string& object_name) override
      {
        TViewerImpl::remove (object_name);
        qvtk_widget_->update ();
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
, save_viewpoint_action_ (new QAction ("Save viewpoint", 0))
, load_viewpoint_action_ (new QAction ("Load viewpoint", 0))
{
  ui_->setupUi (this);
  tviewer_.reset (new detail::TViewerWidgetImpl (this));
  menu_.reset (new QMenu ("TViewer"));
  connect (save_viewpoint_action_.get (),
           SIGNAL (triggered ()),
           this,
           SLOT (onSaveViewpointActionTriggered ()));
  menu_->addAction (save_viewpoint_action_.get ());
  connect (load_viewpoint_action_.get (),
           SIGNAL (triggered ()),
           this,
           SLOT (onLoadViewpointActionTriggered ()));
  menu_->addAction (load_viewpoint_action_.get ());
  menu_->addSeparator ();
  action_signal_mapper_.reset (new QSignalMapper (this));
  connect (action_signal_mapper_.get (),
           SIGNAL (mapped (const QString&)),
           this,
           SLOT (onVisibilityActionToggled (const QString&)));
}

TViewerWidget::~TViewerWidget ()
{
  tviewer_.reset ();
  delete ui_;
}

void
TViewerWidget::add (tviewer::VisualizationObjectPtr object, bool show, bool update)
{
  remove (object->getName ());
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
  auto action = actions_.find (object_name);
  if (action != actions_.end ())
    actions_.erase (action);
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
TViewerWidget::onVisibilityActionToggled (const QString& name)
{
  auto object_name = name.toStdString ();
  if (actions_[object_name]->isChecked ())
    tviewer_->show (object_name);
  else
    tviewer_->hide (object_name);
}

void
TViewerWidget::onSaveViewpointActionTriggered ()
{
  tviewer_->saveCameraParameters ("viewpoint.cam");
}

void
TViewerWidget::onLoadViewpointActionTriggered ()
{
  if (boost::filesystem::exists ("viewpoint.cam"))
    tviewer_->loadCameraParameters ("viewpoint.cam");
}

