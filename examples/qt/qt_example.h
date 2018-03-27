#ifndef QT_EXAMPLE_H
#define QT_EXAMPLE_H

#include <QMainWindow>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tviewer/tviewer.h>

namespace Ui
{
  class QtExample;
}

class QtExample : public QMainWindow
{

    Q_OBJECT

  public:

    explicit QtExample (QWidget* parent = 0);

    ~QtExample ();

  public Q_SLOTS:

    void
    onDataGenerationSettingsChanged ();

    void
    onButtonFitClicked ();

    void
    onPointPicked (const QString& name, int index);

  private:

    void
    generateDataset ();

    Ui::QtExample* ui_;

    pcl::PointCloud<pcl::PointNormal>::Ptr dataset_;
    pcl::IndicesPtr inliers_;
    tviewer::PrimitivesPtr fitted_primitive_;

};

#endif // QT_EXAMPLE_H
