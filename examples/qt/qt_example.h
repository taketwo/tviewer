#ifndef QT_EXAMPLE_H
#define QT_EXAMPLE_H

#include <QMainWindow>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr dataset_;
    std::vector<int> inliers_;

};

#endif // QT_EXAMPLE_H
