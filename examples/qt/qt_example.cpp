#include <random>
#include "qt_example.h"
#include "ui_qt_example.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

#include <tviewer/tviewer.h>

using namespace tviewer;

QtExample::QtExample (QWidget* parent)
: QMainWindow (parent)
, ui_ (new Ui::QtExample)
, dataset_ (new pcl::PointCloud<pcl::PointNormal>)
{
  ui_->setupUi (this);

  connect (ui_->viewer,
           SIGNAL (pointPicked (QString, int)),
           this,
           SLOT (onPointPicked (QString, int)));

  generateDataset ();

  ui_->viewer->add
  ( CreatePointCloudObject<pcl::PointNormal> ("dataset", "d")
  . description                              ("Generated dataset")
  . data                                     (dataset_)
  . pointSize                                (2)
  . visibility                               (1.0)
  . color                                    (generateRandomColor ())
  , true
  );

  ui_->viewer->add
  ( CreateNormalCloudObject<pcl::PointNormal> ("normals", "n")
  . description                               ("Generated dataset normals")
  . data                                      (dataset_)
  . level                                     (5)
  . scale                                     (2.5f)
  , false
  , true
  );

  ui_->viewer->add
  ( CreatePointCloudObject<pcl::PointXYZ> ("inliers", "c")
  . description                           ("Model inliers")
  . onUpdate                              ([this]()
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
      inliers.reset (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud (*dataset_, inliers_, *inliers);
      return inliers;
    })
  . pointSize                             (5)
  . visibility                            (1.0)
  . color                                 (generateRandomColor ())
  );

  ui_->menu_bar->addMenu (ui_->viewer->getMenu ("View"));
}

QtExample::~QtExample ()
{
  delete ui_;
}

void
QtExample::onDataGenerationSettingsChanged ()
{
  generateDataset ();
  inliers_.clear ();
  ui_->viewer->update ();
}

void
QtExample::onButtonFitClicked ()
{
  pcl::SampleConsensusModel<pcl::PointNormal>::Ptr model;

  if (ui_->radio_line->isChecked ())
  {
    model.reset (new pcl::SampleConsensusModelLine<pcl::PointNormal> (dataset_, true));
  }
  else if (ui_->radio_plane->isChecked ())
  {
    model.reset (new pcl::SampleConsensusModelPlane<pcl::PointNormal> (dataset_, true));
  }
  else if (ui_->radio_sphere->isChecked ())
  {
    model.reset (new pcl::SampleConsensusModelSphere<pcl::PointNormal> (dataset_, true));
    model->setRadiusLimits(1, 1000);
  }
  else if (ui_->radio_cylinder->isChecked ())
  {
    auto m = new pcl::SampleConsensusModelCylinder<pcl::PointNormal, pcl::PointNormal> (dataset_, true);
    m->setInputNormals (dataset_);
    model.reset (m);
    model->setRadiusLimits (1, 1000);
  }

  pcl::RandomSampleConsensus<pcl::PointNormal> ransac (model);
  ransac.setDistanceThreshold (ui_->spinbox_distance_threshold->value ());
  ransac.setProbability (ui_->spinbox_probability->value ());
  ransac.setMaxIterations (ui_->spinbox_max_iterations->value ());
  ransac.computeModel (1);
  ransac.getInliers (inliers_);
  Eigen::VectorXf model_coefficients;
  ransac.getModelCoefficients (model_coefficients);
  std::cout << "Model type: " << model->getClassName() << std::endl;
  std::cout << "Coefficients: " << model_coefficients.transpose () << std::endl;
  std::cout << "Number of inliers: " << inliers_.size () << std::endl;
  ui_->viewer->update ("inliers");
  ui_->viewer->show ("inliers");
}

void
QtExample::onPointPicked (const QString& name, int /* index */)
{
  std::cout << "Picked point from object: " << name.toStdString() << std::endl;
}

void
QtExample::generateDataset ()
{
  static std::random_device rnd;
  static std::mt19937 gen (rnd ());

  float size = 100.0;

  Eigen::Vector3f center (0, 0, 0);
  auto push_point = [&](const pcl::PointNormal& p)
  {
    dataset_->push_back (p);
    dataset_->back ().getVector3fMap () += center;
  };

  std::uniform_real_distribution<> o_dist (-size / 2, size / 2);
  auto generate_outlier = [&]()
  {
    pcl::PointNormal p;
    p.getVector3fMap () = Eigen::Vector3f (o_dist (gen), o_dist (gen), o_dist (gen));
    p.getNormalVector3fMap () = Eigen::Vector3f::Random ().normalized();
    return p;
  };

  size_t N = ui_->spinbox_ppm->value ();
  dataset_->clear ();

  std::cout << "Generating dataset" << std::endl;

  if (ui_->checkbox_line->checkState ())
  {
    // Generate model parameters
    std::uniform_real_distribution<> m_dist (0.0, M_PI / 2);
    float theta = m_dist (gen);
    float phi = m_dist (gen);
    // Create generator
    std::uniform_real_distribution<> p_dist (-size / 2, size / 2);
    Eigen::Vector3f v (std::sin (theta) * std::cos (phi), std::sin (theta) * std::sin (phi), std::cos (theta));
    auto generate_inlier = [&]()
    {
      pcl::PointNormal p;
      p.getVector3fMap () = v * p_dist (gen);
      p.getNormalVector3fMap () = (v.cross (Eigen::Vector3f::Random ())).normalized();
      return p;
    };
    // Generate inliers
    size_t inliers = ui_->spinbox_inlier_fraction->value () * N;
    for (size_t i = 0; i < inliers; ++i)
      push_point (generate_inlier ());
    // Generate outliers
    for (size_t i = 0; i < N - inliers; ++i)
      push_point (generate_outlier ());
    // Update center
    center[0] += size;
    std::cout << "* Line: theta (" << theta << "), phi (" << phi << ")" << std::endl;
  }

  if (ui_->checkbox_sphere->checkState ())
  {
    // Generate model parameters (radius)
    std::uniform_real_distribution<> m_dist (size / 5, size / 2);
    float r = m_dist (gen);
    // Create generator
    std::uniform_real_distribution<> p_dist (-1.0, 1.0);
    auto generate_inlier = [&]()
    {
      float x = 1.0, y = 1.0;
      while (x * x + y * y >= 1.0)
      {
        x = p_dist (gen);
        y = p_dist (gen);
      }
      float s = 2 * std::sqrt (1 - x * x - y * y) * r;
      pcl::PointNormal p;
      p.getVector3fMap () = Eigen::Vector3f (s * x, s * y, (1.0 - 2.0 * (x * x + y * y)) * r);
      p.getNormalVector3fMap () = p.getVector3fMap ().normalized();
      return p;
    };
    // Generate inliers
    size_t inliers = ui_->spinbox_inlier_fraction->value () * N;
    for (size_t i = 0; i < inliers; ++i)
      push_point (generate_inlier ());
    // Generate outliers
    for (size_t i = 0; i < N - inliers; ++i)
      push_point (generate_outlier ());
    // Update center
    center[0] += size;
    std::cout << "* Sphere: radius (" << r << ")" << std::endl;
  }

  if (ui_->checkbox_cylinder->checkState ())
  {
    // Generate model parameters (radius, height)
    std::uniform_real_distribution<> m_dist (size / 5, size / 2);
    float radius = m_dist (gen);
    float height = m_dist (gen);
    // Create generator
    std::uniform_real_distribution<> p_dist (-1.0, 1.0);
    auto generate_inlier = [&]()
    {
      auto a = p_dist (gen) * M_PI;
      auto x = std::sin (a) * radius;
      auto y = std::cos (a) * radius;
      auto z = p_dist (gen) * height / 2;
      pcl::PointNormal p;
      p.getVector3fMap () = Eigen::Vector3f (x, y, z);
      p.getNormalVector3fMap () = Eigen::Vector3f (x, y, 0).normalized();
      return p;
    };
    // Generate inliers
    size_t inliers = ui_->spinbox_inlier_fraction->value () * N;
    for (size_t i = 0; i < inliers; ++i)
      push_point (generate_inlier ());
    // Generate outliers
    for (size_t i = 0; i < N - inliers; ++i)
      push_point (generate_outlier ());
    // Update center
    center[0] += size;
    std::cout << "* Cylinder: radius (" << radius << "), height (" << height << ")" << std::endl;
  }
}

