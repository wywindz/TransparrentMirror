#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ccn_estimation.h"
#include "board_detector.h"

namespace radi
{

  void
  removeIndices (std::vector<int> & source_indices, const std::vector<int> & removed_indices);

  CCNEstimation::CCNEstimation() : min_radius_(0.1), max_radius_(1.0), search_radius_(0.2),
      threshold_position_(0.1), threshold_radius_(0.05), threshold_angle_(0.5), min_num_points_(20)
  { }

  CCNEstimation::~CCNEstimation()
  { }

  void
  CCNEstimation::setInputCloud (const PointCloudConstPtr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  void
  CCNEstimation::setRadiusLimits (float min_radius, float max_radius)
  {
    min_radius_ = min_radius;
    max_radius_ = max_radius;
  }

  void
  CCNEstimation::setSearchRadius (float search_radius)
  {
    search_radius_ = search_radius;
  }

  void
  CCNEstimation::setMinNumPoints (int min_num_points)
  {
    min_num_points_ = min_num_points;
  }

  void
  CCNEstimation::setIdentityThresholds(float position, float radius, float angle)
  {
    threshold_position_ = position;
    threshold_radius_ = radius;
    threshold_angle_ = angle;
  }

  void CCNEstimation::esimate(std::vector<CCNFeature> & ccn_feature_list)
  {
    // Extract the board points in the scene cloud.
    std::vector<int> board_point_indices;
    radi::BoardDetector board_detector;
    board_detector.setInputCloud(point_cloud_);
    board_detector.compute(board_point_indices);

    indices_ = boost::make_shared<std::vector<int> >  (board_point_indices);

    std::cout << "Number of board points: " << board_point_indices.size() << std::endl;

    // Classify the board points.
    pcl::SACSegmentation<pcl::PointXYZ> sac_segment;
    sac_segment.setInputCloud(this->point_cloud_);
    sac_segment.setModelType(pcl::SACMODEL_CIRCLE3D);
    sac_segment.setMethodType(pcl::SAC_RANSAC);
    sac_segment.setMaxIterations(1000);
    sac_segment.setRadiusLimits(min_radius_, max_radius_);
    sac_segment.setDistanceThreshold(0.005);

    std::vector<pcl::ModelCoefficients> circle_list;
    while (board_point_indices.size () > min_num_points_)
    {
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      sac_segment.setIndices(boost::make_shared<std::vector<int> >(board_point_indices));

      // pcl::search::KdTree<pcl::PointXYZ>::Ptr search (new pcl::search::KdTree<pcl::PointXYZ>);
      // search->setInputCloud(this->point_cloud_);
      // sac_segment.setSamplesMaxDist(search_radius_, search);

      sac_segment.segment(*inliers, *coefficients);

      if (inliers->indices.size() >= min_num_points_)
      {
        std::cout << "Number of inliers: " << inliers->indices.size() << std::endl;
        std::cout << "Model coefficients: " << *coefficients << std::endl;
        if (~isInCircleList(*coefficients, circle_list))
        {
          circle_list.push_back(*coefficients);

          CCNFeature ccn_feature;
          ccn_feature.setCenter(Eigen::Vector3f((*coefficients).values[0], (*coefficients).values[1], (*coefficients).values[2]));
          ccn_feature.setRadius((*coefficients).values[3]);
          ccn_feature.setNormal(Eigen::Vector3f((*coefficients).values[4], (*coefficients).values[5], (*coefficients).values[6]));
          ccn_feature_list.push_back(ccn_feature);
        }

        // // Visualize points on a circle.
        // pcl::PointCloud<pcl::PointXYZ>::Ptr circle(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::ExtractIndices<pcl::PointXYZ> extracter;
        // extracter.setInputCloud(this->point_cloud_);
        // extracter.setIndices(inliers);
        // extracter.setNegative(false);
        // extracter.filter(*circle);
        // pcl::visualization::PCLVisualizer viewer("Board Points");
        // viewer.addPointCloud(this->point_cloud_);
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(circle, 0, 255, 0);
        // viewer.addPointCloud(circle, single_color, "Board points");
        // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Board points");
        // while (!viewer.wasStopped()) {
        //     viewer.spinOnce();
        // }
      }

      removeIndices(board_point_indices, inliers->indices);
    }

  }

  float
  CCNEstimation::getMinRadius ()
  {
    return (min_radius_);
  }

  std::size_t
  CCNEstimation::getMinNumPoints ()
  {
    return (min_num_points_);
  }

  const pcl::IndicesConstPtr
  CCNEstimation::getIndices ()
  {
    return (indices_);
  }

  bool
  CCNEstimation::isInCircleList(const pcl::ModelCoefficients & coefficients, const std::vector<pcl::ModelCoefficients> & circle_list)
  {
    Eigen::Vector3f pos_center (coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    float radius = coefficients.values[3];
    Eigen::Vector3f normal (coefficients.values[4], coefficients.values[5], coefficients.values[6]);

    for (int idx_circle = 0; idx_circle < circle_list.size(); ++idx_circle)
    {
      const pcl::ModelCoefficients & coeff_exist = circle_list[idx_circle];
      Eigen::Vector3f pos_center_exist (coeff_exist.values[0], coeff_exist.values[1], coeff_exist.values[2]);
      float radius_exist = coeff_exist.values[3];
      Eigen::Vector3f normal_exist (coeff_exist.values[4], coeff_exist.values[5], coeff_exist.values[6]);

      float err_position = std::abs(pos_center.norm() - pos_center_exist.norm());
      float err_radius = std::abs(radius - radius_exist);
      float err_angle = std::acos(std::abs(normal.dot(normal_exist)));

      if ((err_position < threshold_position_) && (err_radius < threshold_radius_) && (err_angle < threshold_angle_))
      {
        return true;
      }
    }

    return false;
  }

  void
  removeIndices (std::vector<int> & source_indices, const std::vector<int> & removed_indices)
  {
    std::vector<int> full_indices = source_indices;
    source_indices = std::vector<int> (full_indices.size() - removed_indices.size());
    int iCount = 0;
    for (int idx_full = 0; idx_full < full_indices.size(); ++idx_full)
    {
      bool flag_in = false;
      for (int idx_removed = 0; idx_removed < removed_indices.size(); ++idx_removed)
      {
        if (full_indices[idx_full] == removed_indices[idx_removed])
        {
          flag_in = true;
          break;
        }
      }

      if (!flag_in)
      {
        source_indices[iCount] = full_indices[idx_full];
        iCount++;
      }
    }
  }

} // namespace radi
