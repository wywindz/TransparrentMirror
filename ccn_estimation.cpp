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

  CCNEstimation::CCNEstimation() : min_radius_(0.1), min_num_points_(20)
  { }

  CCNEstimation::~CCNEstimation()
  { }

  void
  CCNEstimation::setInputCloud (const PointCloudConstPtr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  void
  CCNEstimation::setMinRadius (float min_radius)
  {
    min_radius_ = min_radius;
  }

  void
  CCNEstimation::setMinNumPoints (std::size_t min_num_points)
  {
    min_num_points_ = min_num_points;
  }

  void CCNEstimation::esimate(std::vector<CCNFeature> & ccn_feature_list)
  {
    // Extract the board points in the scene cloud.
    std::vector<int> board_point_indices;
    radi::BoardDetector board_detector;
    board_detector.setInputCloud(point_cloud_);
    board_detector.compute(board_point_indices);

    std::cout << "Number of board points: " << board_point_indices.size() << std::endl;

    // Classify the board points.
    pcl::SACSegmentation<pcl::PointXYZ> sac_segment;
    sac_segment.setInputCloud(this->point_cloud_);
    sac_segment.setModelType(pcl::SACMODEL_CIRCLE3D);
    sac_segment.setMethodType(pcl::SAC_RANSAC);
    sac_segment.setMaxIterations(100);
    sac_segment.setDistanceThreshold(0.01);

    std::vector<pcl::ModelCoefficients> circle_list;
    std::vector<pcl::PointIndices> indices_list;
    while (board_point_indices.size () > 10)
    {
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      sac_segment.setIndices(boost::make_shared<std::vector<int> >(board_point_indices));

      // pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr oct_search (new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>);
      // // oct_search->setInputCloud(this->point_cloud_);
      // sac_segment.setSamplesMaxDist(0.1, oct_search);

      sac_segment.segment(*inliers, *coefficients);

      if (inliers->indices.size() >= 10)
      {
        std::cout << "Number of inliers: " << inliers->indices.size() << std::endl;
        circle_list.push_back(*coefficients);
        indices_list.push_back(*inliers);
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
