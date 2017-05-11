#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>

#include "ccn_estimation.h"
#include "board_detector.h"

namespace radi
{
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

    // Classify the board points.

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

} // namespace radi
