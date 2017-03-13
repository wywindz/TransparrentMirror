#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

#include "cvs_estimation.h"
#include "edge_detector.h"

namespace radi {

  CVSEstimation::CVSEstimation() : radius_(0.1), distance_(0.2), num_edges_(2)
  { }

  CVSEstimation::~CVSEstimation()
  { }

  void CVSEstimation::setInputCloud(const PointCloudConstPtr & point_cloud)
  {
      point_cloud_ = point_cloud;
  }

  void CVSEstimation::setRadius(float radius)
  {
      radius_ = radius;
  }

  void CVSEstimation::setNumEdges(std::size_t numEdges)
  {
      num_edges_ = numEdges;
  }

  std::vector<CVSFeature> CVSEstimation::esimate()
  {
      // Find corners.
      // Note: The output data type of HarrisKeypoint3D should contain intensity.
      pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> corner_detector;
      pcl::PointCloud<pcl::PointXYZI>::Ptr corners (new pcl::PointCloud<pcl::PointXYZI> ());
      corner_detector.setInputCloud (point_cloud_);
      corner_detector.setNonMaxSupression (true);
      corner_detector.setRadius (0.1);
      corner_detector.setThreshold (1e-3);
      corner_detector.compute (*corners);

      // ToDo: May refine corners to reduce the computation, remove extra corners which are too close with each other.

      // Detect the CVS features.
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.01);
      octree.setInputCloud (point_cloud_);
      octree.addPointsFromInputCloud ();
      for (std::size_t idx_corner = 0; idx_corner < corners->size(); ++idx_corner) {
          // Extract the neighborhoods of these corners, in which edge detection will be performed.
          pcl::PointXYZ corner ((*corners).points[idx_corner].x, (*corners).points[idx_corner].y,
                                (*corners).points[idx_corner].z);
          std::vector<int> neighbor_indices;
          std::vector<float> neighbor_distances;
          octree.radiusSearch (corner, 0.2, neighbor_indices, neighbor_distances);

          pcl::ExtractIndices<pcl::PointXYZ> extractor;
          extractor.setInputCloud (point_cloud_);
          pcl::PointCloud<pcl::PointXYZ>::Ptr neiborhood (new pcl::PointCloud<pcl::PointXYZ> ());
          // Need to convert the indices from std::vector<int> to boost::shared_ptr<std::vector<int> >.
          extractor.setIndices (boost::make_shared<std::vector<int> > (neighbor_indices));
          extractor.filter (*neiborhood);

          std::cout << neiborhood->size() << std::endl;

          // ToDo:: Detect the edges in the neighborhood of the keypoint.
          std::vector<radi::Edge> edge_list;
          radi::EdgeDetector edge_detector;
          edge_detector.setInputCloud(neiborhood);
          edge_detector.compute(edge_list);
      }

      return std::vector<CVSFeature>();
  }

  float CVSEstimation::getRadius()
  {
      return (radius_);
  }

  std::size_t CVSEstimation::getMinNumEdges()
  {
      return (num_edges_);
  }

} // namespace radi
