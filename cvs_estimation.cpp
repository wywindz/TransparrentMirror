#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

#include "cvs_estimation.h"
#include "edge_detector.h"

namespace radi
{
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

  void CVSEstimation::esimate(std::vector<CVSFeature> & cvs_feature_list)
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

    // ToDo: May refine corners to reduce the computation, remove extra corners which are
    // too close with each other.

    // Detect the CVS features.
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.01);
    octree.setInputCloud (point_cloud_);
    octree.addPointsFromInputCloud ();
    for (std::size_t idx_corner = 0; idx_corner < corners->size(); ++idx_corner)
    {
      // Extract the neighborhoods of these corners, in which edge detection will be performed.
      pcl::PointXYZ corner ((*corners).points[idx_corner].x, (*corners).points[idx_corner].y,
              (*corners).points[idx_corner].z);
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      octree.radiusSearch (corner, 0.2, neighbor_indices, neighbor_distances);

      pcl::ExtractIndices<pcl::PointXYZ> extractor;
      extractor.setInputCloud (point_cloud_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr neighborhood (new pcl::PointCloud<pcl::PointXYZ> ());
      // Need to convert the indices from std::vector<int> to boost::shared_ptr<std::vector<int> >.
      extractor.setIndices (boost::make_shared<std::vector<int> > (neighbor_indices));
      extractor.filter (*neighborhood);

      // Detect the edges in the neighborhood of the keypoint.
      std::vector<int> edge_point_indices;
      radi::EdgeDetector edge_detector;
      edge_detector.setInputCloud(neighborhood);
      edge_detector.compute(edge_point_indices);

      // Classify the points.
      Eigen::Vector3f pos_corner (corner.x, corner.y, corner.z);
      std::vector<std::vector<int> > edge_candidates;
      std::vector<Eigen::Vector3f> direction_references;
      for (std::size_t idx_edge = 0; idx_edge < edge_point_indices.size(); ++idx_edge)
      {
        Eigen::Vector3f pos_edge_point ((*neighborhood).points[edge_point_indices[idx_edge]].x,
                (*neighborhood).points[edge_point_indices[idx_edge]].y,
                (*neighborhood).points[edge_point_indices[idx_edge]].z);
        Eigen::Vector3f vect_direction = pos_edge_point - pos_corner;
        vect_direction /= std::sqrt(vect_direction.dot(vect_direction));

        // std::cout << "Direction: " << vect_direction[0] << "  " << vect_direction[1] << "  " << vect_direction[2] << std::endl;

        if (edge_candidates.empty())
        {
          edge_candidates.push_back (std::vector<int> ());
          edge_candidates[0].push_back (idx_edge);
          direction_references.push_back (vect_direction);
        }
        else
        {
          bool has_found = false;
          for (std::size_t idx_direction = 0; idx_direction < direction_references.size(); ++idx_direction)
          {
            if (1.0-vect_direction.dot(direction_references[idx_direction]) < 0.1)
            {
              has_found = true;
              edge_candidates[idx_direction].push_back (idx_edge);
              break;
            }
          }

          if (!has_found)
          {
            edge_candidates.push_back (std::vector<int> ());
            edge_candidates[edge_candidates.size()-1].push_back (idx_edge);
            direction_references.push_back (vect_direction);
          }
        }
      }

      // Construct cvs feature.
      CVSFeature cvs_feature;
      cvs_feature.setCorner(corner);
      // Number of the points in one edge should be larger than 5.
      for (std::size_t idx_feature = 0; idx_feature < edge_candidates.size(); ++idx_feature)
      {
        if (edge_candidates[idx_feature].size() >=5)
        {
          cvs_feature.appendVector(direction_references[idx_feature]);
        }
      }

      cvs_feature.compute();

      const std::vector<float> & angles = cvs_feature.getIncludedAngles();
      std::cout << angles[0] << "  " << angles[1] << "  " << angles[2] << std::endl;

      std::cout << "Number of edges: " << cvs_feature.getNumEdges() << std::endl;

      cvs_feature_list.push_back(cvs_feature);
    }
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
