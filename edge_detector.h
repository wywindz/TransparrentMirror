/*
 * Define Corner Vector Sequence (CVS) Feature as well as the related classes for estimating the feature.
 *
 */

#ifndef MIRROR_EDGE_DETECTOR_H_
#define MIRROR_EDGE_DETECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace radi {

  class Edge
  {
    public:
      Eigen::Vector3f point_;
      Eigen::Vector3f orient_vector_;
  };

  class EdgeDetector
  {
    public:
      EdgeDetector ();
      ~EdgeDetector ();

      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      void
      setInputCloud (const PointCloudConstPtr & point_cloud);

      void
      setRadius (float radius);

      void
      setK (std::size_t k);

      // Obtain the indices of the points which are in an edge.
      void
      compute (std::vector<int> & edge_point_indices);

      float
      getRadius ();

      std::size_t
      getK ();

    private:
      PointCloudConstPtr point_cloud_;
      float radius_;
      float k_;
  };

} // namespace radi

#endif // MIRROR_CVS_H_
