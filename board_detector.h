/*
 * Detect board points.
 *
 */

#ifndef MIRROR_BOARD_DETECTOR_H_
#define MIRROR_BOARD_DETECTOR_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace radi {

  class BoardDetector
  {
    public:
      BoardDetector ();
      ~BoardDetector ();

      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      void
      setInputCloud (const PointCloudConstPtr & point_cloud);

      inline void
      setK (int k) { k_ = k; }

      inline int
      getK () { return (k_); }

      // Obtain the indices of the points which are in a board.
      void
      compute (std::vector<int> & board_point_indices);

    private:
      PointCloudConstPtr point_cloud_;
      int k_;
  };

} // namespace radi

#endif // MIRROR_BOARD_H_
