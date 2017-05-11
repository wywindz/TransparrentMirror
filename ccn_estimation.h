/*
 * Estimate CCN Feature..
 *
 */

#ifndef MIRROR_CCN_ESTIMATION_H_
#define MIRROR_CCN_ESTIMATION_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "ccn.h"

namespace radi
{
  class CCNEstimation
  {
    public:
      CCNEstimation();
      ~CCNEstimation();

      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      void
      setInputCloud (const PointCloudConstPtr & point_cloud);

      void
      setMinRadius (float min_radius);

      void
      setMinNumPoints (std::size_t min_num_points);

      void esimate(std::vector<CCNFeature> & ccn_feature_list);

      float
      getMinRadius ();

      std::size_t
      getMinNumPoints ();

    private:
      PointCloudConstPtr point_cloud_;
      float min_radius_;
      float min_num_points_;
  };

} // namespace radi

#endif // MIRROR_CCN_H_
