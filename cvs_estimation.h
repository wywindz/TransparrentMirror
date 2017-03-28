/*
 * Define Corner Vector Sequence (CVS) Feature as well as the related classes for estimating the feature.
 *
 */

#ifndef MIRROR_CVS_ESTIMATION_H_
#define MIRROR_CVS_ESTIMATION_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include "cvs.h"

namespace radi
{
  class CVSEstimation
  {
    public:
      CVSEstimation();
      ~CVSEstimation();

      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      void
      setInputCloud (const PointCloudConstPtr & point_cloud);

      void
      setRadius (float radius);

      void
      setDistance (float distance);

      void
      setMinNumEdges (std::size_t min_num_edges);

      void esimate(std::vector<CVSFeature> & cvs_feature_list);

      float
      getRadius ();

      float
      getMinDistance ();

      std::size_t
      getMinNumEdges ();

    private:
      // pcl::PointCloud<pcl::PointXYZ>::ConstPtr inputCloud;
      PointCloudConstPtr point_cloud_;
      float radius_;
      float distance_;
      float min_num_edges_;
  };

} // namespace radi

#endif // MIRROR_CVS_H_
