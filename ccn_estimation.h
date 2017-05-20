/*
 * Estimate CCN Feature..
 *
 */

#ifndef MIRROR_CCN_ESTIMATION_H_
#define MIRROR_CCN_ESTIMATION_H_

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
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
      setRadiusLimits (float min_radius, float max_radius);

      void
      setSearchRadius (float search_radius);

      void
      setMinNumPoints (int min_num_points);

      void
      setIdentityThresholds(float position, float radius, float angle);

      void esimate(std::vector<CCNFeature> & ccn_feature_list);

      float
      getMinRadius ();

      std::size_t
      getMinNumPoints ();

      /*!
       * \fn const pcl::IndicesConstPtr getIndices ()
       * \brief Get the indices of the points in features, such as neighbors of the corner, boad points, etc.
       */
      const pcl::IndicesConstPtr
      getIndices ();

    private:
      PointCloudConstPtr point_cloud_;    /*!< Input point cloud. */
      float min_radius_;    /*!< Minimum radius of the 3D cricle. */
      float max_radius_;    /*!< Maximum radius of the 3D cricle. */
      float search_radius_;     /*!< Search radius during SacSegmentation. */
      int min_num_points_;      /*!< Minimum number of points on one circle. */
      float threshold_position_;    /*!< Threshold of position error which is used to detect whether 2 circles are identical. */
      float threshold_radius_;    /*!< Threshold of position error which is used to detect whether 2 circles are identical. */
      float threshold_angle_;   /*!< Threshold of normal error which is used to detect whether 2 circles are identical. */
      pcl::IndicesConstPtr indices_;  /*!< Indices of the points in features. */


      bool
      isInCircleList(const pcl::ModelCoefficients & coefficients, const std::vector<pcl::ModelCoefficients> & circle_list);
  };

} // namespace radi

#endif // MIRROR_CCN_H_
