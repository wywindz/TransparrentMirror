/*
 * Correspondence group.
 */

#ifndef MIRROR_CCN_CORRESP_GROUP_H_
#define MIRROR_CCN_CORRESP_GROUP_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pcl/recognition/cg/correspondence_grouping.h>

#include "ccn.h"

namespace radi
{
  class CCNCorrespGroup
  {
    public:
      CCNCorrespGroup ();
      ~CCNCorrespGroup ();

      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

      void
      setInputCloud (const PointCloudConstPtr & point_cloud);

      void
      setModelFeatures (const std::vector<CCNFeature> * model_features);

      void
      setSceneFeatures (const std::vector<CCNFeature> * scene_features);

      void
      setRadiusVariation (float radius_variation);

      inline void
      setResolution (float resolution) { resolution_ = resolution; }

      inline const std::vector<CCNFeature> *
      getModelFeatures () const { return model_features_; }

      inline const std::vector<CCNFeature> *
      getSceneFeatures () const { return scene_features_; }

      inline float
      getRadiusVariation () const { return radius_variation_; }

      inline float
      getResolution () const { return resolution_; }

      void
      recognize (std::vector<Eigen::Matrix4f> & transf_list);

      void
      recognize (std::vector<Eigen::Matrix4f> & transf_list,
                 std::vector<pcl::Correspondence> & feature_corresp_list);

    private:
      PointCloudConstPtr point_cloud_;    /*!< Input point cloud. */
      const std::vector<CCNFeature> * model_features_;    /*<! Model feature list. */
      const std::vector<CCNFeature> * scene_features_;    /*<! Point cloud feature list. */
      float radius_variation_;    /*<! Variation of radius which is used to pair 2 features. */
      float resolution_;   /*<! Resolution around the normal of circle plane. */

  }; // class CCNCorrespGroup

} // namespace radi

#endif
