/*
 * Correspondence group.
 */

#ifndef MIRROR_CCN_CORRESP_GROUP_H_
#define MIRROR_CCN_CORRESP_GROUP_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pcl/recognition/cg/correspondence_grouping.h>

#include "ccn.h"
#include "icf.h"

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

      typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

      void
      setReferenceModel (const std::string & model_file_path);

      void
      setInputCloud (const PointCloudConstPtr & point_cloud);

      void
      setIndices (const IndicesConstPtr & indices);

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
      recognize (float & best_objective, Eigen::Matrix4f & best_transformation);

    private:
      std::string model_file_;
      PointCloudConstPtr point_cloud_;    /*!<! Input point cloud. */
      IndicesConstPtr indices_;   /*!<! Indices of the points used in ICF algorithm. */
      const std::vector<CCNFeature> * model_features_;    /*<! Model feature list. */
      const std::vector<CCNFeature> * scene_features_;    /*<! Point cloud feature list. */
      float radius_variation_;    /*<! Variation of radius which is used to pair 2 features. */
      float resolution_;   /*<! Resolution of the rotation around the normal of circle plane. */
      IterativeClosestFace icf_;   /*<! ICF algorithm. */

      void
      calSelfRotation (float & best_objective, Eigen::Matrix4f & best_transformation,
          const CCNFeature & scene_feature, const CCNFeature & model_feature, bool reversed = false);

  }; // class CCNCorrespGroup

} // namespace radi

#endif
