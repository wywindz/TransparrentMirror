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

      void
      setModelFeatures (const std::vector<CCNFeature> * model_features);

      void
      setSceneFeatures (const std::vector<CCNFeature> * scene_features);

      void
      setThresholdRadius (float threshold_radius);

      void
      setThresholdNormal (float threshold_normal);

      inline const std::vector<CCNFeature> *
      getModelFeatures () const { return model_features_; }

      inline const std::vector<CCNFeature> *
      getSceneFeatures () const { return scene_features_; }

      inline float
      getThresholdRadius () const { return threshold_radius_; }

      inline float
      getThresholdNormal () const { return threshold_normal_; }

      void
      recognize (std::vector<Eigen::Matrix4f> & transf_list);

      void
      recognize (std::vector<Eigen::Matrix4f> & transf_list,
                 std::vector<pcl::Correspondence> & feature_corresp_list);

    private:
      const std::vector<CCNFeature> * model_features_;
      const std::vector<CCNFeature> * scene_features_;
      float threshold_radius_;
      float threshold_normal_;

      bool
      pairFeatures (const CCNFeature & scene_feature,
              const CCNFeature & model_feature, std::vector<std::vector<pcl::Correspondence> > & corresps_list);

  }; // class CCNCorrespGroup

} // namespace radi

#endif
