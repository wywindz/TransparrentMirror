/*
 * Correspondence group.
 */

#ifndef MIRROR_CVS_CORRESP_GROUP_H_
#define MIRROR_CVS_CORRESP_GROUP_H_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <pcl/recognition/cg/correspondence_grouping.h>

#include "cvs.h"

namespace radi
{
  class CVSCorrespGroup
  {
    public:
      CVSCorrespGroup ();
      ~CVSCorrespGroup ();

      void
      setModelFeatures (const std::vector<CVSFeature> * model_features);

      void
      setSceneFeatures (const std::vector<CVSFeature> * scene_features);

      void
      setThreshold (float threshold);

      inline const std::vector<CVSFeature> *
      getModelFeatures () const { return model_features_; }

      inline const std::vector<CVSFeature> *
      getSceneFeatures () const { return scene_features_; }

      inline float
      getThreshold () const { return threshold_; }

      void
      recognize (std::vector<Eigen::Matrix4f> & transf_list);

      void
      recognize (std::vector<Eigen::Matrix4f> & transf_list, std::vector<pcl::Correspondence> & feature_corresp_list);

    private:
      const std::vector<CVSFeature> * model_features_;
      const std::vector<CVSFeature> * scene_features_;
      float threshold_;

      bool
      pairFeatures (const CVSFeature & scene_feature, const CVSFeature & model_feature,
          std::vector<std::vector<pcl::Correspondence> > & angle_corresps_list);

  }; // class CVSCorrespGroup

} // namespace radi

#endif
