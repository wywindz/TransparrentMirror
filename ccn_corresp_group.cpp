#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "ccn_corresp_group.h"

namespace radi
{
  CCNCorrespGroup::CCNCorrespGroup () : threshold_radius_(0.1), threshold_normal_(0.5)
  { }

  CCNCorrespGroup::~CCNCorrespGroup ()
  { }

  void
  CCNCorrespGroup::setModelFeatures (const std::vector<CCNFeature> * model_features)
  {
    model_features_ = model_features;
  }

  void
  CCNCorrespGroup::setSceneFeatures (const std::vector<CCNFeature> * scene_features)
  {
    scene_features_ = scene_features;
  }

  void
  CCNCorrespGroup::setThresholdRadius (float threshold_radius)
  {
    threshold_radius_ = threshold_radius;
  }

  void
  CCNCorrespGroup::setThresholdNormal (float threshold_normal)
  {
    threshold_normal_ = threshold_normal;
  }

  void
  CCNCorrespGroup::recognize (std::vector<Eigen::Matrix4f> & transf_list)
  {
    std::vector<pcl::Correspondence> corresp_list;
    this->recognize(transf_list, corresp_list);
  }

  void
  CCNCorrespGroup::recognize (std::vector<Eigen::Matrix4f> & transf_list,
             std::vector<pcl::Correspondence> & feature_corresp_list)
  {
    /*
    for (std::size_t idx_scene = 0; idx_scene < scene_features_->size (); ++idx_scene)
    {
      // Match every feature in the model.
      for (std::size_t idx_model = 0; idx_model < model_features_->size (); ++idx_model)
      {
        // vector pair candiates.
        std::vector<std::vector<pcl::Correspondence> > angle_corresps_list;
        if (pairFeatures((*scene_features_)[idx_scene], (*model_features_)[idx_model], angle_corresps_list))
        {
          // Calcualte the transformation matrices.
          for (std::size_t idx_angle_corresps = 0; idx_angle_corresps < angle_corresps_list.size (); ++idx_angle_corresps)
          {
            Eigen::Matrix3f mat_covariance = Eigen::MatrixXf::Zero(3,3);
            for (std::size_t idx_angle = 0; idx_angle < angle_corresps_list[idx_angle_corresps].size (); ++idx_angle)
            {
              std::size_t idx_angle_scene = angle_corresps_list[idx_angle_corresps][idx_angle].index_query;
              std::size_t idx_angle_model = angle_corresps_list[idx_angle_corresps][idx_angle].index_match;
              std::size_t idx_vector_scene = (*scene_features_)[idx_scene].getIndexPairs()[idx_angle_scene][0];
              std::size_t idx_vector_model = (*model_features_)[idx_scene].getIndexPairs()[idx_angle_model][0];
              Eigen::Vector3f vect_scene = (*scene_features_)[idx_scene].getVector(idx_vector_scene);
              Eigen::Vector3f vect_model = (*model_features_)[idx_model].getVector(idx_vector_model);
              // mat_covariance += vect_scene * vect_model.transpose();
              mat_covariance += vect_model * vect_scene.transpose();
            }

            Eigen::JacobiSVD<Eigen::Matrix3f> svd_solver;
            svd_solver.compute(mat_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::Matrix3f mat_rotation = svd_solver.matrixV() * svd_solver.matrixU().transpose();

            Eigen::Matrix4f mat_transf = Eigen::MatrixXf::Identity(4,4);
            mat_transf.block(0,0,3,3) = mat_rotation;
            // mat_transf.block(0,3,3,1) = -mat_rotation * (*scene_features_)[idx_scene].getCornerPosition()
            //         + (*model_features_)[idx_model].getCornerPosition();
            mat_transf.block(0,3,3,1) = -mat_rotation * (*model_features_)[idx_scene].getCornerPosition()
                    + (*scene_features_)[idx_model].getCornerPosition();

            // ToDo: Detect if the transformation can be applied to other features.
            if (scene_features_->size() > 1)
            {
              // for (std::size_t idx_feature = 0; idx_feature < scene_features_->size(); ++idx_feature)
              // {
              //   CVSFeature feature_transformed;
              //   transformCVSFeature(transf_candidates[idx_refine], (*scene_features_)[idx_feature], feature_transformed);

              //   bool found_pair = false;
              //   for (std::size_t idx_feature_model = 0; idx_feature_model < model_features_->size(); ++idx_feature_model)
              //   {
              //     if (pairFeatures(feature_transformed, (*model_features_)[idx_feature_model]))
              //     {
              //       found_pair = true;
              //       break;
              //     }
              //   }

              //   if (!found_pair)
              //   {
              //     // ToDo:
              //     // Cannot find a pair. Remove the corresponding transformation matrix.
              //   }
              // }
            }
            else
            {
              transf_list.push_back(mat_transf);
              pcl::Correspondence corresp;
              corresp.index_query = idx_scene;
              corresp.index_match = idx_model;
              feature_corresp_list.push_back(corresp);

              // std::cout << "Number of transformation matrices: " << transf_list.size() << std::endl;
            }
          }
        }
      }
    }
    */
  }

  bool
  CCNCorrespGroup::pairFeatures (const CCNFeature & scene_feature,
          const CCNFeature & model_feature, std::vector<std::vector<pcl::Correspondence> > & corresps_list)
  {
    /*
    const std::vector<float> & angle_sequence_scene = scene_feature.getIncludedAngles();
    const std::vector<float> & angle_sequence_model = model_feature.getIncludedAngles();

    angle_corresps_list = std::vector<std::vector<pcl::Correspondence> > ();

    // Combine and find the pair.
    // The number of angles in the scene_feature MUST be less than or equal to that in the model_feature.
    if (angle_sequence_scene.size() == angle_sequence_model.size())
    {
      // Fully pair.
      int idx_start;
      for (std::size_t idx_model = 0; idx_model < angle_sequence_model.size(); ++idx_model)
      {
        // Find the start index.
        if (std::abs(angle_sequence_scene[0]-angle_sequence_model[idx_model]) < threshold_)
        {
          idx_start = idx_model;
          // Start to match the angles.
          bool flag_matched = true;
          for (std::size_t k = 0; k < angle_sequence_scene.size(); ++k)
          {
            float angle_scene = angle_sequence_scene[k];
            float angle_model = angle_sequence_model[(idx_start+k)%angle_sequence_model.size()];
            if (std::abs(angle_scene-angle_model) > threshold_)
            {
              flag_matched = false;
              break;
            }
          }

          if (flag_matched)
          {
            std::vector<pcl::Correspondence> corresp_list;
            for (std::size_t k = 0; k < angle_sequence_scene.size(); ++k)
            {
              pcl::Correspondence corresp;
              corresp.index_query = k;
              corresp.index_match = (idx_start+k)%angle_sequence_model.size();
              corresp_list.push_back(corresp);
            }

            angle_corresps_list.push_back(corresp_list);
          }
        }
      }
    }
    else if (angle_sequence_scene.size() < angle_sequence_model.size())
    {
      // Partially pair.
      // One angle in the scene_feature is invalid. Pair the features using the remain angle sequence.
    }

    if (angle_corresps_list.empty ())
      return (false);
    else
      return (true);

    */
  }

} // namespace radi
