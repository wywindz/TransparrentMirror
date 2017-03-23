#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include "cvs_corresp_group.h"

namespace radi
{
  CVSCorrespGroup::CVSCorrespGroup () : threshold_(0.1)
  { }

  CVSCorrespGroup::~CVSCorrespGroup ()
  { }

  void
  CVSCorrespGroup::setModelFeatures (const std::vector<CVSFeature> * model_features)
  {
    model_features_ = model_features;
  }

  void
  CVSCorrespGroup::setSceneFeatures (const std::vector<CVSFeature> * scene_features)
  {
    scene_features_ = scene_features;
  }

  void
  CVSCorrespGroup::setThreshold (float threshold)
  {
    threshold_ = threshold;
  }

  void
  CVSCorrespGroup::recognize (std::vector<Eigen::Matrix4f> & transf_list)
  {
    std::vector<pcl::Correspondence> corresp_list;
    this->recognize(transf_list, corresp_list);
  }

  void
  CVSCorrespGroup::recognize (std::vector<Eigen::Matrix4f> & transf_list,
             std::vector<pcl::Correspondence> & corresp_list)
  {
    for (std::size_t idx_scene = 0; idx_scene < scene_features_->size (); ++idx_scene)
    {
      for (std::size_t idx_model = 0; idx_model < model_features_->size (); ++idx_model)
      {
        // vector pair candiates.
        std::vector<std::vector<pcl::Correspondence> > vector_pairs;
        pairFeatures((*scene_features_)[idx_scene], (*model_features_)[idx_model], vector_pairs);

        std::cout << "Vector pairs: " << vector_pairs.size() << std::endl;

        // Calcualte the transformation matrices.
        std::vector<Eigen::Matrix4f> transf_candidates;
        for (std::size_t idx_pair = 0; idx_pair < vector_pairs.size (); ++idx_pair)
        {
          Eigen::Matrix3f mat_covariance = Eigen::MatrixXf::Zero(3,3);
          for (std::size_t idx_vector = 0; idx_vector < vector_pairs[idx_pair].size (); ++idx_vector)
          {
            std::size_t idx_vector_scene = vector_pairs[idx_pair][idx_vector].index_query;
            std::size_t idx_vector_model = vector_pairs[idx_pair][idx_vector].index_match;
            Eigen::Vector3f vect_scene = (*scene_features_)[idx_scene].getVector(idx_vector_scene);
            Eigen::Vector3f vect_model = (*model_features_)[idx_model].getVector(idx_vector_model);
            mat_covariance += vect_scene * vect_model.transpose();
          }

          Eigen::JacobiSVD<Eigen::Matrix3f> svd_solver;
          svd_solver.compute(mat_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

          Eigen::Matrix3f mat_rotation = svd_solver.matrixV() * svd_solver.matrixU().transpose();

          Eigen::Matrix4f mat_transf = Eigen::MatrixXf::Identity(4,4);
          mat_transf.block(0,0,3,3) = mat_rotation;
          mat_transf.block(0,3,3,1) = (*scene_features_)[idx_scene].getCornerPosition()
                  - (*model_features_)[idx_model].getCornerPosition();

          transf_candidates.push_back(mat_transf);
        }

        std::cout << "Number of transformation matrices: " << transf_candidates.size() << std::endl;
        transf_list = transf_candidates;

        // ToDo:
        // Detect if the transformations can be applied to other features and refine them.
        /*
        for (std::size_t idx_refine = 0; idx_refine < transf_candidates.size(); ++idx_refine)
        {
          if (scene_features_->size() > 1)
          {
            for (std::size_t idx_feature = 0; idx_feature < scene_features_->size(); ++idx_feature)
            {
              CVSFeature feature_transformed;
              transformCVSFeature(transf_candidates[idx_refine], (*scene_features_)[idx_feature], feature_transformed);

              bool found_pair = false;
              for (std::size_t idx_feature_model = 0; idx_feature_model < model_features_->size(); ++idx_feature_model)
              {
                if (pairFeatures(feature_transformed, (*model_features_)[idx_feature_model]))
                {
                  found_pair = true;
                  break;
                }
              }

              if (!found_pair)
              {
                // ToDo:
                // Cannot find a pair. Remove the corresponding transformation matrix.
              }
            }
          }
        }
        */
      }
    }
  }

  bool
  CVSCorrespGroup::pairFeatures (const CVSFeature & scene_feature, const CVSFeature & model_feature)
  {
    std::vector<std::vector<pcl::Correspondence> > vector_pairs;
    pairFeatures (scene_feature, model_feature, vector_pairs);

    if (vector_pairs.empty())
      return (false);
    else
      return (true);
  }

  void
  CVSCorrespGroup::pairFeatures (const CVSFeature & scene_feature,
          const CVSFeature & model_feature, std::vector<std::vector<pcl::Correspondence> > & vector_pairs)
  {
    const std::vector<float> & angle_list_scene = scene_feature.getIncludedAngles();
    const std::vector<float> & angle_list_model = model_feature.getIncludedAngles();

    // Combine and find the pair.
    if (angle_list_scene.size() == angle_list_model.size())
    {
      // fully pair.
      int start_model;
      for (std::size_t j = 0; j < angle_list_model.size(); ++j)
      {
        // Find the start point.
        if (std::abs(angle_list_scene[0]-angle_list_model[j]) < 0.3)
        {
          start_model = j;
          // Start to match the angles.
          bool hasMatched = true;
          for (std::size_t k = 0; k < angle_list_model.size(); ++k)
          {
            if (std::abs(angle_list_scene[k]-angle_list_model[(start_model+k)%angle_list_model.size()]) > 0.3)
            {
              hasMatched = false;
              break;
            }
          }

          if (hasMatched)
          {
            std::vector<pcl::Correspondence> pair_indices_list;
            for (std::size_t k = 0; k < angle_list_model.size(); ++k)
            {
              pcl::Correspondence corresp;
              corresp.index_query = k;
              corresp.index_match = (start_model+k)%angle_list_model.size();
              pair_indices_list.push_back(corresp);
            }

            std::cout << "Has matched." << std::endl;

            vector_pairs.push_back(pair_indices_list);
          }
        }
      }
    }
    else if (angle_list_scene.size() > angle_list_model.size())
    {
      // This is impossible for a pair of correspondence features.
      vector_pairs = std::vector<std::vector<pcl::Correspondence> >();
    }
    else
    {
      // partially pair.
    }

  }

} // namespace radi
