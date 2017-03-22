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

        // Detect if the transformations can be applied to other features and refine them.
        for (std::size_t idx_refine = 0; idx_refine < transf_candidates.size(); ++idx_refine)
        {

        }

      }
    }
  }

  void
  CVSCorrespGroup::pairFeatures (const CVSFeature & scene_feature,
          const CVSFeature & model_feature, std::vector<std::vector<pcl::Correspondence> > & vector_pairs)
  {

  }

} // namespace radi
