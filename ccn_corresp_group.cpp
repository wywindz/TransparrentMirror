#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include "ccn_corresp_group.h"
#include "icf.h"

namespace radi
{
  CCNCorrespGroup::CCNCorrespGroup () : radius_variation_ (0.3), resolution_ (0.5)
  { }

  CCNCorrespGroup::~CCNCorrespGroup ()
  { }

  void
  CCNCorrespGroup::setInputCloud (const PointCloudConstPtr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

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
  CCNCorrespGroup::setRadiusVariation (float radius_variation)
  {
    radius_variation_ = radius_variation;
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
    IterativeClosestFace icf;
    icf.setScenePointCloud (this->point_cloud_);
    icf.setReferenceModel ("Models/cup.stl");

    // Iterate each feature in the scene.
    for (std::size_t idx_scene = 0; idx_scene < scene_features_->size (); ++idx_scene)
    {
      // Match every feature in the model.
      for (std::size_t idx_model = 0; idx_model < model_features_->size (); ++idx_model)
      {
        const CCNFeature & ccn_scene = (*scene_features_)[idx_scene];
        const CCNFeature & ccn_model = (*model_features_)[idx_model];
        if (std::abs (ccn_scene.getRadius () - ccn_model.getRadius ()) < radius_variation_)
        {
          // Construct the base transformation.
          float theta_scene = std::acos (ccn_scene.getNormal ().dot (Eigen::Vector3f (0.0, 0.0, 1.0)));
          Eigen::Vector3f axis_scene = ccn_scene.getNormal ().cross (Eigen::Vector3f (0.0, 0.0, 1.0));
          Eigen::Affine3f affine_transf_scene = Eigen::Translation3f (ccn_scene.getCenter ())
              * Eigen::AngleAxisf (theta_scene, axis_scene);

          float theta_model = std::acos (ccn_model.getNormal ().dot (Eigen::Vector3f (0.0, 0.0, 1.0)));
          // Eigen::Vector3f axis_model = ccn_model.getNormal().cross(Eigen::Vector3f(0.0, 0.0, 1.0));
          Eigen::Vector3f axis_model = Eigen::Vector3f (1.0, 0.0, 0.0);
          Eigen::Affine3f affine_transf_model = Eigen::Translation3f (ccn_model.getCenter ())
              * Eigen::AngleAxisf (theta_model, axis_model);

          int num_rotation = std::floor (2*3.1415926 / resolution_);
          std::vector<Eigen::Matrix4Xf> inner_transf_list;
          std::vector<float> inner_objective_list;
          for (int idx_roation = 0; idx_roation <= num_rotation; ++idx_roation)
          {
            Eigen::Affine3f affine_transf_scene_new = affine_transf_scene
                * Eigen::AngleAxisf (float (idx_roation)*resolution_, ccn_scene.getNormal ());
            Eigen::Affine3f transformation = affine_transf_model * affine_transf_scene_new.inverse ();

            std::cout << "Transform normal of the scene: " << transformation.rotation ()*ccn_scene.getNormal () << std::endl;
            std::cout << "Transform center of the scene: " << transformation.rotation ()*ccn_scene.getCenter () + transformation.translation () << std::endl;

            // Perform ICF algorithm. Refine the tramsformations.
            try
            {
              float objective_value = icf.calObjectiveValue (transformation.matrix ());
              inner_transf_list.push_back (transformation.matrix ());
              inner_objective_list.push_back(objective_value);
            }
            catch (char const * msg)
            {
              std::cout << "Bad initial transformation. " << msg << std::endl;
              continue;
            }
          }

          if (~inner_objective_list.empty())
          {
            std::vector<std::size_t> order_indices (inner_objective_list.size());
            std::iota(order_indices.begin (), order_indices.end (), 0);
            std::sort(order_indices.begin (), order_indices.end (), [&inner_objective_list](int idx_1, int idx_2)
                    { return inner_objective_list[idx_1] >= inner_objective_list[idx_2]; });

            transf_list.push_back(inner_transf_list[order_indices[0]]);
          }
        }
      }
    }

    std::cout << "Number of transformations: " << transf_list.size() << std::endl;
  }

} // namespace radi
