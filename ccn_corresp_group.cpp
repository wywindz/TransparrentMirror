#include <cmath>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ccn_corresp_group.h"
#include "icf.h"

namespace radi
{
  CCNCorrespGroup::CCNCorrespGroup () : model_file_ (""), point_cloud_ (new pcl::PointCloud<pcl::PointXYZ> ()),
      indices_ (new std::vector<int> ()), model_features_ (NULL), scene_features_ (NULL),
      radius_variation_ (0.3), resolution_ (0.5)
  { }

  CCNCorrespGroup::~CCNCorrespGroup ()
  { }

  void
  CCNCorrespGroup::setReferenceModel (const std::string & model_file_path)
  {
    model_file_ = model_file_path;
  }

  void
  CCNCorrespGroup::setInputCloud (const PointCloudConstPtr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  void
  CCNCorrespGroup::setIndices (const IndicesConstPtr & indices)
  {
    indices_ = indices;
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
  CCNCorrespGroup::recognize (float & best_objective, Eigen::Matrix4f & best_transformation)
  {
    icf_.setReferenceModel(this->model_file_);
    icf_.setScenePointCloud (this->point_cloud_);
    icf_.setIndices(this->indices_);

    std::vector<float> objective_list;
    std::vector<Eigen::Matrix4f> transf_list;

    // Iterate each feature in the scene.
    for (std::size_t idx_scene = 0; idx_scene < scene_features_->size (); ++idx_scene)
    {
      // Match every feature in the model.
      for (std::size_t idx_model = 0; idx_model < model_features_->size (); ++idx_model)
      {
        const CCNFeature & ccn_scene = (*scene_features_)[idx_scene];
        const CCNFeature & ccn_model = (*model_features_)[idx_model];

        float objective;
        Eigen::Matrix4f mat_transf;
        calSelfRotation(objective, mat_transf, ccn_scene, ccn_model);
        float objective_reversed;
        Eigen::Matrix4f mat_transf_reversed;
        calSelfRotation(objective_reversed, mat_transf_reversed, ccn_scene, ccn_model, true);

        if (objective >= objective_reversed)
        {
          objective_list.push_back (objective);
          transf_list.push_back (mat_transf);
        }
        else
        {
          objective_list.push_back (objective_reversed);
          transf_list.push_back (mat_transf_reversed);
        }
      }
    }

    if (!transf_list.empty ())
    {
      std::vector<std::size_t> order_indices (objective_list.size());
      std::iota(order_indices.begin (), order_indices.end (), 0);
      std::sort(order_indices.begin (), order_indices.end (), [&objective_list](int idx_1, int idx_2)
              { return objective_list[idx_1] >= objective_list[idx_2]; });

      best_objective = objective_list[order_indices[0]];
      best_transformation = transf_list[order_indices[0]];
    }
    else
    {
      best_objective = 0;
      best_transformation = Eigen::Matrix4Xf::Identity (4,4);
    }
  }

  void
  CCNCorrespGroup::calSelfRotation (float & best_objective, Eigen::Matrix4f & best_transformation,
      const CCNFeature & scene_feature, const CCNFeature & model_feature, bool reversed)
  {
    // Reverse the normal or not.
    Eigen::Vector3f scene_normal;
    if (reversed)
      scene_normal = -scene_feature.getNormal ();
    else
      scene_normal = scene_feature.getNormal ();

    // Construct the base transformation.
    float theta_scene = std::acos (scene_normal.dot (Eigen::Vector3f (0.0, 0.0, 1.0)));
    Eigen::Vector3f axis_scene;
    if (theta_scene < 1.0E-6)
    {
      theta_scene = 0.0;
      axis_scene = Eigen::Vector3f (1.0, 0.0, 0.0);
    }
    else
    {
      axis_scene = scene_normal.cross (Eigen::Vector3f (0.0, 0.0, 1.0));
    }
    Eigen::Affine3f affine_transf_scene = Eigen::Translation3f (scene_feature.getCenter ())
        * Eigen::AngleAxisf (theta_scene, axis_scene.normalized ());

    float theta_model = std::acos (model_feature.getNormal ().dot (Eigen::Vector3f (0.0, 0.0, 1.0)));
    Eigen::Vector3f axis_model;
    if (theta_model < 1.0E-6)
    {
      theta_model = 0.0;
      axis_model = Eigen::Vector3f (1.0, 0.0, 0.0);
    }
    else
    {
      axis_model = model_feature.getNormal ().cross (Eigen::Vector3f (0.0, 0.0, 1.0));
    }
    Eigen::Affine3f affine_transf_model = Eigen::Translation3f (model_feature.getCenter ())
        * Eigen::AngleAxisf (theta_model, axis_model.normalized ());

    int num_rotation = std::floor (2*3.1415926 / resolution_);
    std::vector<Eigen::Matrix4Xf> inner_transf_list;
    std::vector<float> inner_objective_list;
    for (int idx_roation = 0; idx_roation <= num_rotation; ++idx_roation)
    {
      Eigen::Affine3f affine_transf_scene_new = affine_transf_scene
          * Eigen::AngleAxisf (float (idx_roation)*resolution_, scene_normal);
      Eigen::Affine3f transformation = affine_transf_model * affine_transf_scene_new.inverse ();

      // std::cout << "Transform normal of the scene: " << transformation.rotation ()*scene_normal << std::endl;
      // std::cout << "Transform center of the scene: " << transformation.rotation ()*scene_feature.getCenter () + transformation.translation () << std::endl;

      // Show 3d model and transformed point cloud.
      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scene (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud(*(this->point_cloud_), *transformed_scene, transformation.matrix ());
      pcl::PolygonMesh mesh;
      // pcl::io::loadPolygonFileSTL("Models/cuboid.stl", mesh);
      pcl::io::loadPolygonFileSTL("Models/cup.stl", mesh);
      pcl::visualization::PCLVisualizer viewer ("Model & Point Cloud");
      viewer.addPolygonMesh(mesh);
      viewer.addPointCloud<pcl::PointXYZ> (transformed_scene, "Transformed point cloud");
      while (!viewer.wasStopped ())
      {
        viewer.spinOnce ();
      }

      // Perform ICF algorithm. Refine the tramsformations.
      try
      {
        float objective_value = icf_.calObjectiveValue (transformation.matrix ());
        inner_transf_list.push_back (transformation.matrix ());
        inner_objective_list.push_back(objective_value);
      }
      catch (char const * msg)
      {
        std::cout << "Bad initial transformation. " << msg << std::endl;
        continue;
      }
    }

    if (!inner_objective_list.empty())
    {
      std::vector<std::size_t> order_indices (inner_objective_list.size());
      std::iota(order_indices.begin (), order_indices.end (), 0);
      std::sort(order_indices.begin (), order_indices.end (), [&inner_objective_list](int idx_1, int idx_2)
              { return inner_objective_list[idx_1] >= inner_objective_list[idx_2]; });

      std::cout << "Index: " << order_indices[0] << std::endl;
      best_objective = inner_objective_list[order_indices[0]];
      best_transformation = inner_transf_list[order_indices[0]];
    }
    else
    {
      best_objective = 0.0;
      best_transformation = Eigen::Matrix4Xf::Identity (4,4);
    }
  }

} // namespace radi
