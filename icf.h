/*
 * Iterative Closest Face (ICF) algorithm.
 */

#ifndef MIRROR_ICF_H_
#define MIRROR_ICF_H_

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Dense>

#include "mesh.h"
#include "pose.h"

namespace radi
{
  class IterativeClosestFace
  {
    public:

      // typedefs
      typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;


      IterativeClosestFace ();
      IterativeClosestFace (const std::string & model_file_path,
                           PointCloudConstPtr scene_point_cloud);
      ~IterativeClosestFace ();

      void
      setReferenceModel (const std::string & model_file_path);

      void
      setScenePointCloud (PointCloudConstPtr scene_point_cloud);

      void
      setMinRangeTranslation (const Eigen::Vector3f & min_range);

      void
      setMaxRangeTranslation (const Eigen::Vector3f & max_range);

      void
      setMinRangeRotation (const Eigen::Vector3f & min_range);

      void
      setMaxRangeRotation (const Eigen::Vector3f & max_range);

      void
      setIterationOuter (std::size_t iteration_outer);

      void
      setIterationInner (std::size_t iteration_inner);

      void
      setInitialTransformation (const Eigen::Matrix4f & init_transf);

      const Mesh &
      getReferenceModel () const;

      PointCloudConstPtr
      getScenePointCloud () const;

      const Eigen::Vector3f
      getMinRangeTranslation ();

      const Eigen::Vector3f
      getMaxRangeTranslation ();

      const Eigen::Vector3f
      getMinRangeRotation ();

      const Eigen::Vector3f
      getMaxRangeRotation ();

      std::size_t
      getIterationOuter ();

      std::size_t
      getIterationInner ();

      const Eigen::Matrix4f
      getInitialTransformation ();

      void
      estimate (Eigen::Matrix4f & estimated_transf);

      float
      objectiveValue ();

    private:
      Mesh model_mesh_;
      PointCloudConstPtr scene_point_cloud_;
      Eigen::Vector3f min_range_translation_;
      Eigen::Vector3f max_range_translation_;
      Eigen::Vector3f min_range_rotation_;
      Eigen::Vector3f max_range_rotation_;
      std::size_t iteration_outer_;
      std::size_t iteration_inner_;
      Eigen::Matrix4f init_transf_;
      float threshold_distance_near_;
      float threshold_distance_extreme_;
      float threshold_valid_;
      bool has_converged;
      Pose relativePose;

      float
      calObjectiveValue (const Eigen::Matrix4f & mat_transf);
  }; // class IterativeClosestFace

  Eigen::Vector3f uniformRandom(const Eigen::Vector3f & min_boundary, const Eigen::Vector3f & max_boundary);
  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, const Eigen::Vector3f & deviation);
  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, float deviation);

  const Eigen::Vector3f matrix2euler (const Eigen::Matrix3f & mat_rotation);
  const Eigen::Matrix3f euler2matrix (const Eigen::Vector3f & euler_angle);


} // namespace radi

#endif
