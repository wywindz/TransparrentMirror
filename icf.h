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
      IterativeClosestFace ();
      IterativeClosestFace (const std::string & refModelPath,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr scene);
      ~IterativeClosestFace ();

      void
      setReferenceModel (const std::string & refModelPath);

      void
      setScenePointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

      void
      setMaxIterations (std::size_t numIterationOuter, std::size_t numIterationMedium,
                        std::size_t numIterationInner);

      const Mesh &
      getReferenceModel () const;

      pcl::PointCloud<pcl::PointXYZ>::Ptr
      getScenePointCloud () const;

      Eigen::Vector3i
      getMaxIterations ();

      void
      estimate (Eigen::Matrix4f & matrix);

      float
      objectiveValue ();

    private:
      Mesh modelMesh;
      pcl::PointCloud<pcl::PointXYZ>::Ptr scenePointCloud;
      Eigen::Vector3f minBoundaryTranslation;
      Eigen::Vector3f maxBoundaryTranslation;
      Eigen::Vector3f minBoundaryRotation;
      Eigen::Vector3f maxBoundaryRotation;
      Eigen::Vector3f deviationTranslation;
      Eigen::Vector3f deviationRotation;
      std::size_t iterationOuter;
      std::size_t iterationMedium;
      std::size_t iterationInner;
      bool hasConverged;
      Eigen::Matrix4f transfMatrix;
      Pose relativePose;
  }; // class IterativeClosestFace

  Eigen::Vector3f uniformRandom(const Eigen::Vector3f & minBoundary, const Eigen::Vector3f & maxBoundary);
  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, const Eigen::Vector3f & deviation);
  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, float deviation);


} // namespace radi

#endif
