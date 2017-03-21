/*
 * Iterative Closest Face (ICF) algorithm.
 */

#ifndef MIRROR_POSE_H_
#define MIRROR_POSE_H_

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Dense>

#include "mesh.h"

namespace radi
{

  class Pose
  {
    public:
      Pose ();
      Pose (const Mesh & refModel, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
            const Eigen::Matrix4f & transformMatrix = Eigen::MatrixXf::Identity(4,4));
      ~Pose ();

      const Pose &
      operator= (const Pose & poseA);

      void
      setTransformMatrix (const Eigen::Matrix4f & transformMatrix);

      template <typename realT> void
      setDistanceThreshold (realT threshold) { this->distanceThreshold_ = threshold; }

      float
      objectiveValue () const;

      float
      objectiveValue (const Mesh & refModel, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

      const Eigen::Vector3f
      zyzEuler () const;

      const Eigen::Vector3f
      translation () const;

    private:
      Eigen::Matrix4f matTransform_;
      float objectiveValue_;
      float distanceThreshold_;
  };

  bool operator==(const Pose & poseA, const Pose & poseB);

  Pose findBestPose(const std::vector<Pose> & poseList);
  Pose findBestPose(std::vector<Pose> & poseList, const Mesh & modelMesh, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

  // Calculate the distance from a point to a triangle mesh.
  float distPointTriangle(const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangleVertices);
  // Calculate the projected point on a plane.
  Eigen::Vector3f pointOnPlane(const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangleVertices);
  // Detect if the projection of a point is inside the triangle.
  bool isPointInTriangle(const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangleVertices);
  // Calculate the distance from a point to another point.
  float distPointPoint(const Eigen::Vector3f & pointA, const Eigen::Vector3f & pointB);
  // Calculate the distance from a point to a line segment.
  float distPointLineSegment(const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & segmentVertices);
  // Calculate the distance from a point to a plane.
  float distPointPlane(const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangleVertices);

  Pose generatePose(const Eigen::Vector3f & translation);
  Pose generatePose(const Eigen::Vector3f & translation, const Eigen::Vector3f & zyzEulerAngle);

} // namespace radi

#endif
