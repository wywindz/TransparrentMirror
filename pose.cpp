#include <cmath>
#include <pcl/common/transforms.h>
#include "pose.h"

namespace radi
{
  Pose::Pose () : matTransform_(Eigen::MatrixXf::Identity(4,4)),
          objectiveValue_(0.0), distanceThreshold_(200)
  { }

  Pose::Pose (const Mesh & refModel, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene,
          const Eigen::Matrix4f & transformMatrix)
  {
    matTransform_ = transformMatrix;
    objectiveValue(refModel, scene);
  }

  Pose::~Pose () { }

  const Pose &
  Pose::operator= (const Pose & poseA)
  {
    this->matTransform_ = poseA.matTransform_;
    this->objectiveValue_ = poseA.objectiveValue_;
    this->distanceThreshold_ = poseA.distanceThreshold_;
  }

  void
  Pose::setTransformMatrix (const Eigen::Matrix4f & transformMatrix)
  {
    matTransform_ = transformMatrix;
  }

  float
  Pose::objectiveValue() const
  {
    return objectiveValue_;
  }

  float
  Pose::objectiveValue (const Mesh & refModel, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
  {
    pcl::PointCloud<pcl::PointXYZ> transformedScene;
    pcl::transformPointCloud(*scene, transformedScene, matTransform_);

    objectiveValue_ = 0.0;
    for (std::size_t i = 0; i < transformedScene.points.size(); ++i)
    {
      Eigen::Vector3f point;
      point[0] = static_cast<float>(transformedScene.points[i].x);
      point[1] = static_cast<float>(transformedScene.points[i].y);
      point[2] = static_cast<float>(transformedScene.points[i].z);
      std::vector<float> distanceList(refModel.getNumTriangles());
      for (std::size_t j = 0; j < refModel.getNumTriangles(); ++j)
      {
        distanceList[j] = distPointTriangle(point, refModel.getTriangle(j));
      }
      float shortestDistance = *std::min_element(distanceList.begin(), distanceList.end());
      float gpx;
      if (shortestDistance < 30.0)
      {
        gpx = shortestDistance;
      }
      else
      {
          gpx = 4*shortestDistance;
      }

      float fxp;
      if (gpx < distanceThreshold_)
      {
          fxp = 1 - gpx/distanceThreshold_;
      }
      else
      {
          fxp = 0;
      }

      objectiveValue_ += fxp;
    }
  }

  const Eigen::Vector3f
  Pose::zyzEuler() const
  {
    const float PI = 3.1415926;
    const float EPS = 1.0E-8;

    float alpha;
    float beta;
    float gamma;

    // Assuming beta is in [0,pi].
    double a_02 = matTransform_(0,2);
    double a_01 = matTransform_(0,1);
    double a_11 = matTransform_(1,1);
    double a_12 = matTransform_(1,2);
    double a_20 = matTransform_(2,0);
    double a_21 = matTransform_(2,1);
    double a_22 = matTransform_(2,2);

    beta = std::atan2(std::sqrt(std::pow(a_02,2)+std::pow(a_12,2)), a_22);

    if ((EPS < beta) && (beta < (PI-EPS)))
    {
      alpha = std::atan2(a_12, a_02);
      gamma = std::atan2(a_21, -a_20);
    }
    else if (beta <= EPS)
    {
      alpha = 0.0;
      gamma = std::atan2(-a_01, a_11);
    }
    else
    {
      alpha = 0.0;
      gamma = std::atan2(a_01, a_11);
    }

    return Eigen::Vector3f(alpha, beta, gamma);
  }

  const Eigen::Vector3f
  Pose::translation() const
  {
    Eigen::Vector3f posTranslate;
    posTranslate[0] = matTransform_(0,3);
    posTranslate[1] = matTransform_(1,3);
    posTranslate[2] = matTransform_(2,3);

    return posTranslate;
  }

  bool
  operator== (const Pose & poseA, const Pose & poseB)
  {
    return (poseA.zyzEuler() == poseB.zyzEuler()) && (poseA.translation() == poseB.translation());
  }


  Pose
  findBestPose (const std::vector<Pose> & poseList)
  {
    std::vector<float> objectiveValues(poseList.size());
    for (std::size_t i = 0; i < poseList.size(); ++i)
    {
      objectiveValues[i] = poseList[i].objectiveValue();
    }

    std::size_t index = std::max_element(objectiveValues.begin(), objectiveValues.end()) - objectiveValues.begin();

    return poseList[index];
  }

  Pose
  findBestPose (std::vector<Pose> & poseList, const Mesh & modelMesh,
          const pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
  {
    for (std::size_t i = 0; i < poseList.size(); ++i)
    {
      poseList[i].objectiveValue(modelMesh, scene);
    }

    return findBestPose(poseList);
  }


  // Calculate the distance from a point to a triangle mesh.
  float
  distPointTriangle (const Eigen::Vector3f & point,
          const std::vector<Eigen::Vector3f> & triangleVertices)
  {
    if (isPointInTriangle(point, triangleVertices))
    {
      return distPointPlane(point, triangleVertices);
    }
    else
    {
      // Calculate the distance from the point to the vertices and segments.
      std::vector<float> distances(6);
      distances[0] = distPointPoint(point, triangleVertices[0]);
      distances[1] = distPointPoint(point, triangleVertices[1]);
      distances[2] = distPointPoint(point, triangleVertices[2]);
      std::vector<Eigen::Vector3f> segmentVertices(2);
      segmentVertices[0] = triangleVertices[0];
      segmentVertices[1] = triangleVertices[1];
      distances[3] = distPointLineSegment(point, segmentVertices);
      segmentVertices[0] = triangleVertices[0];
      segmentVertices[1] = triangleVertices[2];
      distances[4] = distPointLineSegment(point, segmentVertices);
      segmentVertices[0] = triangleVertices[1];
      segmentVertices[1] = triangleVertices[2];
      distances[5] = distPointLineSegment(point, segmentVertices);

      return *std::min_element(distances.begin(), distances.end());
    }
  }

  // Calculate the projected point on a plane.
  Eigen::Vector3f
  pointOnPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangleVertices)
  {
    Eigen::Vector3f projectedPoint;
    Eigen::Vector3f vectAB = triangleVertices[1] - triangleVertices[0];
    Eigen::Vector3f vectAC = triangleVertices[2] - triangleVertices[0];
    Eigen::Vector3f normal = vectAB.cross(vectAC);
    normal /= std::sqrt(normal.dot(normal));
    Eigen::Vector3f vectAP = point - triangleVertices[0];
    float distance = distPointPlane(point, triangleVertices);
    projectedPoint = vectAP - distance*normal;

    return projectedPoint;
  }

  // Detect if the projection of a point is inside the triangle.
  bool
  isPointInTriangle (const Eigen::Vector3f & point,
          const std::vector<Eigen::Vector3f> & triangleVertices)
  {
    Eigen::Vector3f vectAB = triangleVertices[1] - triangleVertices[0];
    Eigen::Vector3f vectAC = triangleVertices[2] - triangleVertices[0];
    Eigen::Vector3f vectAP = point - triangleVertices[0];

    float u = (vectAC.dot(vectAC)*vectAP.dot(vectAB) - vectAC.dot(vectAB)*vectAP.dot(vectAC)) /
            (vectAB.dot(vectAB)*vectAC.dot(vectAC) - vectAB.dot(vectAC)*vectAC.dot(vectAB));
    float v = (vectAB.dot(vectAB)*vectAP.dot(vectAC) - vectAB.dot(vectAC)*vectAP.dot(vectAB)) /
            (vectAB.dot(vectAB)*vectAC.dot(vectAC) - vectAB.dot(vectAC)*vectAC.dot(vectAB));

    return (u >= 0) && (v >= 0) && (u + v <= 1);
  }

  // Calculate the distance from a point to another point.
  float
  distPointPoint (const Eigen::Vector3f & pointA, const Eigen::Vector3f & pointB)
  {
    Eigen::Vector3f vectAB = pointB - pointA;
    return std::sqrt(vectAB.dot(vectAB));
  }

  // Calculate the distance from a point to a line segment.
  float
  distPointLineSegment (const Eigen::Vector3f & point,
          const std::vector<Eigen::Vector3f> & segmentVertices)
  {
    Eigen::Vector3f vectV = segmentVertices[1] - segmentVertices[0];
    Eigen::Vector3f vectW = point - segmentVertices[0];
    float scalar_1 = vectW.dot(vectV);
    float scalar_2 = vectV.dot(vectV);

    if (scalar_1 <= 0) {
        // Projected point on the line is on the left of segmentVertices[0].
        return distPointPoint(point, segmentVertices[0]);
    } else if (scalar_1 >= scalar_2) {
        // Projected point on the line is on the right of segmentVertices[1].
        return distPointPoint(point, segmentVertices[1]);
    } else {
        // Projected point on the line is on the line segment.
        Eigen::Vector3f projectedPoint = segmentVertices[0] + scalar_1/scalar_2*vectV;
        return distPointPoint(point, projectedPoint);
    }
  }

  // Calculate the distance from a point to a plane.
  float
  distPointPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangleVertices)
  {
    Eigen::Vector3f vectAB = triangleVertices[1] - triangleVertices[0];
    Eigen::Vector3f vectAC = triangleVertices[2] - triangleVertices[0];
    Eigen::Vector3f normal = vectAB.cross(vectAC);
    normal /= std::sqrt(normal.dot(normal));
    Eigen::Vector3f vectAP = point - triangleVertices[0];

    return vectAP.dot(normal);
  }

  Pose
  generatePose(const Eigen::Vector3f & translation)
  {
    Eigen::Matrix4f matTransform(Eigen::MatrixXf::Identity(4,4));
    matTransform(0,3) = translation[0];
    matTransform(1,3) = translation[1];
    matTransform(2,3) = translation[2];

    Pose pose;
    pose.setTransformMatrix(matTransform);

    return pose;
  }

  Pose
  generatePose (const Eigen::Vector3f & translation, const Eigen::Vector3f & zyzEulerAngle)
  {
    Eigen::Matrix4f matTransform(Eigen::MatrixXf::Identity(4,4));
    matTransform(0,3) = translation[0];
    matTransform(1,3) = translation[1];
    matTransform(2,3) = translation[2];

    double phi = zyzEulerAngle[0];
    double theta = zyzEulerAngle[1];
    double psi = zyzEulerAngle[2];

    matTransform(0,0) = std::cos(phi)*std::cos(theta)*std::cos(psi) -
            std::sin(phi)*std::sin(psi);
    matTransform(0,1) = -std::cos(phi)*std::cos(theta)*std::sin(psi) -
            std::sin(phi)*std::cos(psi);
    matTransform(0,2) = std::cos(phi)*std::sin(theta);

    matTransform(1,0) = std::sin(phi)*std::cos(theta)*std::cos(psi) +
            std::cos(phi)*std::sin(psi);
    matTransform(1,1) = -std::sin(phi)*std::cos(theta)*std::sin(psi) +
            std::cos(phi)*std::cos(psi);
    matTransform(1,2) = std::sin(phi)*std::sin(theta);

    matTransform(2,0) = -std::sin(theta)*std::cos(psi);
    matTransform(2,1) = std::sin(theta)*std::sin(psi);
    matTransform(2,2) = std::cos(theta);

    Pose pose;
    pose.setTransformMatrix(matTransform);

    return pose;
  }

} // namespace radi
