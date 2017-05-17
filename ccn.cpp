#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

#include "ccn.h"

namespace radi
{
  CCNFeature::CCNFeature () : center_(), normal_()
  { }

  CCNFeature::~CCNFeature ()
  { }

  void
  CCNFeature::setCenter (const Eigen::Vector3f & center)
  {
    center_ = center;
  }

  void
  CCNFeature::setNormal (const Eigen::Vector3f & normal)
  {
    normal_ = normal;
  }

  void
  CCNFeature::setRadius (float radius)
  {
    radius_ = radius;
  }

  const Eigen::Vector3f &
  CCNFeature::getCenter () const
  {
    return (center_);
  }

  const Eigen::Vector3f &
  CCNFeature::getNormal () const
  {
    return (normal_);
  }

  float
  CCNFeature::getRadius () const
  {
    return (radius_);
  }

  void
  transformCCNFeature (const Eigen::Matrix4f & mat_transf, const CCNFeature & source_feature, CCNFeature & target_feature)
  {
    const Eigen::Vector3f & center = source_feature.getCenter ();
    const Eigen::Vector3f & normal = source_feature.getNormal ();
    Eigen::Vector3f center_transformed = mat_transf.block (0,0,3,3)*center + mat_transf.block (0,3,3,1);
    Eigen::Vector3f normal_transformed = mat_transf.block (0,0,3,3)*normal;
    target_feature.setCenter (center_transformed);
    target_feature.setNormal (normal_transformed);
    target_feature.setRadius (source_feature.getRadius ());
  }

} // namespace radi
