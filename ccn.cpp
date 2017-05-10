#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

#include "ccn.h"

namespace radi
{
  CCNFeature::CCNFeature ()
          : center_list_(std::vector<Eigen::Vector3f>()), normal_list_(std::vector<Eigen::Vector3f>())
  { }

  CCNFeature::~CCNFeature ()
  { }

  void
  CCNFeature::append (const Eigen::Vector3f & center, const Eigen::Vector3f & normal)
  {
    center_list_.push_back(center);
    normal_list_.push_back(normal);
  }

  const std::vector<Eigen::Vector3f> &
  CCNFeature::getCenterList() const
  {
    return (center_list_);
  }

  const std::vector<Eigen::Vector3f> &
  CCNFeature::getNormalList() const
  {
    return (normal_list_);
  }

  void
  transformCCNFeature (const Eigen::Matrix4f & mat_transf,
          const CCNFeature & source_feature, CCNFeature & target_feature)
  {
    const std::vector<Eigen::Vector3f> & center_list = source_feature.getCenterList();
    const std::vector<Eigen::Vector3f> & normal_list = source_feature.getNormalList();
    for (std::size_t idx = 0; idx < center_list.size(); ++idx)
    {
      Eigen::Vector3f center_transformed = mat_transf.block(0,0,3,3)*center_list[idx] + mat_transf.block(0,3,3,1);
      Eigen::Vector3f normal_transformed = mat_transf.block(0,0,3,3)*normal_list[idx];
      target_feature.append(center_transformed, normal_transformed);
    }
  }

} // namespace radi
