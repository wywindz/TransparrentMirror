#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

#include "cvs.h"

namespace radi
{
  CVSFeature::CVSFeature (pcl::PointXYZ corner)
          : corner_(corner), edge_vectors_(std::vector<Eigen::Vector3f>())
  { }

  CVSFeature::CVSFeature (pcl::PointXYZ corner, std::vector<Eigen::Vector3f> vectors)
          : corner_(corner), edge_vectors_(vectors)
  { }

  CVSFeature::~CVSFeature ()
  { }

  const pcl::PointXYZ
  CVSFeature::getCorner ()
  {
    return (corner_);
  }

  const Eigen::Vector3f
  CVSFeature::getCornerPosition () const
  {
    return (Eigen::Vector3f(corner_.x, corner_.y, corner_.z));
  }

  void
  CVSFeature::appendVector (const Eigen::Vector3f &vector)
  {
    edge_vectors_.push_back(vector);
  }

  void
  CVSFeature::compute ()
  {
    int nCountTotal;
    if (edge_vectors_.size() == 2)
      nCountTotal = 1;
    else
      nCountTotal = edge_vectors_.size();

    Eigen::Vector3f center_vector = Eigen::Matrix3Xf::Zero(3,1);
    for (std::size_t i = 0; i < edge_vectors_.size(); ++i)
    {
      center_vector += edge_vectors_[i];
    }
    center_vector /= std::sqrt(center_vector.dot(center_vector));

    std::vector<Eigen::Vector3f> normal_list(edge_vectors_.size());
    for (std::size_t i = 0; i < edge_vectors_.size(); ++i)
    {
      normal_list[i] = edge_vectors_[i].cross(center_vector);
      normal_list[i] /= std::sqrt(normal_list[i].dot(normal_list[i]));
    }

    std::vector<float> angle_plane_list(edge_vectors_.size());
    angle_plane_list[0] = 0.0;
    for (std::size_t i = 1; i < edge_vectors_.size(); ++i)
    {
      float vector_included_angle = std::acos(normal_list[0].dot(normal_list[i]));
      if ((normal_list[0].cross(normal_list[i])).dot(center_vector) > 0.0)
      {
        angle_plane_list[i] = 2*3.1415926 - vector_included_angle;
      }
      else
      {
        angle_plane_list[i] = vector_included_angle;
      }
    }

    // Sort.
    std::vector<std::size_t> order_indices(angle_plane_list.size());
    std::iota(order_indices.begin (), order_indices.end (), 0);
    std::sort(order_indices.begin(), order_indices.end(),
              [&angle_plane_list](int idx_1, int idx_2){ return angle_plane_list[idx_1] < angle_plane_list[idx_2]; });

    for (std::size_t i = 0; i < order_indices.size()-1; ++i)
    {
      std::vector<int> pair_indicies(2);
      pair_indicies[0] = order_indices[i];
      pair_indicies[1] = order_indices[i+1];
      angle_list_.push_back(std::acos(edge_vectors_[pair_indicies[0]].dot(edge_vectors_[pair_indicies[1]])));
      indices_list_.push_back(pair_indicies);
    }

    if (nCountTotal > 1)
    {
      std::vector<int> pair_indicies(2);
      pair_indicies[0] = order_indices[nCountTotal-1];
      pair_indicies[1] = order_indices[0];
      angle_list_.push_back(std::acos(edge_vectors_[pair_indicies[0]].dot(edge_vectors_[pair_indicies[1]])));
      indices_list_.push_back(pair_indicies);
    }
  }

  const Eigen::Vector3f
  CVSFeature::getVector (std::size_t index) const
  {
    return (edge_vectors_[index]);
  }

  const std::vector<Eigen::Vector3f> &
  CVSFeature::getVectors () const
  {
    return (edge_vectors_);
  }

  const std::vector<float> &
  CVSFeature::getIncludedAngles() const
  {
    return (angle_list_);
  }

  const std::vector<std::vector<int> > &
  CVSFeature::getIndexPairs() const
  {
    return (indices_list_);
  }

  std::size_t
  CVSFeature::getNumEdges() const
  {
    return (edge_vectors_.size());
  }

  const std::vector<CVSFeature>
  refineCVSFeatureList (const std::vector<CVSFeature> & cvsFeatureList)
  {

  }

  void
  transformCVSFeature (const Eigen::Matrix4f & mat_transf,
          const CVSFeature & source_feature, CVSFeature & target_feature)
  {
    Eigen::Vector3f pos_corner = source_feature.getCornerPosition();
    Eigen::Vector3f pos_transformed = mat_transf.block(0,0,3,3)*pos_corner + mat_transf.block(0,3,3,1);
    target_feature.setCorner(pcl::PointXYZ(pos_transformed[0], pos_transformed[1], pos_transformed[2]));

    for (std::size_t idx_vector = 0; idx_vector < source_feature.getNumEdges(); ++idx_vector)
    {
      Eigen::Vector3f vect_transformed = mat_transf.block(0,0,3,3) * source_feature.getVector(idx_vector);
      target_feature.appendVector(vect_transformed);
    }
  }

  bool
  isInList(int index, std::vector<int> index_list)
  {
    if (index_list.empty())
    {
      return (true);
    }
    else
    {
      for (std::size_t i = 0; i < index_list.size(); ++i)
      {
        if (index == index_list[i])
          return (true);
      }

      return (false);
    }
  }

} // namespace radi
