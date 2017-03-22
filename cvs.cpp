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

    int nCount = 0;
    int index_start = 0;
    int index_previous = -1;
    std::vector<int> index_accumulator;
    while (nCount < nCountTotal)
    {
      Eigen::Vector3f vect_origin = edge_vectors_[index_start];
      std::vector<int> pair_indices(2);
      pair_indices[0] = index_start;
      float included_angle = 3.14;
      for (std::size_t idxI = 0; idxI < this->edge_vectors_.size(); ++idxI)
      {
        // if (idxI != index_previous)
        if (!isInList(idxI, index_accumulator))
        {
          float angle = std::acos(vect_origin.dot(edge_vectors_[idxI]));
          if (angle < included_angle)
          {
            included_angle = angle;
            pair_indices[1] = idxI;
          }
        }
      }

      nCount++;
      index_previous = index_start;
      index_start = pair_indices[1];

      angle_list_.push_back(included_angle);
      indices_list_.push_back(pair_indices);

      if ((nCount != 0) && (nCount == nCountTotal-1))
      {
        nCount++;
        Eigen::Vector3f vect_origin = edge_vectors_[index_start];
        std::vector<int> pair_indices(2);
        pair_indices[0] = index_start;
        pair_indices[1] = indices_list_[0][0];
        angle_list_.push_back(std::acos(vect_origin.dot(edge_vectors_[pair_indices[1]])));
      }
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

  std::vector<float>
  CVSFeature::getIncludedAngles ()
  {
    // ToDO: Calculate the included angles between each pair of edge vectors.
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
