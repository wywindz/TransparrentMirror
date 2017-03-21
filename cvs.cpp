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

  void
  CVSFeature::appendVector (const Eigen::Vector3f &vector)
  {
    edge_vectors_.push_back(vector);
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
  CVSFeature::getNumEdges()
  {
    return (edge_vectors_.size());
  }

  const std::vector<CVSFeature>
  refineCVSFeatureList (const std::vector<CVSFeature> & cvsFeatureList)
  {

  }

} // namespace radi
