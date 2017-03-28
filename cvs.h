/*
 * Define Corner Vector Sequence (CVS) Feature as well as the related classes for estimating the feature.
 *
 */

#ifndef MIRROR_CVS_H_
#define MIRROR_CVS_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace radi
{
  class CVSFeature
  {
    public:
      CVSFeature (pcl::PointXYZ corner = pcl::PointXYZ());
      CVSFeature (pcl::PointXYZ corner, std::vector<Eigen::Vector3f> vectors);
      ~CVSFeature ();

      template<typename PointType> void
      setCorner (PointType corner)
      {
        corner_ = pcl::PointXYZ(corner.x, corner.y, corner.z);
      }

      const pcl::PointXYZ
      getCorner();

      const Eigen::Vector3f
      getCornerPosition() const;

      void
      appendVector (const Eigen::Vector3f & vector);

      void
      compute ();

      const Eigen::Vector3f
      getVector (std::size_t index) const;

      const std::vector<Eigen::Vector3f> &
      getVectors () const;

      const std::vector<float> &
      getIncludedAngles () const;

      const std::vector<std::vector<int> > &
      getIndexPairs() const;

      std::size_t
      getNumEdges () const;

    private:
      pcl::PointXYZ corner_;
      std::vector<Eigen::Vector3f> edge_vectors_;
      std::vector<float> angle_list_;
      std::vector<std::vector<int> > indices_list_; // Index pair of edge vectors between which the included angle is calculated.
  };

  // Refine CVS feature list, for example, remove extra features which are two close with each other or remove features
  // which have smaller edge number than the threshold.
  const std::vector<CVSFeature>
  refineCVSFeatureList (const std::vector<CVSFeature> & cvsFeatureList);

  void
  transformCVSFeature (const Eigen::Matrix4f & mat_transf,
          const CVSFeature & source_feature, CVSFeature & target_feature);

  bool
  isInList(int index, std::vector<int> index_list);

} // namespace radi

#endif // MIRROR_CVS_H_
