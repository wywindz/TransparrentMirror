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

      const Eigen::Vector3f
      getVector (std::size_t index) const;

      const std::vector<Eigen::Vector3f> &
      getVectors () const;

      std::vector<float>
      getIncludedAngles ();

      std::size_t
      getNumEdges ();

    private:
      pcl::PointXYZ corner_;
      std::vector<Eigen::Vector3f> edge_vectors_;
  };

  // Refine CVS feature list, for example, remove extra features which are two close with each other or remove features
  // which have smaller edge number than the threshold.
  const std::vector<CVSFeature> refineCVSFeatureList(const std::vector<CVSFeature> & cvsFeatureList);

} // namespace radi

#endif // MIRROR_CVS_H_
