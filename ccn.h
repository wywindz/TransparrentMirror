/*
 * Define Circle Center Normal (CCN) Feature as well as the related classes for estimating the feature.
 *
 */

#ifndef MIRROR_CCN_H_
#define MIRROR_CCN_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace radi
{
  class CCNFeature
  {
    public:
      CCNFeature ();
      ~CCNFeature ();

      void
      setCenter(const Eigen::Vector3f & center);

      void
      setNormal(const Eigen::Vector3f & normal);

      void
      setRadius(float radius);

      const Eigen::Vector3f &
      getCenter() const;

      const Eigen::Vector3f &
      getNormal () const;

      float
      getRadius () const;

    private:
      Eigen::Vector3f center_;
      Eigen::Vector3f normal_;
      float radius_;
  };

  void
  transformCCNFeature (const Eigen::Matrix4f & mat_transf,
          const CCNFeature & source_feature, CCNFeature & target_feature);

} // namespace radi

#endif // MIRROR_CVS_H_
