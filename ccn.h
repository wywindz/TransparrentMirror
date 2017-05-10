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
      append (const Eigen::Vector3f & center, const Eigen::Vector3f & normal);

      const std::vector<Eigen::Vector3f> &
      getCenterList() const;

      const std::vector<Eigen::Vector3f> &
      getNormalList () const;

    private:
      std::vector<Eigen::Vector3f> center_list_;
      std::vector<Eigen::Vector3f> normal_list_;
  };

  void
  transformCCNFeature (const Eigen::Matrix4f & mat_transf,
          const CCNFeature & source_feature, CCNFeature & target_feature);

} // namespace radi

#endif // MIRROR_CVS_H_
