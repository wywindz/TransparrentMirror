/*
 * Iterative Closest Face (ICF) algorithm.
 */

#ifndef MIRROR_MESH_H_
#define MIRROR_MESH_H_

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <eigen3/Eigen/Dense>

namespace radi
{
  class Mesh
  {
    public:
      Mesh ();
      ~Mesh ();

      void
      loadModel(const std::string & filePath);

      inline std::size_t
      getNumTriangles () const { return triangles.size(); }

      const std::vector<Eigen::Vector3f> &
      getTriangle (std::size_t index) const;

    private:
      std::vector<std::vector<Eigen::Vector3f> > triangles;
  }; // class Mesh

} // namespace radi

#endif
