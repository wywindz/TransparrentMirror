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
      loadModel (const std::string & file_path);

      inline int
      getNumTriangles () const { return triangles_.size(); }

      const std::vector<Eigen::Vector3f> &
      getTriangle (std::size_t index) const;

    private:
      std::vector<std::vector<Eigen::Vector3f> > triangles_;

  }; // class Mesh

} // namespace radi

#endif
