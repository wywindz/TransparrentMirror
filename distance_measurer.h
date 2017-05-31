// Calculate the distance from a point to the meshes of an stl file.

#ifndef __MIRROR_DISTANCE_H_
#define __MIRROR_DISTANCE_H_

#include <vector>
#include <Eigen/Dense>
#include <cuda.h>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
// #include <pcl/gpu/containers/device_array.h>

namespace radi {

  class DistanceMeasurer
  {
    public:
      DistanceMeasurer();
      ~DistanceMeasurer();

      void
      setNumTriangles (int num_triangles);

      void
      setTriangles (const std::vector<std::vector<Eigen::Vector3f> > & triangles);

      /*!
       * \fn float calShortestDistance (const float * point);
       * \brief Calculate the shortest distance from the given point to triangle meshes in a STL file.
       * \param[in] point Pointer that points to the array storing the coordinates of the point.
       * \return The shortest distance from the given point to triangle meshes in a STL file.
       */
      float
      calShortestDistance (const float * point);

    private:
      int num_triangles_;
      int * dev_num_triangles_;
      float * dev_triangles_;

  };

  /*!
   * \fn __global__ void distPointTriangle (float * dev_point, const float * dev_triangles, int * dev_num_triangles, float * dev_distances);
   * \brief Calculate the distance from the given point to a triangle mesh. NOTICE! The coordinates of the triangle
   *        vertices of all the meshes are stored in a single array.
   * \param[in] dev_point Coordinates of the point.
   * \param[in] dev_triangles Coordinates of vertices of all the triangle meshes in the STL file. Notice that this is a
   *            1-Dim array with (9*dev_num_triangle_meshes) elements. Every 9 elements represent the coordinates of the 3 vertices
   *            in a single mesh.
   * \param[in] dev_num_triangles Number of the triangle meshes in the STL file.
   * \param[out] dev_distances Ditances from the given point to each triangle mesh.
   */
  __global__ void
  distPointTriangle (float * dev_point, float * dev_triangles, int * dev_num_triangles, float * dev_distances);

  // Calculate the projected point on a plane.
  __device__ void
  pointProjectionOnPlane (const float * dev_point, const float * dev_triangle_vertices, float * dev_point_projection);

  // Detect if the projection of a point is inside the triangle.
  __device__ bool
  isPointInTriangle (const float * dev_point, const float * dev_triangle_vertices);

  // Calculate the distance from a point to another point.
  __device__ float
  distPointPoint (const float * dev_point_a, const float * dev_point_b);

  // Calculate the distance from a point to a line segment.
  __device__ float
  distPointLineSegment (const float * dev_point, const float * dev_segment_vertices);

  // Calculate the distance from a point to a plane.
  __device__ float
  distPointPlane (const float * dev_point, const float * dev_triangle_vertices);
}

#endif
