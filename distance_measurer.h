// Calculate the distance from a point to the meshes of an stl file.

#ifndef __MIRROR_DISTANCE_H_
#define __MIRROR_DISTANCE_H_

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
      setTriangles (const std::vector<std::vector<float> > triangles);

      float
      calShortestDistance (const std::vector<float> point);

    private:
      thrust::device_vector<thrust::device_vector<float> > dev_triangles_;
      thrust::device_vector<float> dev_distances_;

  };

  // Calculate the distance from a point to a triangle mesh.
  /*!
   * \fn __global__ void distPointTriangle (const float * dev_point, const float ** dev_triangles, float * dev_distances);
   * \brief Calculate the distance from the given point to a triangle mesh. NOTICE! The coordinates of the triangle
   *        vertices are stored in a single array, i.e., 9 numbers are stored in one array.
   * \param[in] dev_point Coordinates of the point.
   * \param[in] dev_triangles Triangle meshes in the stl file. Notice that this is a 2-Dim array. The outer
   *            contains num_triangle_meshes elements, and the inner contains 9 elements which store the coordinates of
   *            the 3 vertices in one triangle mesh sequentially.
   * \param[out] dev_distances Ditances from the given point to each triangle mesh.
   */
  __global__ void
  distPointTriangle (const float * dev_point, const float ** dev_triangles, const int & dev_num_triangles, float * dev_distances);

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
