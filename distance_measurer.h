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

      /*
      // Calculate the distance from a point to a triangle mesh.
      __global__ float
      distPointTriangle (const thrust::device_vector<float> & point, thrust::device_vector<float> & dev_distances);

      // Calculate the projected point on a plane.
      __device__ thrust::device_vector
      pointProjectionOnPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);

      // Detect if the projection of a point is inside the triangle.
      __device__ bool
      isPointInTriangle (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);
      // Calculate the distance from a point to another point.

      __device__ float
      distPointPoint (const Eigen::Vector3f & point_a, const Eigen::Vector3f & point_b);
      // Calculate the distance from a point to a line segment.

      __device__ float
      distPointLineSegment (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & segment_vertices);
      // Calculate the distance from a point to a plane.

      __device__ float
      distPointPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices);

      */
  };

}

#endif
