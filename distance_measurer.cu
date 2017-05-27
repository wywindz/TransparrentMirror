#include <thrust/sort.h>
#include <thrust/functional.h>

#include "distance_measurer.h"

namespace radi {

  DistanceMeasurer::DistanceMeasurer ()
  {

  }

  DistanceMeasurer::~DistanceMeasurer ()
  {

  }

  void
  DistanceMeasurer::setTriangles (const std::vector<std::vector<float> > triangles)
  {
    dev_triangles_ = triangles;
    dev_distances_ = thrust::device_vector<float> (triangles.size ());
  }

  float
  DistanceMeasurer::calShortestDistance (const std::vector<float> point)
  {
    thrust::device_vector = point;

    distancePointTriangle<<<(dev_triangles_.size () + 127)/128, 128>>> (point, dev_distances_);

    return (0.0);
  }


  // Calculate the distance from a point to a triangle mesh.
  __global__ float
  DistanceMeasurer::distPointTriangle (const thrust::device_vector<float> & point, thrust::device_vector<float> & dev_distances)
  {
    return (0.0);
    // int tid = threadIdx.x + blockDim.x * blockIdx.x;

    // if (tid < dev_triangles_.size ())
    // {
    //   if (isPointInTriangle (point, dev_triangles_[tid]))
    //   {
    //     return (distPointPlane (point, dev_triangles_[tid]));
    //   }
    //   else
    //   {
    //     // Calculate the distance from the point to the vertices and segments.
    //     thrust::vector<float> distances (6);
    //     distances[0] = distPointPoint (point, dev_triangles_[tid][0]);
    //     distances[1] = distPointPoint (point, dev_triangles_[tid][1]);
    //     distances[2] = distPointPoint (point, dev_triangles_[tid][2]);

    //     thrust::vector<thrust::vector<float> > segment_vertices (2);
    //     segment_vertices[0] = dev_triangles_[tid][0];
    //     segment_vertices[1] = dev_triangles_[tid][1];
    //     distances[3] = distPointLineSegment (point, segment_vertices);
    //     segment_vertices[0] = dev_triangles_[tid][0];
    //     segment_vertices[1] = dev_triangles_[tid][2];
    //     distances[4] = distPointLineSegment (point, segment_vertices);
    //     segment_vertices[0] = dev_triangles_[tid][1];
    //     segment_vertices[1] = dev_triangles_[tid][2];
    //     distances[5] = distPointLineSegment (point, segment_vertices);

    //     thrust::stable_sort (distances.begin (), distances.end (), thrust::less<float> ());
    //     return (distances[0]);
    //   }

    // }

  }

  /*
  // Calculate the projected point on a plane.
  __device__ thrust::device_vector
  DistanceMeasurer::pointProjectionOnPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    return (thrust::device_vector ());
  }

  // Detect if the projection of a point is inside the triangle.
  __device__ bool
  DistanceMeasurer::isPointInTriangle (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    return (true);
  }

  // Calculate the distance from a point to another point.
  __device__ float
  DistanceMeasurer::distPointPoint (const Eigen::Vector3f & point_a, const Eigen::Vector3f & point_b)
  {
    return (0.0);
  }

  // Calculate the distance from a point to a line segment.

  __device__ float
  DistanceMeasurer::distPointLineSegment (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & segment_vertices)
  {
    return (0.0);
  }

  // Calculate the distance from a point to a plane.

  __device__ float
  DistanceMeasurer::distPointPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    return (0.0);
  }

  */

}
