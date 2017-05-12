#include <numeric>
#include <boost/range/numeric.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Eigenvalues>

#include <pcl/octree/octree_search.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/extract_indices.h>

// Test
#include <pcl/visualization/pcl_visualizer.h>

#include "board_detector.h"

namespace radi
{

  float
  calMaxIncludedAngle(const std::vector<Eigen::Vector3f> & vectors, const Eigen::Vector3f & normal);

  BoardDetector::BoardDetector() : k_(10)
  { }

  BoardDetector::~BoardDetector()
  { }

  void
  BoardDetector::setInputCloud (const PointCloudConstPtr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  void
  BoardDetector::compute (std::vector<int> & board_point_indices)
  {
    std::vector<float> mu_list (point_cloud_->size ());   // Average distance from the point to its neighbors.
    std::vector<float> curvature_list (point_cloud_->size ());   // Curvature at each point.
    std::vector<Eigen::Vector3f> omega_cr_list (point_cloud_->size ());
    std::vector<Eigen::Vector3f> omega_b_1_list (point_cloud_->size ());
    std::vector<float> omega_b_2_list (point_cloud_->size ());
    std::vector<float> omega_co_list (point_cloud_->size ());
    std::vector<std::vector<int> > neighbor_indices_list (point_cloud_->size ());

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.01);
    octree.setInputCloud (point_cloud_);
    octree.addPointsFromInputCloud ();
    for (std::size_t idx_point = 0; idx_point < point_cloud_->size (); ++idx_point)
    {
      const pcl::PointXYZ & point = (*point_cloud_)[idx_point];
      Eigen::Vector3f point_p (point.x, point.y, point.z);
      // Search 'k' points around the 'point'.
      // The first point of the neighborhood is the query point.
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      octree.nearestKSearch(point, k_, neighbor_indices, neighbor_distances);
      neighbor_indices_list[idx_point] = neighbor_indices;

      if (neighbor_indices.size () > 1)
      {
        mu_list[idx_point] = boost::accumulate(neighbor_distances, 0.0) / neighbor_indices.size();
        // Calculate the center location of the neighborhood.
        Eigen::Vector3f center (0.0, 0.0, 0.0);
        for (std::size_t idx_neighbor = 0; idx_neighbor < neighbor_indices.size (); ++idx_neighbor)
        {

          const pcl::PointXYZ & point_neighbor = (*point_cloud_)[neighbor_indices[idx_neighbor]];
          Eigen::Vector3f point_q (point_neighbor.x, point_neighbor.y, point_neighbor.z);
          center += point_q;
        }
        center /= neighbor_indices.size();

        // Calculate correlation matrix.
        Eigen::Matrix3f correlation_matrix = Eigen::MatrixXf::Zero (3 ,3);
        for (std::size_t idx_neighbor = 0; idx_neighbor < neighbor_indices.size(); ++idx_neighbor)
        {
          const pcl::PointXYZ & point_neighbor = (*point_cloud_)[neighbor_indices[idx_neighbor]];
          Eigen::Vector3f point_q (point_neighbor.x, point_neighbor.y, point_neighbor.z);
          Eigen::Vector3f vect_cq = point_q - center;
          correlation_matrix += vect_cq * Eigen::Transpose<Eigen::Vector3f>(vect_cq);
        }

        // Calculate eigen values and eigen vectors of the correlation matrix.
        Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver;
        eigen_solver.compute(correlation_matrix);
        std::vector<float> eigen_values(3);
        eigen_values[0] = eigen_solver.eigenvalues()[0].real();
        eigen_values[1] = eigen_solver.eigenvalues()[1].real();
        eigen_values[2] = eigen_solver.eigenvalues()[2].real();
        std::vector<Eigen::Vector3f> eigen_vectors(3);
        eigen_vectors[0] = eigen_solver.eigenvectors().col(0).real();
        eigen_vectors[1] = eigen_solver.eigenvectors().col(1).real();
        eigen_vectors[2] = eigen_solver.eigenvectors().col(2).real();

        // Sort the eigen values.
        std::vector<std::size_t> order_indices(3);
        std::iota(order_indices.begin (), order_indices.end (), 0);
        std::sort(order_indices.begin(), order_indices.end(),
                  [&eigen_values](int idx_1, int idx_2){ return eigen_values[idx_1] < eigen_values[idx_2]; });
        float lambda_0 = eigen_values[order_indices[0]];
        float lambda_1 = eigen_values[order_indices[1]];
        float lambda_2 = eigen_values[order_indices[2]];
        Eigen::Vector3f eig_vector_0 = eigen_vectors[order_indices[0]];
        Eigen::Vector3f eig_vector_1 = eigen_vectors[order_indices[1]];
        Eigen::Vector3f eig_vector_2 = eigen_vectors[order_indices[2]];

        // Calculate curvature.
        float distance = std::abs (eig_vector_0.dot(point_p-center));
        curvature_list[idx_point] = 2 * distance / (std::pow (mu_list[idx_point], 2));
        // Calculate penalty function.
        omega_cr_list[idx_point] = std::max (lambda_1-lambda_0, std::abs (lambda_2-lambda_1-lambda_0))
                / lambda_2 * eig_vector_2;
        omega_co_list[idx_point] = (lambda_2 - lambda_0) / lambda_2;

        omega_b_1_list[idx_point] = std::abs(lambda_2 - 2.0*lambda_1) / lambda_2 * eig_vector_2;

        // Calculate beta.
        std::vector<Eigen::Vector3f> project_vectors(neighbor_indices.size () - 1);
        for (std::size_t idx_neighbor = 1; idx_neighbor < neighbor_indices.size(); ++idx_neighbor)
        {
          const pcl::PointXYZ & point_neighbor = (*point_cloud_)[neighbor_indices[idx_neighbor]];
          Eigen::Vector3f point_q (point_neighbor.x, point_neighbor.y, point_neighbor.z);
          Eigen::Vector3f vect_pq = point_q - point_p;
          project_vectors[idx_neighbor-1] = vect_pq - vect_pq.dot(eig_vector_0)/eig_vector_0.dot(eig_vector_0) * eig_vector_0;
        }

        float beta = calMaxIncludedAngle(project_vectors, eig_vector_0);
        omega_b_2_list[idx_point] = 1.0 - beta / (2.0*3.14151926);

      }
    }

    // Calculate omega_k_list.
    std::vector<float> omega_k_list (point_cloud_->size ());
    float curvature_max = *(boost::max_element (curvature_list));
    for (std::size_t idx_point = 0; idx_point < point_cloud_->size (); ++idx_point)
    {
      omega_k_list[idx_point] = 1.0 - curvature_list[idx_point]/curvature_max;
    }

    // Detect edge.
    float tau = 0.7;
    float gamma = 0.5;
    for (std::size_t idx_point = 0; idx_point < point_cloud_->size (); ++idx_point)
    {
      if (omega_b_2_list[idx_point] < tau)
      {
        const pcl::PointXYZ & point = (*point_cloud_)[idx_point];
        Eigen::Vector3f point_p (point.x, point.y, point.z);
        std::vector<std::vector<int> > queue;
        std::vector<float> queue_weights;
        for (std::size_t idx_neighbor = 1; idx_neighbor < neighbor_indices_list[idx_point].size (); ++idx_neighbor)
        {
          std::size_t idx_neighbor_point = neighbor_indices_list[idx_point][idx_neighbor];
          if (omega_cr_list[idx_neighbor_point].norm() < tau)
          {
            const pcl::PointXYZ & point_neighbor = (*point_cloud_)[idx_neighbor_point];
            Eigen::Vector3f point_q (point_neighbor.x, point_neighbor.y, point_neighbor.z);
            Eigen::Vector3f vect_pq = point_q - point_p;
            std::vector<int> pair(2);
            pair[0] = idx_point;
            pair[1] = idx_neighbor_point;
            queue.push_back (pair);
            float weight_boarder = gamma*(omega_b_2_list[idx_point] + omega_b_2_list[idx_neighbor_point])
                + (1.0-gamma)*(std::abs (omega_b_1_list[idx_point].dot(vect_pq/vect_pq.norm ()))
                + std::abs (omega_b_1_list[idx_neighbor_point].dot(vect_pq/vect_pq.norm())))
                + 2.0*vect_pq.norm()/(mu_list[idx_point]+mu_list[idx_neighbor_point]);
            queue_weights.push_back (weight_boarder);
          }
        }

        if (!queue.empty())
        {
          // std::cout << "Candidates in the queue: " << queue.size() << std::endl;
          std::vector<std::size_t> order_edge (queue.size ());
          std::iota (order_edge.begin (), order_edge.end (), 0);
          std::sort(order_edge.begin(), order_edge.end(),
                    [&queue_weights](int idx_1, int idx_2){ return queue_weights[idx_1] < queue_weights[idx_2]; });

          board_point_indices.push_back(queue[order_edge[0]][0]);
          board_point_indices.push_back(queue[order_edge[0]][1]);
        }
      }
    }

    // Remove duplicate indices.
    std::set<int> edge_point_indices_set(board_point_indices.begin(), board_point_indices.end());
    board_point_indices = std::vector<int>(edge_point_indices_set.begin(), edge_point_indices_set.end());
  }

  float
  calMaxIncludedAngle (const std::vector<Eigen::Vector3f> & vectors, const Eigen::Vector3f & normal)
  {
    // Choose the first vector as X axis.
    Eigen::Vector3f x_axis = vectors[0] / vectors[0].norm ();
    Eigen::Vector3f y_axis = (normal/normal.norm ()).cross (x_axis);

    std::vector<float> included_angles (vectors.size ());
    for (int idx_vector = 0; idx_vector < vectors.size (); ++idx_vector)
    {
      float x_comp = vectors[idx_vector].dot (x_axis) / x_axis.dot (x_axis);
      float y_comp = vectors[idx_vector].dot (y_axis) / y_axis.dot (y_axis);
      included_angles[idx_vector] = std::atan2 (y_comp, x_comp);
      if (included_angles[idx_vector] < 0)
      {
        included_angles[idx_vector] += 2*3.1415926;
      }
    }

    // Sort the included angles.
    std::vector<std::size_t> order_indices (included_angles.size ());
    std::iota (order_indices.begin (), order_indices.end (), 0);
    std::sort (order_indices.begin(), order_indices.end(),
        [&included_angles](int idx_1, int idx_2){ return included_angles[idx_1] < included_angles[idx_2]; });

    float beta = 0.0;
    for (int idx_order = 0; idx_order < order_indices.size () - 1 ; ++idx_order)
    {
      float angle = included_angles[order_indices[idx_order+1]] - included_angles[order_indices[idx_order]];
      if (angle > beta)
      {
        beta = angle;
      }
    }

    return (beta);
  }

} // namespace radi
