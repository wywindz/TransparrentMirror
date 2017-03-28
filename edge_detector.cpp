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

#include "edge_detector.h"

namespace radi
{

  EdgeDetector::EdgeDetector() : radius_(0.5), k_(20)
  { }

  EdgeDetector::~EdgeDetector()
  { }

  void
  EdgeDetector::setInputCloud (const PointCloudConstPtr & point_cloud)
  {
    point_cloud_ = point_cloud;
  }

  void
  EdgeDetector::setRadius (float radius)
  {
    radius_ = radius;
  }

  void
  EdgeDetector::setK (std::size_t k)
  {
    k_ = k;
  }

  void
  EdgeDetector::compute (std::vector<int> & edge_point_indices)
  {
    std::vector<float> mu_list (point_cloud_->size ());   // Average distance from the point to its neighbors.
    std::vector<float> curvature_list (point_cloud_->size ());   // Curvature at each point.
    std::vector<Eigen::Vector3f> omega_cr_list (point_cloud_->size ());
    std::vector<float> omega_co_list (point_cloud_->size());
    std::vector<std::vector<int> > neighbor_indices_list(point_cloud_->size());

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.01);
    octree.setInputCloud (point_cloud_);
    octree.addPointsFromInputCloud();
    for (std::size_t idx_point = 0; idx_point < point_cloud_->size (); ++idx_point)
    {
      const pcl::PointXYZ & point = (*point_cloud_)[idx_point];
      Eigen::Vector3f point_p((*point_cloud_)[idx_point].x, (*point_cloud_)[idx_point].y, (*point_cloud_)[idx_point].z);
      // Search 'K' points around the 'point'.
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      octree.nearestKSearch(point, k_, neighbor_indices, neighbor_distances);
      neighbor_indices_list[idx_point] = neighbor_indices;

      if (neighbor_indices.size ())
      {
        mu_list[idx_point] = boost::accumulate(neighbor_distances, 0.0) / neighbor_indices.size();
        // Calculate the center location of the neighborhood.
        Eigen::Vector3f center (0.0, 0.0, 0.0);
        for (std::size_t idx_neighbor = 0; idx_neighbor < neighbor_indices.size (); ++idx_neighbor)
        {
          Eigen::Vector3f point_q = Eigen::Vector3f((*point_cloud_)[neighbor_indices[idx_neighbor]].x,
                  (*point_cloud_)[neighbor_indices[idx_neighbor]].y, (*point_cloud_)[neighbor_indices[idx_neighbor]].z);
          center += point_q;
        }
        center /= neighbor_indices.size();

        // Calculate correlation matrix.
        Eigen::Matrix3f correlation_matrix = Eigen::MatrixXf::Zero (3 ,3);
        for (std::size_t idx_neighbor = 0; idx_neighbor < neighbor_indices.size(); ++idx_neighbor)
        {
          Eigen::Vector3f point_q = Eigen::Vector3f((*point_cloud_)[neighbor_indices[idx_neighbor]].x,
                  (*point_cloud_)[neighbor_indices[idx_neighbor]].y, (*point_cloud_)[neighbor_indices[idx_neighbor]].z);
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

        // if (omega_cr_list[idx_point].norm() < 0.5) {
        //   std::cout <<  "Found" << std::endl;
        // }
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
    float tau = 0.5;
    float alpha = 0.5;
    for (std::size_t idx_point = 0; idx_point < point_cloud_->size (); ++idx_point)
    {
      if (omega_cr_list[idx_point].norm () < tau)
      {
        // std::cout << "Number of points in the neighborhood: " << neighbor_indices_list[idx_point].size () << std::endl;
        Eigen::Vector3f point_p((*point_cloud_)[idx_point].x, (*point_cloud_)[idx_point].y, (*point_cloud_)[idx_point].z);
        std::vector<std::vector<int> > queue;
        std::vector<float> queue_weights;
        for (std::size_t idx_neighbor = 0; idx_neighbor < neighbor_indices_list[idx_point].size (); ++idx_neighbor)
        {
          std::size_t idx_neighbor_point = neighbor_indices_list[idx_point][idx_neighbor];
          if (omega_cr_list[idx_neighbor_point].norm() < tau)
          {
            Eigen::Vector3f point_q((*point_cloud_)[idx_neighbor_point].x, (*point_cloud_)[idx_neighbor_point].y,
                                    (*point_cloud_)[idx_neighbor_point].z);
            Eigen::Vector3f vect_pq = point_q - point_p;
            queue.push_back (std::vector<int> (idx_point, idx_neighbor_point));
            float weight = alpha*(omega_k_list[idx_point] + omega_k_list[idx_neighbor_point])
                + (1-alpha)*(std::min (std::abs (omega_cr_list[idx_point].dot(vect_pq/vect_pq.norm ())), omega_co_list[idx_point])
                + std::min (std::abs (omega_cr_list[idx_point].dot(vect_pq/vect_pq.norm ())), omega_co_list[idx_neighbor_point]))
                + 2*vect_pq.norm()/(mu_list[idx_point]+mu_list[idx_neighbor_point]);
            queue_weights.push_back (weight);
          }
        }
        // if (queue.size())
        //   std::cout << "Candidates in the queue: " << queue.size() << std::endl;

        std::vector<std::size_t> order_edge (queue.size ());
        std::iota (order_edge.begin (), order_edge.end (), 0);
        std::sort(order_edge.begin(), order_edge.end(),
                  [&queue_weights](int idx_1, int idx_2){ return queue_weights[idx_1] < queue_weights[idx_2]; });
        edge_point_indices.push_back(queue[order_edge[0]][0]);
        edge_point_indices.push_back(queue[order_edge[0]][1]);
      }
    }

    // Remove duplicate indices.
    std::set<int> edge_point_indices_set(edge_point_indices.begin(), edge_point_indices.end());
    edge_point_indices = std::vector<int>(edge_point_indices_set.begin(), edge_point_indices_set.end());
  }

} // namespace radi
