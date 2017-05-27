#include <cmath>
#include <random>
#include <exception>

#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "icf.h"

namespace radi
{
  IterativeClosestFace::IterativeClosestFace ()
  {
    model_file_ = "";
    left_board_translation_ = Eigen::Vector3f (100, 100, 100);
    right_board_translation_ = Eigen::Vector3f (100, 100, 100);
    left_board_rotation_ = Eigen::Vector3f (3.1415926, 3.1415926, 3.1415926);
    right_board_rotation_ = Eigen::Vector3f (3.1415926, 3.1415926, 3.1415926);
    iteration_outer_ = 50;
    iteration_inner_ = 20;
    use_indices_ = false;
    threshold_distance_near_ = 0.03;
    threshold_distance_extreme_ = 0.1;
    threshold_valid_ = 0.4;
    init_transf_ = Eigen::MatrixXf::Identity (4,4);
  }

  IterativeClosestFace::IterativeClosestFace (const std::string & model_file_path,
      const PointCloudConstPtr & scene_point_cloud)
  {
    model_file_ = model_file_path;
    model_mesh_.loadModel (model_file_path);
    scene_point_cloud_ = scene_point_cloud;
    point_cloud_used_ = scene_point_cloud_;
  }

  IterativeClosestFace::~IterativeClosestFace ()
  { }

  void
  IterativeClosestFace::setReferenceModel (const std::string & model_file_path)
  {
    model_file_ = model_file_path;
    model_mesh_.loadModel (model_file_path);
  }

  void
  IterativeClosestFace::setScenePointCloud (const PointCloudConstPtr & scene_point_cloud)
  {
    scene_point_cloud_ = scene_point_cloud;
    point_cloud_used_ = scene_point_cloud_;
  }

  void
  IterativeClosestFace::setIndices (const IndicesConstPtr & indices)
  {
    if (indices->empty())
    {
      indices_ = NULL;
      use_indices_ = false;
      point_cloud_used_ = scene_point_cloud_;
    }
    else
    {
      indices_ = indices;
      use_indices_ = true;
      IterativeClosestFace::PointCloud::Ptr point_cloud (new IterativeClosestFace::PointCloud ());
      pcl::ExtractIndices<pcl::PointXYZ> extractor;
      extractor.setInputCloud (scene_point_cloud_);
      extractor.setIndices (indices_);
      extractor.filter (*point_cloud);

      point_cloud_used_ = point_cloud;
    }
  }

  void
  IterativeClosestFace::setVariationTranslation (const Eigen::Vector3f & left_board, const Eigen::Vector3f & right_board)
  {
    left_board_translation_ = left_board;
    right_board_translation_ = right_board;
  }

  void
  IterativeClosestFace::setVariationRotation (const Eigen::Vector3f & left_board, const Eigen::Vector3f & right_board)
  {
    left_board_rotation_ = left_board;
    right_board_rotation_ = right_board;
  }

  void
  IterativeClosestFace::setIterationOuter (int iteration_outer)
  {
    iteration_outer_ = iteration_outer;
  }

  void
  IterativeClosestFace::setIterationInner (int iteration_inner)
  {
    iteration_inner_ = iteration_inner;
  }

  void
  IterativeClosestFace::setInitialTransformation (const Eigen::Matrix4f & init_transf)
  {
    init_transf_ = init_transf;
  }

  void
  IterativeClosestFace::setThresholds (float near, float extreme, float valid)
  {
    threshold_distance_near_ = near;
    threshold_distance_extreme_ = extreme;
    threshold_valid_ = valid;
  }

  const Mesh &
  IterativeClosestFace::getReferenceModel () const
  {
    return (model_mesh_);
  }

  IterativeClosestFace::PointCloudConstPtr
  IterativeClosestFace::getScenePointCloud () const
  {
    return (scene_point_cloud_);
  }

  void
  IterativeClosestFace::getVariationTranslation (Eigen::Vector3f & left_board, Eigen::Vector3f & right_board)
  {
    left_board = left_board_translation_;
    right_board = right_board_translation_;
  }

  void
  IterativeClosestFace::getVariationRotation (Eigen::Vector3f & left_board, Eigen::Vector3f & right_board)
  {
    left_board = left_board_rotation_;
    right_board = right_board_rotation_;
  }

  int
  IterativeClosestFace::getIterationOuter ()
  {
    return (iteration_outer_);
  }

  int
  IterativeClosestFace::getIterationInner ()
  {
    return (iteration_inner_);
  }

  const Eigen::Matrix4f
  IterativeClosestFace::getInitialTransformation ()
  {
    return (init_transf_);
  }

  float
  IterativeClosestFace::calObjectiveValue (const Eigen::Matrix4f & mat_transf)
  {
    std::cout << "Number of points used: " << this->point_cloud_used_->size () << std::endl;
    std::cout << "Number of triangles: " << this->model_mesh_.getNumTriangles() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scene (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud(*point_cloud_used_, *transformed_scene, mat_transf);

    float objective_value = 0.0;
    for (std::size_t i = 0; i < (*transformed_scene).points.size(); ++i)
    {
      Eigen::Vector3f point;
      point[0] = static_cast<float> ((*transformed_scene).points[i].x);
      point[1] = static_cast<float> ((*transformed_scene).points[i].y);
      point[2] = static_cast<float> ((*transformed_scene).points[i].z);
      std::vector<float> distance_list (model_mesh_.getNumTriangles());
      for (std::size_t j = 0; j < model_mesh_.getNumTriangles (); ++j)
      {
        distance_list[j] = distPointTriangle (point, model_mesh_.getTriangle(j));
      }
      float shortest_distance = *(std::min_element (distance_list.begin (), distance_list.end ()));

      if (shortest_distance > threshold_distance_extreme_)
      {
        std::cout << "Shortest distance: " << shortest_distance << std::endl;
        throw "Too large distance.";
      }

      float gpx;
      if (shortest_distance < threshold_distance_near_)
        gpx = shortest_distance;
      else
          gpx = 4.0 * shortest_distance;

      float fxp;
      if (gpx < threshold_valid_)
          fxp = 1.0 - gpx/threshold_valid_;
      else
          fxp = 0.0;

      objective_value += fxp;
    }

    return (objective_value);
  }


  void
  IterativeClosestFace::estimate (Eigen::Matrix4f & estimated_transf)
  {
    Eigen::Matrix4f mat_transf = init_transf_;
    float objective_value;
    try
    {
      objective_value = calObjectiveValue(mat_transf);
    }
    catch (char const * msg)
    {
      std::cout << "Bad initial transformation. " << msg << std::endl;
      return;
    }

    std::cout << "-- OK --" << std::endl;
    return;

    for (std::size_t idx_outer = 0; idx_outer < iteration_outer_; ++idx_outer)
    {
      Eigen::Vector3f translation = mat_transf.block (0,3,3,1);
      Eigen::Vector3f rotation = matrix2euler (mat_transf.block (0,0,3,3));

      Eigen::Vector3f translation_sampled = uniformRandom (translation-left_board_translation_,
              translation+right_board_translation_);

      std::vector<float> objective_value_list (iteration_inner_);
      std::vector<Eigen::Matrix4f> mat_transf_list (iteration_inner_);
      for (std::size_t idx_inner = 0; idx_inner < iteration_inner_; ++idx_inner)
      {
        Eigen::Vector3f rotation_sampled = uniformRandom (rotation-left_board_rotation_,
                rotation+right_board_rotation_);
        mat_transf_list[idx_inner] = Eigen::Matrix4Xf::Identity (4,4);
        mat_transf_list[idx_inner].block (0,0,3,3) = euler2matrix (rotation_sampled);
        mat_transf_list[idx_inner].block (0,3,3,1) = translation_sampled;

        objective_value_list[idx_inner] = calObjectiveValue (mat_transf_list[idx_inner]);
      }

      // Sort 'objective_value' and select the best sample.
      std::vector<std::size_t> order_indices (iteration_inner_);
      std::iota(order_indices.begin (), order_indices.end (), 0);
      std::sort(order_indices.begin (), order_indices.end (), [&objective_value_list](int idx_1, int idx_2)
              { return objective_value_list[idx_1] >= objective_value_list[idx_2]; });

      if (objective_value_list[order_indices[0]] > objective_value)
      {
        objective_value = objective_value_list[order_indices[0]];
        mat_transf = mat_transf_list[order_indices[0]];
      }
    }

    estimated_transf = mat_transf;
  }

  const Eigen::Vector3f
  uniformRandom (const Eigen::Vector3f & min_boundary, const Eigen::Vector3f & max_boundary)
  {
      Eigen::Vector3f random_value;
      std::random_device rand_device;
      std::mt19937 generator (rand_device());
      std::uniform_real_distribution<float> distr_x (min_boundary[0], max_boundary[0]);
      std::uniform_real_distribution<float> distr_y (min_boundary[1], max_boundary[1]);
      std::uniform_real_distribution<float> distr_z (min_boundary[2], max_boundary[2]);
      random_value[0] = distr_x (generator);
      random_value[1] = distr_y (generator);
      random_value[2] = distr_z (generator);

      return (random_value);
  }

  const Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, const Eigen::Vector3f & deviation)
  {
      Eigen::Vector3f random_value;
      std::random_device rand_device;
      std::mt19937 generator (rand_device());
      std::normal_distribution<float> distr_x (mean[0], deviation[0]);
      std::normal_distribution<float> distr_y (mean[1], deviation[1]);
      std::normal_distribution<float> distr_z (mean[2], deviation[2]);
      random_value[0] = distr_x (generator);
      random_value[1] = distr_y (generator);
      random_value[2] = distr_z (generator);

      return (random_value);
  }

  const Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, float deviation)
  {
      Eigen::Vector3f std_deviation;
      std_deviation[0] = deviation;
      std_deviation[1] = deviation;
      std_deviation[2] = deviation;
      return (gaussianRandom (mean, std_deviation));
  }

  const Eigen::Vector3f matrix2euler (const Eigen::Matrix3f & mat_rotation)
  {
    const float PI = 3.1415926;
    const float EPS = 1.0E-8;

    float alpha;
    float beta;
    float gamma;

    // Assume beta is in [0,pi].
    double a_02 = mat_rotation (0,2);
    double a_01 = mat_rotation (0,1);
    double a_11 = mat_rotation (1,1);
    double a_12 = mat_rotation (1,2);
    double a_20 = mat_rotation (2,0);
    double a_21 = mat_rotation (2,1);
    double a_22 = mat_rotation (2,2);

    beta = std::atan2 (std::sqrt (std::pow (a_02,2)+std::pow (a_12,2)), a_22);

    if ((EPS < beta) && (beta < (PI-EPS)))
    {
      alpha = std::atan2 (a_12, a_02);
      gamma = std::atan2 (a_21, -a_20);
    }
    else if (beta <= EPS)
    {
      alpha = 0.0;
      gamma = std::atan2 (-a_01, a_11);
    }
    else
    {
      alpha = 0.0;
      gamma = std::atan2 (a_01, a_11);
    }

    return (Eigen::Vector3f (alpha, beta, gamma));
  }

  const Eigen::Matrix3f euler2matrix (const Eigen::Vector3f & euler_angle)
  {
    double phi = euler_angle[0];
    double theta = euler_angle[1];
    double psi = euler_angle[2];

    Eigen::Matrix3f mat_rotation;
    mat_rotation (0,0) = std::cos (phi)*std::cos (theta)*std::cos (psi) - std::sin (phi)*std::sin (psi);
    mat_rotation (0,1) = -std::cos (phi)*std::cos (theta)*std::sin (psi) - std::sin (phi)*std::cos (psi);
    mat_rotation (0,2) = std::cos (phi)*std::sin (theta);

    mat_rotation (1,0) = std::sin (phi)*std::cos (theta)*std::cos (psi) + std::cos (phi)*std::sin (psi);
    mat_rotation (1,1) = -std::sin (phi)*std::cos (theta)*std::sin (psi) + std::cos (phi)*std::cos (psi);
    mat_rotation (1,2) = std::sin (phi)*std::sin (theta);

    mat_rotation (2,0) = -std::sin (theta)*std::cos (psi);
    mat_rotation (2,1) = std::sin (theta)*std::sin (psi);
    mat_rotation (2,2) = std::cos (theta);

    return (mat_rotation);
  }

  // Calculate the distance from a point to a triangle mesh.
  float
  distPointTriangle (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    if (isPointInTriangle (point, triangle_vertices))
    {
      return (distPointPlane (point, triangle_vertices));
    }
    else
    {
      // Calculate the distance from the point to the vertices and segments.
      std::vector<float> distances (6);
      distances[0] = distPointPoint (point, triangle_vertices[0]);
      distances[1] = distPointPoint (point, triangle_vertices[1]);
      distances[2] = distPointPoint (point, triangle_vertices[2]);

      std::vector<Eigen::Vector3f> segment_vertices (2);
      segment_vertices[0] = triangle_vertices[0];
      segment_vertices[1] = triangle_vertices[1];
      distances[3] = distPointLineSegment (point, segment_vertices);
      segment_vertices[0] = triangle_vertices[0];
      segment_vertices[1] = triangle_vertices[2];
      distances[4] = distPointLineSegment (point, segment_vertices);
      segment_vertices[0] = triangle_vertices[1];
      segment_vertices[1] = triangle_vertices[2];
      distances[5] = distPointLineSegment (point, segment_vertices);

      return (*(std::min_element (distances.begin (), distances.end ())));
    }
  }

  // Calculate the projected point on a plane.
  Eigen::Vector3f
  pointProjectionOnPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    Eigen::Vector3f vect_ab = triangle_vertices[1] - triangle_vertices[0];
    Eigen::Vector3f vect_ac = triangle_vertices[2] - triangle_vertices[0];
    Eigen::Vector3f normal = vect_ab.cross (vect_ac);
    normal /= std::sqrt (normal.dot (normal));
    Eigen::Vector3f vect_ap = point - triangle_vertices[0];
    Eigen::Vector3f vect_ap_perpend = vect_ap - vect_ap.dot (normal) / normal.dot (normal) * normal;

    return (triangle_vertices[0]+vect_ap_perpend);
  }

  // Detect if the projection of a point is inside the triangle.
  bool
  isPointInTriangle (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    Eigen::Vector3f vect_ab = triangle_vertices[1] - triangle_vertices[0];
    Eigen::Vector3f vect_ac = triangle_vertices[2] - triangle_vertices[0];
    Eigen::Vector3f vect_ap = point - triangle_vertices[0];

    float u = (vect_ac.dot (vect_ac)*vect_ap.dot (vect_ab) - vect_ac.dot (vect_ab)*vect_ap.dot (vect_ac)) /
            (vect_ab.dot (vect_ab)*vect_ac.dot (vect_ac) - vect_ab.dot (vect_ac)*vect_ac.dot (vect_ab));
    float v = (vect_ab.dot (vect_ab)*vect_ap.dot (vect_ac) - vect_ab.dot (vect_ac)*vect_ap.dot (vect_ab)) /
            (vect_ab.dot (vect_ab)*vect_ac.dot (vect_ac) - vect_ab.dot (vect_ac)*vect_ac.dot (vect_ab));

    return ((u >= 0.0) && (v >= 0.0) && (u + v <= 1.0));
  }

  // Calculate the distance from a point to another point.
  float
  distPointPoint (const Eigen::Vector3f & point_a, const Eigen::Vector3f & point_b)
  {
    Eigen::Vector3f vect_ab = point_b - point_a;
    return (vect_ab.norm ());
  }

  // Calculate the distance from a point to a line segment.
  float
  distPointLineSegment (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & segment_vertices)
  {
    Eigen::Vector3f vectV = segment_vertices[1] - segment_vertices[0];
    Eigen::Vector3f vectW = point - segment_vertices[0];
    float scalar_1 = vectW.dot (vectV);
    float scalar_2 = vectV.dot (vectV);

    if (scalar_1 <= 0.0)
    {
        // Projected point on the line is on the left of segmentVertices[0].
        return (distPointPoint (point, segment_vertices[0]));
    }
    else if (scalar_1 >= scalar_2)
    {
        // Projected point on the line is on the right of segmentVertices[1].
        return (distPointPoint (point, segment_vertices[1]));
    }
    else
    {
        // Projected point on the line is on the line segment.
        Eigen::Vector3f point_projection = segment_vertices[0] + scalar_1/scalar_2*vectV;
        return (distPointPoint (point, point_projection));
    }
  }

  // Calculate the distance from a point to a plane.
  float
  distPointPlane (const Eigen::Vector3f & point, const std::vector<Eigen::Vector3f> & triangle_vertices)
  {
    Eigen::Vector3f vect_ab = triangle_vertices[1] - triangle_vertices[0];
    Eigen::Vector3f vect_ac = triangle_vertices[2] - triangle_vertices[0];
    Eigen::Vector3f normal = vect_ab.cross (vect_ac);
    normal /= std::sqrt (normal.dot (normal));
    Eigen::Vector3f vect_ap = point - triangle_vertices[0];
    Eigen::Vector3f vect_ap_parallel = vect_ap.dot (normal) / normal.dot (normal) * normal;

    return (vect_ap_parallel.norm ());
  }


} // namespace radi
