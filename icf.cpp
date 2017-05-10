#include <cmath>
#include <random>
#include <exception>

#include <pcl/common/transforms.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "icf.h"

namespace radi
{
  IterativeClosestFace::IterativeClosestFace ()
  {
    min_range_translation_ = Eigen::Vector3f(-100, -100, -100);
    max_range_translation_ = Eigen::Vector3f(100, 100, 100);
    min_range_rotation_ = Eigen::Vector3f(-3.1415926, -3.1415926, -3.1415926);
    max_range_rotation_ = Eigen::Vector3f(3.1415926, 3.1415926, 3.1415926);
    iteration_outer_ = 50;
    iteration_inner_ = 20;
    has_converged = false;
    threshold_distance_near_ = 0.03;
    threshold_distance_extreme_ = 0.5;
    threshold_valid_ = 0.4;
    init_transf_ = Eigen::MatrixXf::Identity(4,4);
  }

  IterativeClosestFace::IterativeClosestFace (const std::string & model_file_path,
          pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud)
  {
    model_mesh_.loadModel(model_file_path);
    scene_point_cloud_ = scene_point_cloud;
  }

  IterativeClosestFace::~IterativeClosestFace ()
  { }

  // Import reference model.
  void
  IterativeClosestFace::setReferenceModel (const std::string & model_file_path)
  {
    model_mesh_.loadModel(model_file_path);
  }

  // Set point cloud of the scene.
  void
  IterativeClosestFace::setScenePointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud)
  {
    scene_point_cloud_ = scene_point_cloud;
  }

  void
  IterativeClosestFace::setMinRangeTranslation (const Eigen::Vector3f & min_range)
  {
    min_range_translation_ = min_range;
  }

  void
  IterativeClosestFace::setMaxRangeTranslation (const Eigen::Vector3f & max_range)
  {
    max_range_translation_ = max_range;
  }

  void
  IterativeClosestFace::setMinRangeRotation (const Eigen::Vector3f & min_range)
  {
    min_range_rotation_ = min_range;
  }

  void
  IterativeClosestFace::setMaxRangeRotation (const Eigen::Vector3f & max_range)
  {
    max_range_rotation_ = max_range;
  }

  void
  IterativeClosestFace::setIterationOuter (std::size_t iteration_outer)
  {
    iteration_outer_ = iteration_outer;
  }

  void
  IterativeClosestFace::setIterationInner (std::size_t iteration_inner)
  {
    iteration_inner_ = iteration_inner;
  }

  void
  IterativeClosestFace::setInitialTransformation (const Eigen::Matrix4f & init_transf)
  {
    init_transf_ = init_transf;
  }

  const Mesh &
  IterativeClosestFace::getReferenceModel () const
  {
    return (model_mesh_);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  IterativeClosestFace::getScenePointCloud () const
  {
    return scene_point_cloud_;
  }

  const Eigen::Vector3f
  IterativeClosestFace::getMinRangeTranslation ()
  {
    return (min_range_translation_);
  }

  const Eigen::Vector3f
  IterativeClosestFace::getMaxRangeTranslation ()
  {
    return (max_range_translation_);
  }

  const Eigen::Vector3f
  IterativeClosestFace::getMinRangeRotation ()
  {
    return (min_range_rotation_);
  }

  const Eigen::Vector3f
  IterativeClosestFace::getMaxRangeRotation ()
  {
    return (max_range_rotation_);
  }

  std::size_t
  IterativeClosestFace::getIterationOuter ()
  {
    return (iteration_outer_);
  }

  std::size_t
  IterativeClosestFace::getIterationInner ()
  {
    return (iteration_inner_);
  }

  const Eigen::Matrix4f
  IterativeClosestFace::getInitialTransformation ()
  {
    return (init_transf_);
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

      Eigen::Vector3f translation_sampled = uniformRandom (translation-min_range_translation_,
              translation+max_range_translation_);

      std::vector<float> objective_value_list (iteration_inner_);
      std::vector<Eigen::Matrix4f> mat_transf_list (iteration_inner_);
      for (std::size_t idx_inner = 0; idx_inner < iteration_inner_; ++idx_inner)
      {
        Eigen::Vector3f rotation_sampled = uniformRandom (rotation-min_range_rotation_,
                rotation-max_range_rotation_);
        mat_transf_list[idx_inner] = Eigen::Matrix4Xf::Identity(4,4);
        mat_transf_list[idx_inner].block(0,0,3,3) = euler2matrix(rotation_sampled);
        mat_transf_list[idx_inner].block(0,3,3,1) = translation_sampled;

        objective_value_list[idx_inner] = calObjectiveValue (mat_transf_list[idx_inner]);
      }

      // Sort 'objective_value' and select the best sample.
      std::vector<std::size_t> order_indices(iteration_inner_);
      std::iota(order_indices.begin (), order_indices.end (), 0);
      std::sort(order_indices.begin(), order_indices.end(),
              [&objective_value_list](int idx_1, int idx_2)
              { return objective_value_list[idx_1] > objective_value_list[idx_2]; });

      if (objective_value_list[order_indices[0]] > objective_value)
      {
        objective_value = objective_value_list[order_indices[0]];
        mat_transf = mat_transf_list[order_indices[0]];
      }
    }

    estimated_transf = mat_transf;
  }

  // // Pose estimation.
  // void
  // IterativeClosestFace::estimate (Eigen::Matrix4f & matrix)
  // {
  //   Eigen::Vector3f meanTranslation = uniformRandom(minBoundaryTranslation, maxBoundaryTranslation);
  //   Pose bestTranslation = generatePose(meanTranslation);
  //   bestTranslation.objectiveValue(this->modelMesh, this->scenePointCloud);
  //   std::vector<Pose> bestTranslationList;
  //   std::size_t indexOuter = 0;
  //   bool hasReachedMaximaOnTranslation = false;
  //   std::size_t countRepeationOnTranslation = 0;
  //   while (indexOuter < iterationOuter)
  //   {
  //     std::cout << "Outer iteration: " << indexOuter << std::endl;
  //     // Decrease or increase the standard deviation of sampling the translation.
  //     // ToDo: Add boundaries for the deviations.
  //     if (hasReachedMaximaOnTranslation)
  //     {
  //       // Increase the standard deviation in order to jump out of the local maxima.
  //       deviationTranslation[0] += 20.0;
  //       deviationTranslation[1] += 20.0;
  //       deviationTranslation[2] += 20.0;
  //       // Resample the mean translation again.
  //       meanTranslation = uniformRandom(minBoundaryTranslation, maxBoundaryTranslation);
  //       // Set the flag to false.
  //       hasReachedMaximaOnTranslation = false;
  //     }
  //     else
  //     {
  //       // Descrease the standard deviation.
  //       deviationTranslation[0] -= 10.0;
  //       deviationTranslation[1] -= 10.0;
  //       deviationTranslation[2] -= 10.0;
  //     }
  //     Eigen::Vector3f sampledTranslation = gaussianRandom(meanTranslation, deviationTranslation);
  //     Pose sampledPose = generatePose(sampledTranslation);
  //     if (sampledPose.objectiveValue(modelMesh, scenePointCloud) < bestTranslation.objectiveValue())
  //     {
  //       bestTranslation = sampledPose;
  //     }

  //     // Sample the rotation under the current sampled translation.
  //     Eigen::Vector3f meanZyzEuler = uniformRandom(minBoundaryRotation, maxBoundaryRotation);
  //     Pose bestRotation = generatePose(bestTranslation.translation(), meanZyzEuler);
  //     bestRotation.objectiveValue(this->modelMesh, this->scenePointCloud);
  //     std::vector<Pose> bestRotationList;
  //     std::size_t indexMedium = 0;
  //     bool hasReachedMaximaOnRotation = false;
  //     std::size_t countRepeationOnRotation = 0;
  //     while (indexMedium < iterationMedium)
  //     {
  //       std::cout << "Medium iteration: " << indexMedium << std::endl;
  //       // Decrease or increase the standard deviation of sampling the rotation.
  //       if (hasReachedMaximaOnRotation) {
  //           // Increase the standard deviation in order to jump out of the local maxima.
  //           deviationRotation[0] += 20.0 * 3.1415926/180.0;
  //           deviationRotation[1] += 20.0 * 3.1415926/180.0;
  //           deviationRotation[2] += 20.0 * 3.1415926/180.0;
  //           // Resample the mean rotation again.
  //           meanZyzEuler = uniformRandom(minBoundaryRotation, maxBoundaryRotation);
  //           // Set the flag to false.
  //           hasReachedMaximaOnRotation = false;
  //       } else {
  //           // Descrease the standard deviation.
  //           deviationRotation[0] -= 10.0 * 3.1415926/180.0;
  //           deviationRotation[1] -= 10.0 * 3.1415926/180.0;
  //           deviationRotation[2] -= 10.0 * 3.1415926/180.0;
  //       }
  //       // Sample 'iterationInner' poses under the current sampled translation and rotation.
  //       std::vector<Pose> poseInnerList(iterationInner);
  //       std::size_t indexInner = 0;
  //       while (indexInner < iterationInner) {
  //           Eigen::Vector3f sampledRotation = gaussianRandom(meanZyzEuler, deviationRotation);
  //           poseInnerList[indexInner] = generatePose(sampledTranslation, sampledRotation);
  //           indexInner++;
  //       }

  //       Pose bestPoseInner = findBestPose(poseInnerList, modelMesh, scenePointCloud);
  //       if (bestRotation.objectiveValue() <= bestPoseInner.objectiveValue()) {
  //           countRepeationOnRotation++;
  //           if (countRepeationOnRotation >= 3) {
  //               hasReachedMaximaOnRotation = true;
  //               bestRotationList.push_back(bestRotation);
  //               countRepeationOnRotation = 0;
  //           }
  //       } else {
  //           bestRotation = bestPoseInner;
  //           meanZyzEuler = bestRotation.zyzEuler();
  //           countRepeationOnRotation = 0;
  //       }

  //       std::cout << "Objective value: " << bestRotation.objectiveValue() << std::endl;
  //       std::cout << bestRotation.translation()[0] << "  "
  //               << bestRotation.translation()[1] << "  " << bestRotation.translation()[2] << std::endl;
  //       std::cout << bestRotation.zyzEuler()[0] << "  "
  //               << bestRotation.zyzEuler()[1] << "  " << bestRotation.zyzEuler()[2] << std::endl;


  //       indexMedium++;
  //     }

  //     // Best pose at k-step iteration.
  //     Pose bestPoseMedium = findBestPose(bestRotationList);
  //     if (bestTranslation.objectiveValue() <= bestPoseMedium.objectiveValue()) {
  //         countRepeationOnTranslation++;
  //         if (countRepeationOnTranslation >= 3) {
  //             hasReachedMaximaOnTranslation = true;
  //             bestTranslationList.push_back(bestTranslation);
  //             countRepeationOnTranslation = 0;
  //         }
  //     } else {
  //         bestTranslation = bestPoseMedium;
  //         meanTranslation = bestTranslation.translation();
  //         countRepeationOnTranslation = 0;
  //     }

  //     indexOuter++;
  //   }

  //   // Find the best pose from the local maximas.
  //   Pose bestPose = findBestPose(bestTranslationList);
  //   std::cout << bestPose.translation()[0] << "  "
  //           << bestPose.translation()[1] << "  " << bestPose.translation()[2] << std::endl;
  //   std::cout << bestPose.zyzEuler()[0] << "  "
  //           << bestPose.zyzEuler()[1] << "  " << bestPose.zyzEuler()[2] << std::endl;
  // }

  // Calculate the objective value for an estimated pose.
  float IterativeClosestFace::objectiveValue()
  {
      return relativePose.objectiveValue();
  }

  float
  IterativeClosestFace::calObjectiveValue (const Eigen::Matrix4f & mat_transf)
  {
    std::cout << "Matrix transf: \n" << mat_transf << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_scene (new pcl::PointCloud<pcl::PointXYZ> ());
    // pcl::transformPointCloud(*scene_point_cloud_, transformed_scene, mat_transf);
    Eigen::Matrix4f mat_camera = Eigen::Matrix4Xf::Identity(4,4);
    mat_camera(0,0) = 0.7071;
    mat_camera(0,1) = -0.5;
    mat_camera(0,2) = 0.5;
    mat_camera(1,0) = 0.7071;
    mat_camera(1,1) = 0.5;
    mat_camera(1,2) = -0.5;
    mat_camera(2,0) = 0.0;
    mat_camera(2,1) = 0.7071;
    mat_camera(2,2) = 0.7071;

    mat_camera(0,3) = 2.0;
    mat_camera(1,3) = -2.0;
    mat_camera(2,3) = 2.0;
    Eigen::Matrix4f mat_transf_total = mat_camera * mat_transf.inverse();
    // Eigen::Matrix4f inv_mat_transf = mat_transf.inverse ();
    pcl::transformPointCloud(*scene_point_cloud_, *transformed_scene, mat_transf_total);
    // pcl::transformPointCloud(*scene_point_cloud_, transformed_scene, inv_mat_transf);

    // Show 3d model and transformed point cloud.
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL("Models/cuboid.stl", mesh);
    pcl::visualization::PCLVisualizer viewer ("Model & Point Cloud");
    viewer.addPolygonMesh(mesh);
    viewer.addPointCloud<pcl::PointXYZ> (transformed_scene, "Transformed point cloud");
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce();
    }

    float objective_value = 0.0;
    for (std::size_t i = 0; i < (*transformed_scene).points.size(); ++i)
    {
      Eigen::Vector3f point;
      point[0] = static_cast<float>((*transformed_scene).points[i].x);
      point[1] = static_cast<float>((*transformed_scene).points[i].y);
      point[2] = static_cast<float>((*transformed_scene).points[i].z);
      std::vector<float> distance_list(model_mesh_.getNumTriangles());
      for (std::size_t j = 0; j < model_mesh_.getNumTriangles(); ++j)
      {
        distance_list[j] = distPointTriangle(point, model_mesh_.getTriangle(j));
      }
      float shortest_distance = *std::min_element(distance_list.begin(), distance_list.end());

      if (shortest_distance > threshold_distance_extreme_)
      {
        // ToDo: Throw an exception.
        std::cout << "Shortest distance: " << shortest_distance << std::endl;
        throw "Too large distance.";
      }

      float gpx;
      if (shortest_distance < threshold_distance_near_)
      {
        gpx = shortest_distance;
      }
      else
      {
          gpx = 4.0 * shortest_distance;
      }

      float fxp;
      if (gpx < threshold_valid_)
      {
          fxp = 1.0 - gpx/threshold_valid_;
      }
      else
      {
          fxp = 0.0;
      }

      objective_value += fxp;
    }

    return (objective_value);
  }

  Eigen::Vector3f uniformRandom(const Eigen::Vector3f & min_boundary, const Eigen::Vector3f & max_boundary)
  {
      Eigen::Vector3f random_value;
      std::random_device rand_device;
      std::mt19937 generator (rand_device());
      std::uniform_real_distribution<float> distr_x(min_boundary[0], max_boundary[0]);
      std::uniform_real_distribution<float> distr_y(min_boundary[1], max_boundary[1]);
      std::uniform_real_distribution<float> distr_z(min_boundary[2], max_boundary[2]);
      random_value[0] = distr_x(generator);
      random_value[1] = distr_y(generator);
      random_value[2] = distr_z(generator);

      return random_value;
  }

  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, const Eigen::Vector3f & deviation)
  {
      Eigen::Vector3f random_value;
      std::random_device rand_device;
      std::mt19937 generator (rand_device());
      std::normal_distribution<float> distr_x(mean[0], deviation[0]);
      std::normal_distribution<float> distr_y(mean[1], deviation[1]);
      std::normal_distribution<float> distr_z(mean[2], deviation[2]);
      random_value[0] = distr_x(generator);
      random_value[1] = distr_y(generator);
      random_value[2] = distr_z(generator);

      return random_value;
  }

  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, float deviation)
  {
      Eigen::Vector3f std_deviation;
      std_deviation[0] = deviation;
      std_deviation[1] = deviation;
      std_deviation[2] = deviation;
      return gaussianRandom(mean, std_deviation);
  }

  const Eigen::Vector3f matrix2euler (const Eigen::Matrix3f & mat_rotation)
  {
    const float PI = 3.1415926;
    const float EPS = 1.0E-8;

    float alpha;
    float beta;
    float gamma;

    // Assuming beta is in [0,pi].
    double a_02 = mat_rotation(0,2);
    double a_01 = mat_rotation(0,1);
    double a_11 = mat_rotation(1,1);
    double a_12 = mat_rotation(1,2);
    double a_20 = mat_rotation(2,0);
    double a_21 = mat_rotation(2,1);
    double a_22 = mat_rotation(2,2);

    beta = std::atan2(std::sqrt(std::pow(a_02,2)+std::pow(a_12,2)), a_22);

    if ((EPS < beta) && (beta < (PI-EPS)))
    {
      alpha = std::atan2(a_12, a_02);
      gamma = std::atan2(a_21, -a_20);
    }
    else if (beta <= EPS)
    {
      alpha = 0.0;
      gamma = std::atan2(-a_01, a_11);
    }
    else
    {
      alpha = 0.0;
      gamma = std::atan2(a_01, a_11);
    }

    return (Eigen::Vector3f(alpha, beta, gamma));
  }

  const Eigen::Matrix3f euler2matrix (const Eigen::Vector3f & euler_angle)
  {
    double phi = euler_angle[0];
    double theta = euler_angle[1];
    double psi = euler_angle[2];

    Eigen::Matrix3f mat_rotation;
    mat_rotation(0,0) = std::cos(phi)*std::cos(theta)*std::cos(psi) -
            std::sin(phi)*std::sin(psi);
    mat_rotation(0,1) = -std::cos(phi)*std::cos(theta)*std::sin(psi) -
            std::sin(phi)*std::cos(psi);
    mat_rotation(0,2) = std::cos(phi)*std::sin(theta);

    mat_rotation(1,0) = std::sin(phi)*std::cos(theta)*std::cos(psi) +
            std::cos(phi)*std::sin(psi);
    mat_rotation(1,1) = -std::sin(phi)*std::cos(theta)*std::sin(psi) +
            std::cos(phi)*std::cos(psi);
    mat_rotation(1,2) = std::sin(phi)*std::sin(theta);

    mat_rotation(2,0) = -std::sin(theta)*std::cos(psi);
    mat_rotation(2,1) = std::sin(theta)*std::sin(psi);
    mat_rotation(2,2) = std::cos(theta);

    return (mat_rotation);
  }

} // namespace radi
