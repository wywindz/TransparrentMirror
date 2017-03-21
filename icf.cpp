#include <cmath>
#include <random>
#include "icf.h"

namespace radi
{
  IterativeClosestFace::IterativeClosestFace ()
  {
    minBoundaryTranslation = Eigen::Vector3f(-100, -100, -100);
    maxBoundaryTranslation = Eigen::Vector3f(100, 100, 100);
    minBoundaryRotation = Eigen::Vector3f(-3.1415926, -3.1415926, -3.1415926);
    maxBoundaryRotation = Eigen::Vector3f(3.1415926, 3.1415926, 3.1415926);
    deviationTranslation = Eigen::Vector3f(50, 50, 50);
    deviationRotation = Eigen::Vector3f(50, 50, 50);
    iterationOuter = 50;
    iterationMedium = 100;
    iterationInner = 20;
    hasConverged = false;
    transfMatrix = Eigen::MatrixXf::Identity(4,4);
  }

  IterativeClosestFace::IterativeClosestFace (const std::string & refModelPath,
          pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
  {
    modelMesh.loadModel(refModelPath);
    scenePointCloud = scene;
  }

  IterativeClosestFace::~IterativeClosestFace ()
  { }

  // Import reference model.
  void
  IterativeClosestFace::setReferenceModel (const std::string & refModelPath)
  {
    modelMesh.loadModel(refModelPath);
  }

  // Set point cloud of the scene.
  void
  IterativeClosestFace::setScenePointCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
  {
    scenePointCloud = scene;
  }

  void
  IterativeClosestFace::setMaxIterations (std::size_t numIterationOuter,
          std::size_t numIterationMedium, std::size_t numIterationInner)
  {
    iterationOuter = numIterationOuter;
    iterationMedium = numIterationMedium;
    iterationInner = numIterationInner;
  }

  const Mesh &
  IterativeClosestFace::getReferenceModel () const
  {
    return modelMesh;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr
  IterativeClosestFace::getScenePointCloud () const
  {
    return scenePointCloud;
  }

  Eigen::Vector3i
  IterativeClosestFace::getMaxIterations ()
  {
    Eigen::Vector3i iterations;
    iterations[0] = iterationOuter;
    iterations[1] = iterationMedium;
    iterations[2] = iterationInner;

    return iterations;
  }

  // Pose estimation.
  void
  IterativeClosestFace::estimate (Eigen::Matrix4f & matrix)
  {
    Eigen::Vector3f meanTranslation = uniformRandom(minBoundaryTranslation, maxBoundaryTranslation);
    Pose bestTranslation = generatePose(meanTranslation);
    bestTranslation.objectiveValue(this->modelMesh, this->scenePointCloud);
    std::vector<Pose> bestTranslationList;
    std::size_t indexOuter = 0;
    bool hasReachedMaximaOnTranslation = false;
    std::size_t countRepeationOnTranslation = 0;
    while (indexOuter < iterationOuter)
    {
      std::cout << "Outer iteration: " << indexOuter << std::endl;
      // Decrease or increase the standard deviation of sampling the translation.
      // ToDo: Add boundaries for the deviations.
      if (hasReachedMaximaOnTranslation)
      {
        // Increase the standard deviation in order to jump out of the local maxima.
        deviationTranslation[0] += 20.0;
        deviationTranslation[1] += 20.0;
        deviationTranslation[2] += 20.0;
        // Resample the mean translation again.
        meanTranslation = uniformRandom(minBoundaryTranslation, maxBoundaryTranslation);
        // Set the flag to false.
        hasReachedMaximaOnTranslation = false;
      }
      else
      {
        // Descrease the standard deviation.
        deviationTranslation[0] -= 10.0;
        deviationTranslation[1] -= 10.0;
        deviationTranslation[2] -= 10.0;
      }
      Eigen::Vector3f sampledTranslation = gaussianRandom(meanTranslation, deviationTranslation);
      Pose sampledPose = generatePose(sampledTranslation);
      if (sampledPose.objectiveValue(modelMesh, scenePointCloud) < bestTranslation.objectiveValue())
      {
        bestTranslation = sampledPose;
      }

      // Sample the rotation under the current sampled translation.
      Eigen::Vector3f meanZyzEuler = uniformRandom(minBoundaryRotation, maxBoundaryRotation);
      Pose bestRotation = generatePose(bestTranslation.translation(), meanZyzEuler);
      bestRotation.objectiveValue(this->modelMesh, this->scenePointCloud);
      std::vector<Pose> bestRotationList;
      std::size_t indexMedium = 0;
      bool hasReachedMaximaOnRotation = false;
      std::size_t countRepeationOnRotation = 0;
      while (indexMedium < iterationMedium)
      {
        std::cout << "Medium iteration: " << indexMedium << std::endl;
        // Decrease or increase the standard deviation of sampling the rotation.
        if (hasReachedMaximaOnRotation) {
            // Increase the standard deviation in order to jump out of the local maxima.
            deviationRotation[0] += 20.0 * 3.1415926/180.0;
            deviationRotation[1] += 20.0 * 3.1415926/180.0;
            deviationRotation[2] += 20.0 * 3.1415926/180.0;
            // Resample the mean rotation again.
            meanZyzEuler = uniformRandom(minBoundaryRotation, maxBoundaryRotation);
            // Set the flag to false.
            hasReachedMaximaOnRotation = false;
        } else {
            // Descrease the standard deviation.
            deviationRotation[0] -= 10.0 * 3.1415926/180.0;
            deviationRotation[1] -= 10.0 * 3.1415926/180.0;
            deviationRotation[2] -= 10.0 * 3.1415926/180.0;
        }
        // Sample 'iterationInner' poses under the current sampled translation and rotation.
        std::vector<Pose> poseInnerList(iterationInner);
        std::size_t indexInner = 0;
        while (indexInner < iterationInner) {
            Eigen::Vector3f sampledRotation = gaussianRandom(meanZyzEuler, deviationRotation);
            poseInnerList[indexInner] = generatePose(sampledTranslation, sampledRotation);
            indexInner++;
        }

        Pose bestPoseInner = findBestPose(poseInnerList, modelMesh, scenePointCloud);
        if (bestRotation.objectiveValue() <= bestPoseInner.objectiveValue()) {
            countRepeationOnRotation++;
            if (countRepeationOnRotation >= 3) {
                hasReachedMaximaOnRotation = true;
                bestRotationList.push_back(bestRotation);
                countRepeationOnRotation = 0;
            }
        } else {
            bestRotation = bestPoseInner;
            meanZyzEuler = bestRotation.zyzEuler();
            countRepeationOnRotation = 0;
        }

        std::cout << "Objective value: " << bestRotation.objectiveValue() << std::endl;
        std::cout << bestRotation.translation()[0] << "  "
                << bestRotation.translation()[1] << "  " << bestRotation.translation()[2] << std::endl;
        std::cout << bestRotation.zyzEuler()[0] << "  "
                << bestRotation.zyzEuler()[1] << "  " << bestRotation.zyzEuler()[2] << std::endl;


        indexMedium++;
      }

      // Best pose at k-step iteration.
      Pose bestPoseMedium = findBestPose(bestRotationList);
      if (bestTranslation.objectiveValue() <= bestPoseMedium.objectiveValue()) {
          countRepeationOnTranslation++;
          if (countRepeationOnTranslation >= 3) {
              hasReachedMaximaOnTranslation = true;
              bestTranslationList.push_back(bestTranslation);
              countRepeationOnTranslation = 0;
          }
      } else {
          bestTranslation = bestPoseMedium;
          meanTranslation = bestTranslation.translation();
          countRepeationOnTranslation = 0;
      }

      indexOuter++;
    }

    // Find the best pose from the local maximas.
    Pose bestPose = findBestPose(bestTranslationList);
    std::cout << bestPose.translation()[0] << "  "
            << bestPose.translation()[1] << "  " << bestPose.translation()[2] << std::endl;
    std::cout << bestPose.zyzEuler()[0] << "  "
            << bestPose.zyzEuler()[1] << "  " << bestPose.zyzEuler()[2] << std::endl;
  }

  // Calculate the objective value for an estimated pose.
  float IterativeClosestFace::objectiveValue()
  {
      return relativePose.objectiveValue();
  }

  Eigen::Vector3f uniformRandom(const Eigen::Vector3f & minBoundary, const Eigen::Vector3f & maxBoundary)
  {
      Eigen::Vector3f randomValue;
      std::random_device randDevice;
      std::mt19937 generator(randDevice());
      std::uniform_real_distribution<float> distrX(minBoundary[0], maxBoundary[0]);
      std::uniform_real_distribution<float> distrY(minBoundary[1], maxBoundary[1]);
      std::uniform_real_distribution<float> distrZ(minBoundary[2], maxBoundary[2]);
      randomValue[0] = distrX(generator);
      randomValue[1] = distrY(generator);
      randomValue[2] = distrZ(generator);

      return randomValue;
  }

  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, const Eigen::Vector3f & deviation)
  {
      Eigen::Vector3f randomValue;
      std::random_device randDevice;
      std::mt19937 generator(randDevice());
      std::normal_distribution<float> distrX(mean[0], deviation[0]);
      std::normal_distribution<float> distrY(mean[1], deviation[1]);
      std::normal_distribution<float> distrZ(mean[2], deviation[2]);
      randomValue[0] = distrX(generator);
      randomValue[1] = distrY(generator);
      randomValue[2] = distrZ(generator);

      return randomValue;
  }

  Eigen::Vector3f gaussianRandom(const Eigen::Vector3f & mean, float deviation)
  {
      Eigen::Vector3f stdDeviation;
      stdDeviation[0] = deviation;
      stdDeviation[1] = deviation;
      stdDeviation[2] = deviation;
      return gaussianRandom(mean, stdDeviation);
  }

} // namespace radi
