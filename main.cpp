#include <iostream>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cvs.h"
#include "cvs_estimation.h"
#include "cvs_corresp_group.h"
#include "ccn.h"
#include "ccn_estimation.h"
#include "ccn_corresp_group.h"
#include "board_detector.h"
#include "icf.h"

int main ()
{
  //
  // Specify CVSFeature(s) in the reference model.
  Eigen::Matrix4f mat_camera = Eigen::Matrix4Xf::Identity(4,4);
  // mat_camera(0,0) = -0.7071;
  // mat_camera(0,1) = -0.7071;
  // mat_camera(0,2) = 0.0;
  // mat_camera(1,0) = -0.5;
  // mat_camera(1,1) = 0.5;
  // mat_camera(1,2) = 0.7071;
  // mat_camera(2,0) = -0.5;
  // mat_camera(2,1) = 0.5;
  // mat_camera(2,2) = -0.7071;
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

  Eigen::Matrix4f inv_mat_camera = mat_camera.inverse ();

  std::vector<radi::CVSFeature> cvsFeatures(8);
  Eigen::Vector3f xAxis(1.0, 0.0, 0.0);
  Eigen::Vector3f yAxis(0.0, 1.0, 0.0);
  Eigen::Vector3f zAxis(0.0, 0.0, 1.0);
  xAxis = inv_mat_camera.block(0,0,3,3) * xAxis;
  yAxis = inv_mat_camera.block(0,0,3,3) * yAxis;
  zAxis = inv_mat_camera.block(0,0,3,3) * zAxis;
  Eigen::Vector3f pos_corner_0 (0.5, 1.0, 0.5);
  Eigen::Vector3f pos_corner_0_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_0 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[0] = radi::CVSFeature(pcl::PointXYZ(pos_corner_0_camera[0],
          pos_corner_0_camera[1], pos_corner_0_camera[2]));
  // cvsFeatures[0] = radi::CVSFeature(pcl::PointXYZ(0.5, 1.0, 0.5));
  cvsFeatures[0].appendVector(-yAxis);
  cvsFeatures[0].appendVector(-zAxis);
  cvsFeatures[0].appendVector(-xAxis);
  cvsFeatures[0].compute();

  // std::cout << "Position of corner-0: " << pos_corner_0_camera[0] << " "
  //           << pos_corner_0_camera[1] << " " << pos_corner_0_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_1 (0.5, 1.0, -0.5);
  Eigen::Vector3f pos_corner_1_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_1 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[1] = radi::CVSFeature(pcl::PointXYZ(pos_corner_1_camera[0],
          pos_corner_1_camera[1], pos_corner_1_camera[2]));
  // cvsFeatures[1] = radi::CVSFeature(pcl::PointXYZ(0.5, 1.0, -0.5));
  cvsFeatures[1].appendVector(-yAxis);
  cvsFeatures[1].appendVector(-xAxis);
  cvsFeatures[1].appendVector(zAxis);
  cvsFeatures[1].compute();

  // std::cout << "Position of corner-1: " << pos_corner_1_camera[0] << " "
  //           << pos_corner_1_camera[1] << " " << pos_corner_1_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_2 (0.5, -1.0, 0.5);
  Eigen::Vector3f pos_corner_2_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_2 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[2] = radi::CVSFeature(pcl::PointXYZ(pos_corner_2_camera[0],
          pos_corner_2_camera[1], pos_corner_2_camera[2]));
  // cvsFeatures[2] = radi::CVSFeature(pcl::PointXYZ(0.5, -1.0, 0.5));
  cvsFeatures[2].appendVector(-yAxis);
  cvsFeatures[2].appendVector(xAxis);
  cvsFeatures[2].appendVector(-zAxis);
  cvsFeatures[2].compute();

  // std::cout << "Position of corner-2: " << pos_corner_2_camera[0] << " "
  //           << pos_corner_2_camera[1] << " " << pos_corner_2_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_3 (0.5, -1.0, -0.5);
  Eigen::Vector3f pos_corner_3_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_3 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[3] = radi::CVSFeature(pcl::PointXYZ(pos_corner_3_camera[0],
          pos_corner_3_camera[1], pos_corner_3_camera[2]));
  // cvsFeatures[3] = radi::CVSFeature(pcl::PointXYZ(0.5, -1.0, -0.5));
  cvsFeatures[3].appendVector(-yAxis);
  cvsFeatures[3].appendVector(xAxis);
  cvsFeatures[3].appendVector(zAxis);
  cvsFeatures[3].compute();

  // std::cout << "Position of corner-3: " << pos_corner_3_camera[0] << " "
  //           << pos_corner_3_camera[1] << " " << pos_corner_3_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_4 (-0.5, 1.0, 0.5);
  Eigen::Vector3f pos_corner_4_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_4 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[4] = radi::CVSFeature(pcl::PointXYZ(pos_corner_4_camera[0],
          pos_corner_4_camera[1], pos_corner_4_camera[2]));
  // cvsFeatures[4] = radi::CVSFeature(pcl::PointXYZ(-0.5, 1.0, 0.5));
  cvsFeatures[4].appendVector(yAxis);
  cvsFeatures[4].appendVector(-xAxis);
  cvsFeatures[4].appendVector(-zAxis);
  cvsFeatures[4].compute();

  // std::cout << "Position of corner-4: " << pos_corner_4_camera[0] << " "
  //           << pos_corner_4_camera[1] << " " << pos_corner_4_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_5 (-0.5, 1.0, -0.5);
  Eigen::Vector3f pos_corner_5_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_5 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[5] = radi::CVSFeature(pcl::PointXYZ(pos_corner_5_camera[0],
          pos_corner_5_camera[1], pos_corner_5_camera[2]));
  // cvsFeatures[5] = radi::CVSFeature(pcl::PointXYZ(-0.5, 1.0, -0.5));
  cvsFeatures[5].appendVector(yAxis);
  cvsFeatures[5].appendVector(-xAxis);
  cvsFeatures[5].appendVector(zAxis);
  cvsFeatures[5].compute();

  // std::cout << "Position of corner-5: " << pos_corner_5_camera[0] << " "
  //           << pos_corner_5_camera[1] << " " << pos_corner_5_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_6 (-0.5, -1.0, 0.5);
  Eigen::Vector3f pos_corner_6_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_6 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[6] = radi::CVSFeature(pcl::PointXYZ(pos_corner_6_camera[0],
          pos_corner_6_camera[1], pos_corner_6_camera[2]));
  // cvsFeatures[6] = radi::CVSFeature(pcl::PointXYZ(-0.5, -1.0, 0.5));
  cvsFeatures[6].appendVector(xAxis);
  cvsFeatures[6].appendVector(yAxis);
  cvsFeatures[6].appendVector(-zAxis);
  cvsFeatures[6].compute();

  // std::cout << "Position of corner-6: " << pos_corner_6_camera[0] << " "
  //           << pos_corner_6_camera[1] << " " << pos_corner_6_camera[2] << std::endl;

  Eigen::Vector3f pos_corner_7 (-0.5, -1.0, -0.5);
  Eigen::Vector3f pos_corner_7_camera = inv_mat_camera.block(0,0,3,3)*pos_corner_7 + inv_mat_camera.block(0,3,3,1);
  cvsFeatures[7] = radi::CVSFeature(pcl::PointXYZ(pos_corner_7_camera[0],
          pos_corner_7_camera[1], pos_corner_7_camera[2]));
  // cvsFeatures[7] = radi::CVSFeature(pcl::PointXYZ(-0.5, -1.0, -0.5));
  cvsFeatures[7].appendVector(xAxis);
  cvsFeatures[7].appendVector(yAxis);
  cvsFeatures[7].appendVector(zAxis);
  cvsFeatures[7].compute();

  // Specify CCN Features in the model.
  Eigen::Matrix4f mat_camera_ccn = Eigen::Matrix4Xf::Identity(4,4);
  mat_camera_ccn(0,0) = 0.0;
  mat_camera_ccn(0,1) = 1.0;
  mat_camera_ccn(0,2) = 0.0;
  mat_camera_ccn(1,0) = -0.3752;
  mat_camera_ccn(1,1) = 0.0;
  mat_camera_ccn(1,2) = 0.927;
  mat_camera_ccn(2,0) = 0.927;
  mat_camera_ccn(2,1) = 0.0;
  mat_camera_ccn(2,2) = 0.3752;

  mat_camera_ccn(0,3) = 2.5;
  mat_camera_ccn(1,3) = 0.0;
  mat_camera_ccn(2,3) = 1.0;

  std::vector<radi::CCNFeature> ccn_features (1);
  radi::CCNFeature ccn_feature;
  Eigen::Vector3f center (0.0, 0.0, 0.15);
  Eigen::Vector3f normal (0.0, 0.0, 1.0);
  ccn_feature.setCenter(center);
  ccn_feature.setNormal(normal);
  ccn_feature.setRadius(0.17);
  transformCCNFeature(Eigen::Matrix4Xf::Identity(4,4), ccn_feature, ccn_features[0]);

  // std::cout << "Position of corner-7: " << pos_corner_7_camera[0] << " "
  //           << pos_corner_7_camera[1] << " " << pos_corner_7_camera[2] << std::endl;

  // const std::vector<float> & angles = cvsFeatures[7].getIncludedAngles();
  // std::cout << angles[0] << "  " << angles[1] << "  " << angles[2] << std::endl;

  //
  // Load pcd file.
  // pcl::PointCloud<pcl::PointXYZ>::Ptr sceneRaw(new pcl::PointCloud<pcl::PointXYZ>());
  // if (pcl::io::loadPCDFile<pcl::PointXYZ>("./Models/cuboid00000.pcd", *sceneRaw) == -1)
  // {
  //   PCL_ERROR("Couldn't read file cuboid00000.pcd.\n");
  //   return -1;
  // }

  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneRaw(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("./Models/cup00000.pcd", *sceneRaw) == -1)
  {
    PCL_ERROR("Couldn't read file cup00000.pcd.\n");
    return -1;
  }

  //
  // Filtering & Downsampling.
  // Remove nan data.
  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneNaNClean(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*sceneRaw, *sceneNaNClean, indices);
  // pcl::io::savePCDFileASCII<pcl::PointXYZ>("./Models/cuboid_nan_clean.pcd", *sceneNaNClean);
  pcl::io::savePCDFileASCII<pcl::PointXYZ>("./Models/cup_nan_clean.pcd", *sceneNaNClean);

  // Remove outliers.
  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneFiltered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlierFilter;
  outlierFilter.setInputCloud(sceneNaNClean);
  outlierFilter.setMeanK(50);
  outlierFilter.setStddevMulThresh(1.0);
  outlierFilter.filter(*sceneFiltered);

  // Downsampling. Downsample the origin point cloud and reduce the number of points.
  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneDownSampled(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> voxGrid;
  voxGrid.setInputCloud(sceneFiltered);
  voxGrid.setLeafSize(0.01, 0.01, 0.01);
  voxGrid.filter(*sceneDownSampled);

  // Visualize the downsampled point cloud.
  // pcl::visualization::PCLVisualizer viewer("Downsample");
  // viewer.addPointCloud(sceneDownSampled, "Scene Downsmapled");
  // while (!viewer.wasStopped()) {
  //     viewer.spinOnce();
  // }

  std::cout << "Number of points in the pcd file: " << sceneRaw->size() << std::endl;
  std::cout << "Number of points after filtering and downsampling: " << sceneDownSampled->size() << std::endl;

  radi::CCNEstimation ccn_estimator;
  std::vector<radi::CCNFeature> ccn_feature_list;
  ccn_estimator.setInputCloud(sceneDownSampled);
  ccn_estimator.esimate(ccn_feature_list);

  std::cout << "Number of ccn features in the scene cloud: " << ccn_feature_list.size() << std::endl;

  radi::CCNCorrespGroup ccn_corresp;
  ccn_corresp.setInputCloud(sceneDownSampled);
  ccn_corresp.setModelFeatures(&ccn_features);
  ccn_corresp.setSceneFeatures(&ccn_feature_list);
  std::vector<Eigen::Matrix4f> transf_list;
  ccn_corresp.recognize(transf_list);

  // // Detect board points.
  // radi::BoardDetector board_detector;
  // board_detector.setInputCloud(sceneDownSampled);
  // std::vector<int> board_indices;
  // board_detector.compute(board_indices);

  // std::cout << "Number of board points: " << board_indices.size () << std::endl;

  // // Visualize board points.
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::ExtractIndices<pcl::PointXYZ> extracter;
  // extracter.setInputCloud(sceneDownSampled);
  // extracter.setIndices(boost::make_shared<std::vector<int> > (board_indices));
  // extracter.setNegative(false);
  // extracter.filter(*cloudPlane);
  // pcl::visualization::PCLVisualizer viewer("Board Points");
  // viewer.addPointCloud(cloudPlane, "Board points");
  // while (!viewer.wasStopped()) {
  //     viewer.spinOnce();
  // }

  // std::vector<radi::CVSFeature> cvs_feature_list;
  // radi::CVSEstimation cvs_estimation;
  // cvs_estimation.setInputCloud(sceneDownSampled);
  // cvs_estimation.esimate(cvs_feature_list);

  // std::cout << "Number of cvs features: " << cvs_feature_list.size() << std::endl;

  // // Corresponce group.
  // radi::CVSCorrespGroup corresp_group;
  // corresp_group.setSceneFeatures(&cvs_feature_list);
  // corresp_group.setModelFeatures(&cvsFeatures);
  // std::vector<Eigen::Matrix4f> mat_transf_list;
  // corresp_group.recognize(mat_transf_list);

  // std::cout << mat_transf_list.size() << std::endl;

  // // Perform ICF algorithm. Refine the tramsformations.
  // radi::IterativeClosestFace icf;
  // icf.setScenePointCloud(sceneDownSampled);
  // icf.setReferenceModel("Models/cuboid.stl");
  // std::cout << "Number of triangles: " << icf.getReferenceModel().getNumTriangles() << std::endl;
  // for (std::size_t i = 0; i < mat_transf_list.size(); ++i)
  // {
  //   icf.setInitialTransformation(mat_transf_list[i]);
  //   Eigen::Matrix4f mat_transf;
  //   icf.estimate(mat_transf);
  // }


  // //
  // // Segmentation.
  // //
  // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  // pcl::PointIndices::Ptr inlierIndices(new pcl::PointIndices());
  // pcl::SACSegmentation<pcl::PointXYZI> seg;
  // seg.setOptimizeCoefficients(true);
  // seg.setModelType(pcl::SACMODEL_PLANE);
  // seg.setMethodType(pcl::SAC_RANSAC);
  // seg.setDistanceThreshold(0.01);
  // seg.setInputCloud(cloudFiltered);
  // seg.segment(*inlierIndices, *coefficients);

  // if (inlierIndices->indices.size() == 0) {
  //     PCL_ERROR("Could not estimate a planar model for the given dataset.");
  //     return -1;
  // }

  // // Extract the inliers.
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZI>);
  // pcl::ExtractIndices<pcl::PointXYZI> extracter;
  // extracter.setInputCloud(cloudFiltered);
  // extracter.setIndices(inlierIndices);
  // extracter.setNegative(false);
  // extracter.filter(*cloudPlane);


  std::cout << "Hello World!" << std::endl;
  return 0;
}

