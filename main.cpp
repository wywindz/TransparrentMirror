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
#include "icf.h"

int main ()
{
  //
  // Specify CVSFeature(s) in the reference model.
  std::vector<radi::CVSFeature> cvsFeatures(8);
  Eigen::Vector3f xAxis(1.0, 0.0, 0.0);
  Eigen::Vector3f yAxis(0.0, 1.0, 0.0);
  Eigen::Vector3f zAxis(0.0, 0.0, 1.0);
  cvsFeatures[0] = radi::CVSFeature(pcl::PointXYZ(0.5, 1.0, 0.5));
  cvsFeatures[0].appendVector(-yAxis);
  cvsFeatures[0].appendVector(-zAxis);
  cvsFeatures[0].appendVector(-xAxis);
  cvsFeatures[0].compute();

  cvsFeatures[1] = radi::CVSFeature(pcl::PointXYZ(0.5, 1.0, -0.5));
  cvsFeatures[1].appendVector(-yAxis);
  cvsFeatures[1].appendVector(-xAxis);
  cvsFeatures[1].appendVector(zAxis);
  cvsFeatures[1].compute();

  cvsFeatures[2] = radi::CVSFeature(pcl::PointXYZ(0.5, -1.0, 0.5));
  cvsFeatures[2].appendVector(-yAxis);
  cvsFeatures[2].appendVector(xAxis);
  cvsFeatures[2].appendVector(-zAxis);
  cvsFeatures[2].compute();

  cvsFeatures[3] = radi::CVSFeature(pcl::PointXYZ(0.5, -1.0, -0.5));
  cvsFeatures[3].appendVector(-yAxis);
  cvsFeatures[3].appendVector(xAxis);
  cvsFeatures[3].appendVector(zAxis);
  cvsFeatures[3].compute();

  cvsFeatures[4] = radi::CVSFeature(pcl::PointXYZ(-0.5, 1.0, 0.5));
  cvsFeatures[4].appendVector(yAxis);
  cvsFeatures[4].appendVector(-xAxis);
  cvsFeatures[4].appendVector(-zAxis);
  cvsFeatures[4].compute();

  cvsFeatures[5] = radi::CVSFeature(pcl::PointXYZ(-0.5, 1.0, -0.5));
  cvsFeatures[5].appendVector(yAxis);
  cvsFeatures[5].appendVector(-xAxis);
  cvsFeatures[5].appendVector(zAxis);
  cvsFeatures[5].compute();

  cvsFeatures[6] = radi::CVSFeature(pcl::PointXYZ(-0.5, -1.0, 0.5));
  cvsFeatures[6].appendVector(xAxis);
  cvsFeatures[6].appendVector(yAxis);
  cvsFeatures[6].appendVector(-zAxis);
  cvsFeatures[6].compute();

  cvsFeatures[7] = radi::CVSFeature(pcl::PointXYZ(-0.5, -1.0, -0.5));
  cvsFeatures[7].appendVector(xAxis);
  cvsFeatures[7].appendVector(yAxis);
  cvsFeatures[7].appendVector(zAxis);
  cvsFeatures[7].compute();

  // const std::vector<float> & angles = cvsFeatures[7].getIncludedAngles();
  // std::cout << angles[0] << "  " << angles[1] << "  " << angles[2] << std::endl;

  //
  // Load pcd file.
  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneRaw(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>("cuboid00000.pcd", *sceneRaw) == -1)
  {
    PCL_ERROR("Couldn't read file cuboid00000.pcd.\n");
    return -1;
  }

  //
  // Filtering & Downsampling.
  // Remove nan data.
  pcl::PointCloud<pcl::PointXYZ>::Ptr sceneNaNClean(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*sceneRaw, *sceneNaNClean, indices);
  pcl::io::savePCDFileASCII<pcl::PointXYZ>("cuboid_nan_clean.pcd", *sceneNaNClean);

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

  // // Visualize the downsampled point cloud.
  // pcl::visualization::PCLVisualizer viewer("Downsample");
  // viewer.addPointCloud(sceneDownSampled, "Scene Downsmapled");
  // while (!viewer.wasStopped()) {
  //     viewer.spinOnce();
  // }

  std::cout << "Number of points in the pcd file: " << sceneRaw->size() << std::endl;
  std::cout << "Number of points after filtering and downsampling: " << sceneDownSampled->size() << std::endl;

  std::vector<radi::CVSFeature> cvs_feature_list;
  radi::CVSEstimation cvs_estimation;
  cvs_estimation.setInputCloud(sceneDownSampled);
  cvs_estimation.esimate(cvs_feature_list);

  std::cout << "Number of cvs features: " << cvs_feature_list.size() << std::endl;

  // Corresponce group.
  radi::CVSCorrespGroup corresp_group;
  corresp_group.setSceneFeatures(&cvs_feature_list);
  corresp_group.setModelFeatures(&cvsFeatures);
  std::vector<Eigen::Matrix4f> mat_transf_list;
  corresp_group.recognize(mat_transf_list);

  std::cout << mat_transf_list.size() << std::endl;

  // Perform ICF algorithm. Refine the tramsformations.
  radi::IterativeClosestFace icf;
  icf.setScenePointCloud(sceneDownSampled);
  icf.setReferenceModel("cuboid.stl");
  std::cout << "Number of triangles: " << icf.getReferenceModel().getNumTriangles() << std::endl;
  for (std::size_t i = 0; i < mat_transf_list.size(); ++i)
  {
    icf.setInitialTransformation(mat_transf_list[i]);
    Eigen::Matrix4f mat_transf;
    icf.estimate(mat_transf);
  }


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

